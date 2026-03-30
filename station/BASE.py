"""
CSU33D03 Main Project - Space Station Control Centre
Group 7-9

Controls the offshore wind turbine from LEO orbit via satellite link.

Responsibilities:
  - Connects to turbine command port (TCP, raw socket)
  - Listens to telemetry broadcast (UDP)
  - Polls specific sensors on demand (TCP)
  - Sends DISCOVER probes to find turbines (UDP)
  - Reliability: ACK tracking, retransmit with exponential backoff
  - Channel-aware: routes through channel model for realistic delay/loss
"""

import socket
import threading
import time
import logging
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from protocol.protocol import (
    parse_frame, MsgType,
    make_hello, make_cmd_pitch, make_cmd_yaw,
    make_sensor_req, make_ping, make_discover,
)
from channel.channel_model import SatelliteChannelModel

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [STATION] %(levelname)s %(message)s'
)
log = logging.getLogger('station')


# ── Reliability Layer ──────────────────────────────────────────────────────────
class ReliableTransport:
    """
    Wraps a TCP socket with:
      - Sequence tracking
      - ACK waiting with timeout
      - Exponential backoff retransmit (up to MAX_RETRIES)
    """

    MAX_RETRIES     = 4
    BASE_TIMEOUT_S  = 2.0

    def __init__(self, sock: socket.socket, channel: SatelliteChannelModel = None):
        self.sock    = sock
        self.channel = channel
        self._lock   = threading.Lock()

    def send_reliable(self, frame: bytes, expect_ack_type: MsgType = None) -> dict | None:
        """
        Send a frame and wait for an ACK.
        Returns the ACK message dict, or None if all retries failed.
        """
        delay, lost = self.channel.transmit(frame) if self.channel else (0.0, False)

        for attempt in range(self.MAX_RETRIES):
            timeout = self.BASE_TIMEOUT_S * (2 ** attempt)

            if lost and self.channel:
                log.warning(f"[RELIABLE] Packet lost in channel (attempt {attempt+1})")
                delay, lost = self.channel.transmit(frame)
                continue

            try:
                with self._lock:
                    # Simulate propagation delay before the frame arrives
                    if delay > 0:
                        log.debug(f"[RELIABLE] Simulating {delay*1000:.1f}ms channel delay")
                        time.sleep(delay)
                    self.sock.sendall(frame)

                # Wait for ACK
                if expect_ack_type:
                    self.sock.settimeout(timeout)
                    try:
                        raw = self.sock.recv(4096)
                        if raw:
                            msg = parse_frame(raw)
                            if msg and msg['msg_type'] == expect_ack_type:
                                log.debug(f"[RELIABLE] ACK received after {attempt+1} attempt(s)")
                                return msg
                    except socket.timeout:
                        log.warning(
                            f"[RELIABLE] Timeout waiting for {expect_ack_type.name} "
                            f"(attempt {attempt+1}/{self.MAX_RETRIES})"
                        )
                else:
                    return {}   # fire-and-forget

            except (BrokenPipeError, ConnectionResetError) as e:
                log.error(f"[RELIABLE] Connection lost: {e}")
                return None

            # Retry with backoff
            if self.channel:
                delay, lost = self.channel.transmit(frame)

        log.error(f"[RELIABLE] All {self.MAX_RETRIES} retransmits failed.")
        return None


# ── Space Station Control Centre ───────────────────────────────────────────────
class SpaceStationController:
    """
    Orbital control centre for the wind turbine.

    Call start() to begin background listeners, then use the command
    methods (send_pitch, send_yaw, poll_sensor) to interact with the turbine.
    """

    def __init__(
        self,
        turbine_host   : str = '127.0.0.1',
        node_id        : str = 'STATION-G7-01',
        group          : int = 7,
        compressed_time: bool = True,
        satellite_host : str = None,   # if set, route via satellite relay
    ):
        self.turbine_host   = turbine_host
        self.satellite_host = satellite_host   # None = direct (dev/test mode)
        self.node_id        = node_id
        self.group          = group

        # In 3-device mode the channel model lives in the satellite relay.
        # We still keep a local lightweight copy for display/status purposes only.
        self.channel        = SatelliteChannelModel(
            contact_window_s   = 60 if compressed_time else 600,
            gap_between_pass_s = 120 if compressed_time else 5400,
        )

        # Command host: satellite relay if provided, otherwise turbine directly
        self._cmd_host = satellite_host if satellite_host else turbine_host

        # Ports — turbine native ports
        self.PORT_TELEMETRY   = 5001
        self.PORT_COMMANDS    = 5002
        self.PORT_SENSOR_POLL = 5003
        self.PORT_DISCOVERY   = 5004

        # Satellite relay ports (used when satellite_host is set)
        self.SAT_CMD_PORT     = 6002   # station sends commands to satellite here
        self.SAT_DISC_PORT    = 6003   # discovery via satellite

        self._cmd_sock        = None
        self._transport       = None
        self._running         = False
        self._threads         = []
        self._cmd_port        = self.SAT_CMD_PORT if satellite_host else self.PORT_COMMANDS

        # Latest received telemetry
        self.last_telemetry   : dict = {}
        self.discovered_nodes : list = []

        # Telemetry history for display
        self._telemetry_log   : list = []

    # ── Lifecycle ──────────────────────────────────────────────────────────
    def start(self):
        self._running = True
        self.channel.start()
        log.info(f"SpaceStation '{self.node_id}' starting...")

        self._spawn(self._telemetry_listener)
        self._spawn(self._discovery_listener)
        self._connect_command_socket()

        log.info("Station online. Waiting for satellite contact window...")

    def stop(self):
        self._running = False
        self.channel.stop()
        if self._cmd_sock:
            try:
                self._cmd_sock.close()
            except OSError:
                pass
        log.info("Station stopped.")

    def _spawn(self, fn):
        t = threading.Thread(target=fn, daemon=True)
        t.start()
        self._threads.append(t)

    # ── Command socket setup (RAW TCP) ─────────────────────────────────────
    def _connect_command_socket(self):
        """Opens and handshakes the TCP command connection to the turbine."""
        retries = 0
        while self._running:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect((self._cmd_host, self._cmd_port))

                # Handshake
                hello = make_hello(self.node_id, 'station', self.group)
                sock.sendall(hello)
                raw = sock.recv(4096)
                msg = parse_frame(raw)

                if msg and msg['msg_type'] == MsgType.HELLO_ACK and msg['payload'].get('accepted'):
                    self._cmd_sock   = sock
                    # In 3-device mode, channel effects are applied by satellite relay
                    # so we pass channel=None here to avoid double-applying delay
                    chan = None if self.satellite_host else self.channel
                    self._transport  = ReliableTransport(sock, chan)
                    log.info(f"[CMD] Connected via {'satellite relay' if self.satellite_host else 'direct'} at {self._cmd_host}:{self._cmd_port}")
                    return
                else:
                    log.warning("[CMD] Handshake rejected. Retrying...")
                    sock.close()

            except (ConnectionRefusedError, socket.timeout, OSError) as e:
                retries += 1
                wait = min(2 ** retries, 30)
                log.warning(f"[CMD] Cannot connect to turbine ({e}). Retry in {wait}s...")
                time.sleep(wait)

    # ── Telemetry listener (UDP, raw socket) ──────────────────────────────
    def _telemetry_listener(self):
        """Receives UDP telemetry broadcasts from the turbine."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', self.PORT_TELEMETRY))
        sock.settimeout(1.0)
        log.info(f"[TELEMETRY] Listening on UDP:{self.PORT_TELEMETRY}")

        while self._running:
            try:
                data, addr = sock.recvfrom(8192)
                msg = parse_frame(data)
                if msg and msg['msg_type'] == MsgType.TELEMETRY:
                    self.last_telemetry = msg['payload']['sensors']
                    self._telemetry_log.append({
                        'time'   : time.time(),
                        'sensors': self.last_telemetry,
                    })
                    if len(self._telemetry_log) > 500:
                        self._telemetry_log.pop(0)

                    ch = self.channel.status()
                    log.debug(
                        f"[TELEMETRY] Wind={self.last_telemetry.get('wind_speed_ms')}m/s "
                        f"Power={self.last_telemetry.get('power_kw')}kW "
                        f"Channel={'UP' if ch['in_contact'] else 'DOWN'}"
                    )
            except socket.timeout:
                continue
            except Exception as e:
                log.error(f"[TELEMETRY] Error: {e}")

        sock.close()

    # ── Discovery listener (UDP) ───────────────────────────────────────────
    def _discovery_listener(self):
        """Listens for turbine heartbeats and discover responses."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', self.PORT_DISCOVERY))
        sock.settimeout(1.0)
        log.info(f"[DISCOVERY] Listening on UDP:{self.PORT_DISCOVERY}")

        while self._running:
            try:
                data, addr = sock.recvfrom(4096)
                msg = parse_frame(data)
                if msg and msg['msg_type'] == MsgType.DISCOVER_RESP:
                    node_id = msg['payload'].get('node_id', '?')
                    if node_id not in [n['node_id'] for n in self.discovered_nodes]:
                        self.discovered_nodes.append({
                            'node_id' : node_id,
                            'group'   : msg['payload'].get('group'),
                            'addr'    : addr[0],
                            'services': msg['payload'].get('services', []),
                            'seen_at' : time.time(),
                        })
                        log.info(f"[DISCOVERY] New node: {node_id} @ {addr[0]}")
            except socket.timeout:
                continue
            except Exception as e:
                log.error(f"[DISCOVERY] Error: {e}")

        sock.close()

    # ── Public command API ─────────────────────────────────────────────────
    def send_pitch(self, angle_deg: float, target_node: str = None) -> bool:
        """Send a blade pitch command. Returns True if ACK received."""
        if not self._transport:
            log.error("Not connected to turbine.")
            return False

        target = target_node or 'TURBINE-G8-01'
        frame  = make_cmd_pitch(angle_deg, target)
        log.info(f"[CONTROL] Sending CMD_PITCH {angle_deg}° → {target}")
        result = self._transport.send_reliable(frame, expect_ack_type=MsgType.CMD_ACK)
        if result is not None:
            log.info(f"[CONTROL] Pitch {angle_deg}° acknowledged.")
            return True
        else:
            log.error(f"[CONTROL] Pitch command FAILED (no ACK).")
            return False

    def send_yaw(self, angle_deg: float, target_node: str = None) -> bool:
        """Send a nacelle yaw command. Returns True if ACK received."""
        if not self._transport:
            log.error("Not connected to turbine.")
            return False

        target = target_node or 'TURBINE-G8-01'
        frame  = make_cmd_yaw(angle_deg, target)
        log.info(f"[CONTROL] Sending CMD_YAW {angle_deg}° → {target}")
        result = self._transport.send_reliable(frame, expect_ack_type=MsgType.CMD_ACK)
        if result is not None:
            log.info(f"[CONTROL] Yaw {angle_deg}° acknowledged.")
            return True
        else:
            log.error(f"[CONTROL] Yaw command FAILED (no ACK).")
            return False

    def poll_sensor(self, sensor_name: str) -> float | None:
        """
        Request a specific sensor value from the turbine via TCP sensor poll port.
        Returns the value, or None on failure.
        """
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((self.turbine_host, self.PORT_SENSOR_POLL))

            frame = make_sensor_req(sensor_name, self.node_id)
            sock.sendall(frame)

            raw = sock.recv(4096)
            msg = parse_frame(raw)
            if msg and msg['msg_type'] == MsgType.SENSOR_RESP:
                value = msg['payload']['value']
                unit  = msg['payload']['unit']
                log.info(f"[SENSOR POLL] {sensor_name} = {value}{unit}")
                sock.close()
                return value

            sock.close()
        except Exception as e:
            log.error(f"[SENSOR POLL] Failed for {sensor_name}: {e}")
        return None

    def ping_turbine(self) -> float | None:
        """Ping turbine and return round-trip time in ms, or None."""
        if not self._transport:
            return None
        frame = make_ping(self.node_id)
        t0    = time.time()
        result = self._transport.send_reliable(frame, expect_ack_type=MsgType.PONG)
        if result is not None:
            rtt = (time.time() - t0) * 1000
            log.info(f"[PING] RTT = {rtt:.1f}ms")
            return rtt
        return None

    def send_discover(self):
        """Broadcast a DISCOVER probe to find nearby turbines."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        frame = make_discover(self.node_id, self.group)
        sock.sendto(frame, ('<broadcast>', self.PORT_DISCOVERY))
        sock.close()
        log.info("[DISCOVERY] Sent DISCOVER broadcast.")

    # ── Status ─────────────────────────────────────────────────────────────
    def status(self) -> dict:
        return {
            'station_id'      : self.node_id,
            'channel'         : self.channel.status(),
            'last_telemetry'  : self.last_telemetry,
            'discovered_nodes': self.discovered_nodes,
            'connected'       : self._cmd_sock is not None,
        }


# ── Entry point ────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Space Station Controller')
    parser.add_argument('--turbine-host',   default='127.0.0.1')
    parser.add_argument('--satellite-host', default=None, help='IP of satellite relay (3-device mode)')
    parser.add_argument('--id',             default='STATION-G7-01')
    parser.add_argument('--group',          type=int, default=7)
    args = parser.parse_args()

    station = SpaceStationController(
        turbine_host=args.turbine_host,
        satellite_host=args.satellite_host,
        node_id=args.id,
        group=args.group,
    )
    station.start()

    # Wait for connection
    log.info("Waiting 5s for turbine connection...")
    time.sleep(5)

    try:
        cycle = 0
        while True:
            cycle += 1
            log.info(f"\n{'='*60}\nControl Cycle {cycle}\n{'='*60}")

            # Ping
            rtt = station.ping_turbine()
            log.info(f"Ping RTT: {rtt:.1f}ms" if rtt else "Ping failed.")

            # Poll sensors
            for sensor in ['wind_speed_ms', 'power_kw', 'nacelle_temp_c']:
                station.poll_sensor(sensor)

            # Send a pitch command
            pitch = [0, 5, 10, 15, 20, 45, 90][cycle % 7]
            station.send_pitch(pitch)

            # Send a yaw command
            yaw = (cycle * 30) % 360
            station.send_yaw(yaw)

            # Discover neighbours
            if cycle % 3 == 0:
                station.send_discover()

            status = station.status()
            ch     = status['channel']
            log.info(
                f"Channel: {'UP' if ch['in_contact'] else 'DOWN'} | "
                f"Loss: {ch['effective_loss_pct']}% | "
                f"Nodes discovered: {len(status['discovered_nodes'])}"
            )

            time.sleep(10)

    except KeyboardInterrupt:
        station.stop()
        log.info("Station shut down.")