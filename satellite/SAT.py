"""
CSU33D03 Main Project - LEO Satellite Relay Node
Group 7

Runs on a SEPARATE machine (the "satellite" device).
Sits between the wind turbine and the space station, relaying
all traffic while applying realistic channel effects:
  - Propagation delay
  - Packet loss
  - Contact window simulation
  - Doppler jitter

Physical setup:
  Device 1 (Turbine)   → talks to → Device 2 (Satellite)
  Device 2 (Satellite) → talks to → Device 3 (Space Station)

Port layout on the satellite machine:
  6001 — Listens for traffic FROM the turbine   (UDP + TCP)
  6002 — Forwards traffic TO the space station  (UDP + TCP)
  6003 — Listens for commands FROM the station  (TCP)
  6004 — Forwards commands TO the turbine       (TCP)

Usage:
    python satellite/satellite_relay.py \
        --turbine-host  <turbine_ip>  \
        --station-host  <station_ip>  \
        --my-host       0.0.0.0
"""

import socket
import threading
import time
import logging
import argparse
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from protocol.protocol import parse_frame, MsgType, build_frame
from channel.channel_model import SatelliteChannelModel

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [SATELLITE] %(levelname)s %(message)s'
)
log = logging.getLogger('satellite')


class SatelliteRelayNode:
    """
    LEO Satellite relay — runs on its own physical device.

    Accepts connections from both the turbine and the space station,
    and applies the channel model (delay + loss) to every packet
    it forwards in either direction.
    """

    def __init__(
        self,
        turbine_host : str,
        station_host : str,
        my_host      : str = '0.0.0.0',
    ):
        self.turbine_host  = turbine_host
        self.station_host  = station_host
        self.my_host       = my_host

        # Ports the satellite LISTENS on
        self.PORT_FROM_TURBINE  = 6001   # turbine sends telemetry here
        self.PORT_FROM_STATION  = 6002   # station sends commands here
        self.PORT_DISCOVERY     = 6003   # discovery relay

        # Ports on the OTHER devices we FORWARD to
        self.TURBINE_CMD_PORT   = 5002   # turbine command port
        self.STATION_TELEM_PORT = 5001   # station telemetry listen port

        # Channel model — applied to ALL traffic in both directions
        self.uplink   = SatelliteChannelModel(   # turbine → station
            contact_window_s   = 60,
            gap_between_pass_s = 120,
            base_loss_rate     = 0.02,
            ocean_swell_factor = 0.5,
        )
        self.downlink = SatelliteChannelModel(   # station → turbine
            contact_window_s   = 60,
            gap_between_pass_s = 120,
            base_loss_rate     = 0.015,
            ocean_swell_factor = 0.2,
        )

        self._running  = False
        self._threads  = []

        # Stats
        self.packets_relayed_up   = 0
        self.packets_relayed_down = 0
        self.packets_dropped_up   = 0
        self.packets_dropped_down = 0

    # ── Lifecycle ──────────────────────────────────────────────────────────
    def start(self):
        self._running = True
        self.uplink.start()
        self.downlink.start()

        log.info(
            f"LEO Satellite Relay starting...\n"
            f"  Turbine  : {self.turbine_host}\n"
            f"  Station  : {self.station_host}\n"
            f"  Listening: {self.my_host}"
        )

        # Uplink: receive telemetry from turbine, forward to station
        self._spawn(self._uplink_telemetry_relay)

        # Downlink: receive commands from station, forward to turbine
        self._spawn(self._downlink_command_relay)

        # Discovery relay: pass DISCOVER probes between sides
        self._spawn(self._discovery_relay)

        # Status printer
        self._spawn(self._status_loop)

        log.info("Satellite relay ONLINE.")

    def stop(self):
        self._running = False
        self.uplink.stop()
        self.downlink.stop()
        log.info("Satellite relay OFFLINE.")

    def _spawn(self, fn):
        t = threading.Thread(target=fn, daemon=True)
        t.start()
        self._threads.append(t)

    # ── Uplink: Turbine → Satellite → Station (UDP telemetry) ─────────────
    def _uplink_telemetry_relay(self):
        """
        Listens for UDP telemetry from the turbine.
        Applies uplink channel effects, then forwards to the station.
        """
        # Receive socket (from turbine)
        recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        recv_sock.bind((self.my_host, self.PORT_FROM_TURBINE))
        recv_sock.settimeout(1.0)

        # Forward socket (to station)
        fwd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        log.info(f"[UPLINK] Telemetry relay listening on UDP:{self.PORT_FROM_TURBINE}")

        while self._running:
            try:
                data, addr = recv_sock.recvfrom(8192)
                msg = parse_frame(data)
                if not msg:
                    log.warning("[UPLINK] Corrupt frame from turbine — dropped")
                    continue

                log.debug(f"[UPLINK] Received {msg['msg_type'].name} from {addr}")

                # Apply uplink channel model
                delay, lost = self.uplink.transmit(data)

                if lost:
                    self.packets_dropped_up += 1
                    log.warning(f"[UPLINK] Packet DROPPED by channel (uplink loss)")
                    continue

                # Simulate propagation delay
                if delay > 0:
                    time.sleep(delay)

                # Forward to station's telemetry port
                fwd_sock.sendto(data, (self.station_host, self.STATION_TELEM_PORT))
                self.packets_relayed_up += 1
                log.debug(
                    f"[UPLINK] Relayed {msg['msg_type'].name} → "
                    f"{self.station_host}:{self.STATION_TELEM_PORT} "
                    f"(delay={delay*1000:.1f}ms)"
                )

            except socket.timeout:
                continue
            except Exception as e:
                log.error(f"[UPLINK] Error: {e}")

        recv_sock.close()
        fwd_sock.close()

    # ── Downlink: Station → Satellite → Turbine (TCP commands) ────────────
    def _downlink_command_relay(self):
        """
        Listens for TCP command connections from the station.
        Applies downlink channel effects, then proxies to turbine.
        """
        listen_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listen_sock.bind((self.my_host, self.PORT_FROM_STATION))
        listen_sock.listen(5)
        listen_sock.settimeout(1.0)

        log.info(f"[DOWNLINK] Command relay listening on TCP:{self.PORT_FROM_STATION}")

        while self._running:
            try:
                conn, addr = listen_sock.accept()
                log.info(f"[DOWNLINK] Station connected from {addr}")
                threading.Thread(
                    target=self._proxy_station_to_turbine,
                    args=(conn,),
                    daemon=True
                ).start()
            except socket.timeout:
                continue
            except Exception as e:
                log.error(f"[DOWNLINK] Accept error: {e}")

        listen_sock.close()

    def _proxy_station_to_turbine(self, station_conn: socket.socket):
        """
        Bidirectional TCP proxy between station and turbine,
        with downlink channel effects applied to station→turbine traffic.
        """
        turbine_conn = None
        try:
            # Connect to turbine command port
            turbine_conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            turbine_conn.settimeout(5.0)
            turbine_conn.connect((self.turbine_host, self.TURBINE_CMD_PORT))
            log.info(f"[DOWNLINK] Proxy connected to turbine at {self.turbine_host}:{self.TURBINE_CMD_PORT}")

            # Bidirectional relay in two threads
            stop_event = threading.Event()

            def relay_station_to_turbine():
                station_conn.settimeout(1.0)
                while not stop_event.is_set():
                    try:
                        data = station_conn.recv(4096)
                        if not data:
                            break

                        # Apply downlink channel
                        delay, lost = self.downlink.transmit(data)
                        if lost:
                            self.packets_dropped_down += 1
                            log.warning("[DOWNLINK] Command DROPPED by channel")
                            continue

                        if delay > 0:
                            time.sleep(delay)

                        turbine_conn.sendall(data)
                        self.packets_relayed_down += 1

                        msg = parse_frame(data)
                        if msg:
                            log.info(
                                f"[DOWNLINK] Relayed {msg['msg_type'].name} "
                                f"→ turbine (delay={delay*1000:.1f}ms)"
                            )
                    except socket.timeout:
                        continue
                    except Exception:
                        break
                stop_event.set()

            def relay_turbine_to_station():
                turbine_conn.settimeout(1.0)
                while not stop_event.is_set():
                    try:
                        data = turbine_conn.recv(4096)
                        if not data:
                            break

                        # Return path (ACKs etc) — apply uplink channel
                        delay, lost = self.uplink.transmit(data)
                        if lost:
                            log.warning("[UPLINK] ACK DROPPED by channel")
                            continue

                        if delay > 0:
                            time.sleep(delay)

                        station_conn.sendall(data)

                        msg = parse_frame(data)
                        if msg:
                            log.info(
                                f"[UPLINK] Relayed {msg['msg_type'].name} "
                                f"→ station (delay={delay*1000:.1f}ms)"
                            )
                    except socket.timeout:
                        continue
                    except Exception:
                        break
                stop_event.set()

            t1 = threading.Thread(target=relay_station_to_turbine, daemon=True)
            t2 = threading.Thread(target=relay_turbine_to_station, daemon=True)
            t1.start()
            t2.start()
            t1.join()
            t2.join()

        except Exception as e:
            log.error(f"[DOWNLINK] Proxy error: {e}")
        finally:
            if turbine_conn:
                turbine_conn.close()
            station_conn.close()
            log.info("[DOWNLINK] Proxy session ended.")

    # ── Discovery relay (UDP) ──────────────────────────────────────────────
    def _discovery_relay(self):
        """
        Relays DISCOVER and DISCOVER_RESP packets between turbine and station,
        so discovery works across the satellite hop.
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.my_host, self.PORT_DISCOVERY))
        sock.settimeout(1.0)

        fwd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        log.info(f"[DISCOVERY] Relay listening on UDP:{self.PORT_DISCOVERY}")

        while self._running:
            try:
                data, addr = sock.recvfrom(4096)
                msg = parse_frame(data)
                if not msg:
                    continue

                if msg['msg_type'] == MsgType.DISCOVER:
                    # Forward to turbine discovery port
                    fwd.sendto(data, (self.turbine_host, 5004))
                    log.info(f"[DISCOVERY] Relayed DISCOVER → turbine")

                elif msg['msg_type'] == MsgType.DISCOVER_RESP:
                    # Forward back to station discovery port
                    fwd.sendto(data, (self.station_host, 5004))
                    log.info(f"[DISCOVERY] Relayed DISCOVER_RESP → station")

            except socket.timeout:
                continue
            except Exception as e:
                log.error(f"[DISCOVERY] Relay error: {e}")

        sock.close()
        fwd.close()

    # ── Status loop ────────────────────────────────────────────────────────
    def _status_loop(self):
        while self._running:
            time.sleep(10)
            up   = self.uplink.status()
            down = self.downlink.status()
            log.info(
                f"[STATUS] "
                f"Contact={'UP' if up['in_contact'] else 'DOWN'} | "
                f"Uplink loss={up['effective_loss_pct']}% | "
                f"Downlink loss={down['effective_loss_pct']}% | "
                f"Relayed ↑{self.packets_relayed_up} ↓{self.packets_relayed_down} | "
                f"Dropped ↑{self.packets_dropped_up} ↓{self.packets_dropped_down}"
            )


# ── Entry point ────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='LEO Satellite Relay Node')
    parser.add_argument('--turbine-host', required=True, help='IP of the turbine device')
    parser.add_argument('--station-host', required=True, help='IP of the space station device')
    parser.add_argument('--my-host',      default='0.0.0.0', help='Local bind address')
    args = parser.parse_args()

    relay = SatelliteRelayNode(
        turbine_host=args.turbine_host,
        station_host=args.station_host,
        my_host=args.my_host,
    )
    relay.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        relay.stop()
        log.info("Satellite relay shut down.")