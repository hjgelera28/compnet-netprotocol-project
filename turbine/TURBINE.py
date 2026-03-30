"""
CSU33D03 Main Project - Wind Turbine Simulator
Group 7-9

Simulates an offshore wind turbine with:
  - Realistic sensor emulation (wind speed, power, temp, RPM, blade load)
  - Actuator control: blade pitch and nacelle yaw
  - Autonomous fallback control when comms are lost
  - 4 separate socket servers on different ports (mandatory requirement)
  - At least 2 using raw TCP socket-level code

Port allocation:
  5001 — Telemetry broadcast      (UDP, raw socket)
  5002 — Command receiver         (TCP, raw socket)
  5003 — Sensor polling service   (TCP, high-level)
  5004 — Discovery / heartbeat    (UDP, high-level)
"""

import socket
import threading
import time
import random
import math
import logging
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from protocol.protocol import (
    parse_frame, MsgType,
    make_hello_ack, make_telemetry, make_cmd_ack, make_cmd_nack,
    make_sensor_resp, make_pong, make_discover_resp,
)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [TURBINE] %(levelname)s %(message)s'
)
log = logging.getLogger('turbine')


# ── Turbine physical constants ─────────────────────────────────────────────────
PITCH_MIN_DEG   = -5.0     # fine pitch (max power)
PITCH_MAX_DEG   = 90.0     # feather (emergency stop)
YAW_MIN_DEG     = 0.0
YAW_MAX_DEG     = 360.0
RATED_WIND_MS   = 12.0     # rated wind speed m/s
CUT_IN_WIND_MS  = 3.0      # minimum operating wind speed
CUT_OUT_WIND_MS = 25.0     # storm cut-out speed


# ── Sensor simulation ──────────────────────────────────────────────────────────
class TurbineSensors:
    """
    Generates realistic, time-varying sensor readings.
    Values slowly drift with Gaussian noise to simulate real conditions.
    """

    def __init__(self, node_id: str):
        self.node_id = node_id
        self._t          = 0.0
        self._wind_base  = 10.0  # m/s base wind speed
        self._lock       = threading.Lock()

        # Current state
        self.wind_speed_ms   = self._wind_base
        self.power_kw        = 0.0
        self.rotor_rpm       = 0.0
        self.nacelle_temp_c  = 35.0
        self.blade_load_pct  = 0.0
        self.pitch_deg       = 15.0   # controlled externally
        self.yaw_deg         = 180.0  # controlled externally
        self.vibration_ms2   = 0.0
        self.sea_state_m     = 1.5    # wave height metres

    def update(self):
        """Call periodically to advance sensor state."""
        with self._lock:
            self._t += 0.5

            # Wind: slow sinusoidal variation + Gaussian noise
            self.wind_speed_ms = max(0.0, (
                self._wind_base
                + 3.0 * math.sin(self._t / 60)        # slow gust cycle
                + 1.5 * math.sin(self._t / 8)          # faster fluctuation
                + random.gauss(0, 0.3)
            ))

            # Rotor RPM follows wind (simplified Betz law approximation)
            if self.wind_speed_ms < CUT_IN_WIND_MS:
                self.rotor_rpm = 0.0
            elif self.wind_speed_ms > CUT_OUT_WIND_MS:
                self.rotor_rpm = 0.0
            else:
                # Max RPM ~16, scaled by wind
                self.rotor_rpm = min(16.0, self.wind_speed_ms * 1.3) + random.gauss(0, 0.1)

            # Power output (cubic relationship up to rated wind)
            if self.rotor_rpm > 0:
                wind_ratio = min(self.wind_speed_ms / RATED_WIND_MS, 1.0)
                self.power_kw = 5000 * (wind_ratio ** 3) + random.gauss(0, 20)
            else:
                self.power_kw = 0.0

            # Nacelle temperature: rises with load
            target_temp = 35.0 + self.power_kw / 250
            self.nacelle_temp_c += (target_temp - self.nacelle_temp_c) * 0.05
            self.nacelle_temp_c += random.gauss(0, 0.2)

            # Blade load (% of max rated)
            self.blade_load_pct = min(100, (self.power_kw / 5000) * 100 + random.gauss(0, 1))

            # Vibration increases with RPM and sea state
            self.vibration_ms2 = (
                self.rotor_rpm * 0.05
                + self.sea_state_m * 0.3
                + random.gauss(0, 0.05)
            )

            # Sea state slow drift
            self.sea_state_m = max(0.2, self.sea_state_m + random.gauss(0, 0.01))

    def snapshot(self) -> dict:
        with self._lock:
            return {
                'wind_speed_ms'  : round(self.wind_speed_ms,  2),
                'power_kw'       : round(self.power_kw,        1),
                'rotor_rpm'      : round(self.rotor_rpm,        2),
                'nacelle_temp_c' : round(self.nacelle_temp_c,  1),
                'blade_load_pct' : round(self.blade_load_pct,  1),
                'pitch_deg'      : round(self.pitch_deg,        1),
                'yaw_deg'        : round(self.yaw_deg,          1),
                'vibration_ms2'  : round(self.vibration_ms2,   3),
                'sea_state_m'    : round(self.sea_state_m,      2),
            }


# ── Autonomous local controller ────────────────────────────────────────────────
class AutonomousController:
    """
    Safety-critical local controller.
    Runs always. Overrides remote commands if safety limits are exceeded.
    Becomes primary controller when satellite comms are lost.
    """

    def __init__(self, sensors: TurbineSensors):
        self.sensors   = sensors
        self.comms_ok  = True
        self._last_remote_cmd = time.time()
        self._comms_timeout_s = 30  # assume comms lost after 30s silence

    def check_comms_timeout(self):
        if time.time() - self._last_remote_cmd > self._comms_timeout_s:
            if self.comms_ok:
                log.warning("[AUTONOMOUS] Remote comms timeout — switching to autonomous mode.")
            self.comms_ok = False
        else:
            self.comms_ok = True

    def mark_remote_cmd_received(self):
        self._last_remote_cmd = time.time()

    def safe_pitch(self, requested_deg: float) -> tuple[float, bool]:
        """Validate and clamp pitch command. Returns (actual_deg, accepted)."""
        snap = self.sensors.snapshot()

        # Emergency feather if wind too high
        if snap['wind_speed_ms'] > CUT_OUT_WIND_MS:
            log.warning("[AUTONOMOUS] High wind — forcing feather pitch=90°")
            return 90.0, False

        clamped = max(PITCH_MIN_DEG, min(PITCH_MAX_DEG, requested_deg))
        if clamped != requested_deg:
            log.info(f"[AUTONOMOUS] Pitch clamped {requested_deg}→{clamped}")
        return clamped, True

    def safe_yaw(self, requested_deg: float) -> tuple[float, bool]:
        """Validate and normalise yaw command."""
        normalised = requested_deg % 360.0
        return normalised, True

    def autonomous_step(self):
        """
        Called when remote comms unavailable.
        Simple MPPT-like logic: adjust pitch to track wind.
        """
        if self.comms_ok:
            return

        snap = self.sensors.snapshot()
        wind = snap['wind_speed_ms']

        if wind < CUT_IN_WIND_MS:
            target_pitch = 90.0   # feather — not enough wind
        elif wind > CUT_OUT_WIND_MS:
            target_pitch = 90.0   # feather — too much wind (safety)
        else:
            # Simplified: pitch tracks wind linearly between 0–15°
            target_pitch = 15.0 - ((wind - CUT_IN_WIND_MS) / (CUT_OUT_WIND_MS - CUT_IN_WIND_MS)) * 15.0

        with self.sensors._lock:
            self.sensors.pitch_deg = target_pitch
        log.info(f"[AUTONOMOUS] Auto-pitch={target_pitch:.1f}° wind={wind:.1f}m/s")


# ── Main Turbine Node ──────────────────────────────────────────────────────────
class WindTurbineNode:
    """
    Offshore wind turbine communication node.

    Runs 4 socket servers:
      Port 5001 — UDP telemetry broadcast (raw socket)
      Port 5002 — TCP command receiver    (raw socket)
      Port 5003 — TCP sensor polling      (socket server)
      Port 5004 — UDP discovery/heartbeat (socket server)
    """

    def __init__(self, node_id: str = 'TURBINE-G8-01', group: int = 8,
                 host: str = '0.0.0.0'):
        self.node_id   = node_id
        self.group     = group
        self.host      = host
        self.sensors   = TurbineSensors(node_id)
        self.auto_ctrl = AutonomousController(self.sensors)
        self._running  = False
        self._threads  = []

        # Ports
        self.PORT_TELEMETRY   = 5001
        self.PORT_COMMANDS    = 5002
        self.PORT_SENSOR_POLL = 5003
        self.PORT_DISCOVERY   = 5004

    # ── Start / stop ───────────────────────────────────────────────────────
    def start(self):
        self._running = True
        log.info(f"WindTurbineNode '{self.node_id}' starting on {self.host}...")

        # Sensor update loop
        self._spawn(self._sensor_loop)

        # Port 5001: UDP telemetry broadcast (raw socket) ← MANDATORY raw socket 1
        self._spawn(self._telemetry_server)

        # Port 5002: TCP command receiver (raw socket) ← MANDATORY raw socket 2
        self._spawn(self._command_server)

        # Port 5003: TCP sensor polling (socket module)
        self._spawn(self._sensor_poll_server)

        # Port 5004: UDP discovery heartbeat
        self._spawn(self._discovery_server)

        log.info("All 4 socket services started.")

    def stop(self):
        self._running = False
        log.info("WindTurbineNode stopping.")

    def _spawn(self, fn):
        t = threading.Thread(target=fn, daemon=True)
        t.start()
        self._threads.append(t)

    # ── Sensor update loop ─────────────────────────────────────────────────
    def _sensor_loop(self):
        while self._running:
            self.sensors.update()
            self.auto_ctrl.check_comms_timeout()
            self.auto_ctrl.autonomous_step()
            time.sleep(0.5)

    # ── Port 5001: UDP Telemetry Broadcast (RAW SOCKET) ───────────────────
    def _telemetry_server(self):
        """
        Broadcasts telemetry via UDP every 2 seconds.
        Uses raw socket-level API — no high-level wrappers.
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind((self.host, self.PORT_TELEMETRY))
        sock.settimeout(1.0)
        log.info(f"[PORT {self.PORT_TELEMETRY}] UDP telemetry broadcast READY")

        while self._running:
            snap   = self.sensors.snapshot()
            frame  = make_telemetry(self.node_id, snap)
            try:
                # Broadcast to subnet
                sock.sendto(frame, ('<broadcast>', self.PORT_TELEMETRY))
                log.debug(f"[PORT {self.PORT_TELEMETRY}] Telemetry broadcast sent ({len(frame)} bytes)")
            except OSError as e:
                log.error(f"[PORT {self.PORT_TELEMETRY}] Send error: {e}")
            time.sleep(2.0)

        sock.close()

    # ── Port 5002: TCP Command Receiver (RAW SOCKET) ──────────────────────
    def _command_server(self):
        """
        Listens for pitch/yaw/other commands from the space station.
        Raw TCP socket implementation with handshaking.
        """
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.PORT_COMMANDS))
        srv.listen(5)
        srv.settimeout(1.0)
        log.info(f"[PORT {self.PORT_COMMANDS}] TCP command receiver READY")

        while self._running:
            try:
                conn, addr = srv.accept()
                log.info(f"[PORT {self.PORT_COMMANDS}] Connection from {addr}")
                t = threading.Thread(
                    target=self._handle_command_conn,
                    args=(conn, addr),
                    daemon=True
                )
                t.start()
            except socket.timeout:
                continue
            except OSError as e:
                log.error(f"[PORT {self.PORT_COMMANDS}] Accept error: {e}")

        srv.close()

    def _handle_command_conn(self, conn: socket.socket, addr):
        conn.settimeout(10.0)
        try:
            # Handshake: expect HELLO
            raw = conn.recv(4096)
            if not raw:
                return

            msg = parse_frame(raw)
            if msg and msg['msg_type'] == MsgType.HELLO:
                ack = make_hello_ack(self.node_id, accepted=True)
                conn.sendall(ack)
                log.info(f"[CMD] Handshake OK with {msg['payload'].get('node_id','?')}")
            else:
                log.warning(f"[CMD] Bad handshake from {addr}")
                return

            # Command loop
            while self._running:
                try:
                    raw = conn.recv(4096)
                    if not raw:
                        break
                except socket.timeout:
                    continue

                msg = parse_frame(raw)
                if not msg:
                    log.warning("[CMD] Corrupt frame received")
                    continue

                self.auto_ctrl.mark_remote_cmd_received()
                self._dispatch_command(conn, msg)

        except (ConnectionResetError, BrokenPipeError):
            log.info(f"[CMD] Connection closed by {addr}")
        except Exception as e:
            log.error(f"[CMD] Error: {e}")
        finally:
            conn.close()

    def _dispatch_command(self, conn: socket.socket, msg: dict):
        mt      = msg['msg_type']
        payload = msg['payload']
        seq     = msg['seq']

        if mt == MsgType.CMD_PITCH:
            requested = payload.get('pitch_deg', 0)
            actual, accepted = self.auto_ctrl.safe_pitch(requested)
            with self.sensors._lock:
                self.sensors.pitch_deg = actual
            if accepted:
                log.info(f"[CMD] Pitch set to {actual}°")
                conn.sendall(make_cmd_ack(seq, self.node_id))
            else:
                log.warning(f"[CMD] Pitch {requested}° rejected by safety controller")
                conn.sendall(make_cmd_nack(seq, "safety_override"))

        elif mt == MsgType.CMD_YAW:
            requested = payload.get('yaw_deg', 0)
            actual, _ = self.auto_ctrl.safe_yaw(requested)
            with self.sensors._lock:
                self.sensors.yaw_deg = actual
            log.info(f"[CMD] Yaw set to {actual}°")
            conn.sendall(make_cmd_ack(seq, self.node_id))

        elif mt == MsgType.PING:
            conn.sendall(make_pong(self.node_id, payload.get('sent_at', time.time())))

        else:
            log.debug(f"[CMD] Unhandled msg_type={mt}")

    # ── Port 5003: TCP Sensor Polling ──────────────────────────────────────
    def _sensor_poll_server(self):
        """
        On-demand sensor queries from the station.
        Responds with specific sensor value.
        """
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.PORT_SENSOR_POLL))
        srv.listen(5)
        srv.settimeout(1.0)
        log.info(f"[PORT {self.PORT_SENSOR_POLL}] TCP sensor polling READY")

        while self._running:
            try:
                conn, addr = srv.accept()
                threading.Thread(
                    target=self._handle_sensor_poll,
                    args=(conn,),
                    daemon=True
                ).start()
            except socket.timeout:
                continue

        srv.close()

    def _handle_sensor_poll(self, conn: socket.socket):
        conn.settimeout(5.0)
        try:
            raw = conn.recv(4096)
            if not raw:
                return
            msg = parse_frame(raw)
            if not msg or msg['msg_type'] != MsgType.SENSOR_REQ:
                return

            sensor_name = msg['payload'].get('sensor', '')
            snap = self.sensors.snapshot()
            units = {
                'wind_speed_ms' : 'm/s',
                'power_kw'      : 'kW',
                'rotor_rpm'     : 'RPM',
                'nacelle_temp_c': '°C',
                'blade_load_pct': '%',
                'pitch_deg'     : '°',
                'yaw_deg'       : '°',
                'vibration_ms2' : 'm/s²',
                'sea_state_m'   : 'm',
            }
            value = snap.get(sensor_name, None)
            unit  = units.get(sensor_name, '')

            if value is not None:
                conn.sendall(make_sensor_resp(sensor_name, value, unit))
                log.debug(f"[SENSOR POLL] {sensor_name}={value}{unit}")
            else:
                log.warning(f"[SENSOR POLL] Unknown sensor: {sensor_name}")
        except Exception as e:
            log.error(f"[SENSOR POLL] Error: {e}")
        finally:
            conn.close()

    # ── Port 5004: UDP Discovery / Heartbeat ───────────────────────────────
    def _discovery_server(self):
        """
        Responds to DISCOVER broadcasts from other nodes.
        Also sends periodic heartbeats.
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind((self.host, self.PORT_DISCOVERY))
        sock.settimeout(1.0)
        log.info(f"[PORT {self.PORT_DISCOVERY}] UDP discovery READY")

        services = [
            {'port': self.PORT_TELEMETRY,   'proto': 'UDP', 'service': 'telemetry'},
            {'port': self.PORT_COMMANDS,     'proto': 'TCP', 'service': 'commands'},
            {'port': self.PORT_SENSOR_POLL,  'proto': 'TCP', 'service': 'sensor_poll'},
            {'port': self.PORT_DISCOVERY,    'proto': 'UDP', 'service': 'discovery'},
        ]

        last_heartbeat = 0
        while self._running:
            # Heartbeat every 10s
            if time.time() - last_heartbeat > 10:
                hb = make_discover_resp(self.node_id, self.group, services)
                try:
                    sock.sendto(hb, ('<broadcast>', self.PORT_DISCOVERY))
                except OSError:
                    pass
                last_heartbeat = time.time()

            # Listen for discovery requests
            try:
                data, addr = sock.recvfrom(4096)
                msg = parse_frame(data)
                if msg and msg['msg_type'] == MsgType.DISCOVER:
                    resp = make_discover_resp(self.node_id, self.group, services)
                    sock.sendto(resp, addr)
                    log.info(f"[DISCOVERY] Responded to {addr}")
            except socket.timeout:
                continue
            except OSError as e:
                log.error(f"[DISCOVERY] Error: {e}")

        sock.close()


# ── Entry point ────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Wind Turbine Node')
    parser.add_argument('--id',    default='TURBINE-G7-01', help='Node ID')
    parser.add_argument('--group', type=int, default=7,     help='Group number')
    parser.add_argument('--host',  default='0.0.0.0',       help='Bind address')
    args = parser.parse_args()

    turbine = WindTurbineNode(node_id=args.id, group=args.group, host=args.host)
    turbine.start()

    try:
        while True:
            snap = turbine.sensors.snapshot()
            log.info(
                f"[STATUS] Wind={snap['wind_speed_ms']}m/s  "
                f"Power={snap['power_kw']}kW  "
                f"RPM={snap['rotor_rpm']}  "
                f"Pitch={snap['pitch_deg']}°  "
                f"Yaw={snap['yaw_deg']}°  "
                f"Comms={'OK' if turbine.auto_ctrl.comms_ok else 'LOST'}"
            )
            time.sleep(5)
    except KeyboardInterrupt:
        turbine.stop()
        log.info("Turbine shut down.")