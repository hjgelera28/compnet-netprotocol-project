"""
CSU33D03 Main Project - Terminal Dashboard
Group 7-9

Real-time terminal monitoring of the turbine and channel status.
Runs alongside the station controller, reading its shared state.

Usage:
    python dashboard.py --turbine-host 127.0.0.1 --group 8
"""

import time
import os
import sys
import threading
import argparse

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from station.station_controller import SpaceStationController


# ── ANSI colours ───────────────────────────────────────────────────────────────
R  = '\033[91m'
G  = '\033[92m'
Y  = '\033[93m'
B  = '\033[94m'
M  = '\033[95m'
C  = '\033[96m'
W  = '\033[97m'
DIM= '\033[2m'
RST= '\033[0m'
BOLD='\033[1m'


def clear():
    os.system('cls' if os.name == 'nt' else 'clear')


def bar(value, max_val, width=20, colour=G):
    filled = int((value / max(max_val, 1)) * width)
    filled = max(0, min(filled, width))
    return colour + '█' * filled + DIM + '░' * (width - filled) + RST


def fmt_float(v, decimals=1, unit=''):
    if v is None:
        return f'{DIM}N/A{RST}'
    return f'{W}{v:.{decimals}f}{DIM}{unit}{RST}'


def render(station: SpaceStationController):
    clear()
    status = station.status()
    ch     = status['channel']
    telem  = status['last_telemetry']
    now    = time.strftime('%H:%M:%S')

    contact_str = f"{G}● CONTACT{RST}" if ch['in_contact'] else f"{R}✕ NO SIGNAL{RST}"

    print(f"{BOLD}{B}╔══════════════════════════════════════════════════════════════╗{RST}")
    print(f"{BOLD}{B}║    CSU33D03 Wind Turbine Space Control  │  {W}{now}{B}          ║{RST}")
    print(f"{BOLD}{B}╚══════════════════════════════════════════════════════════════╝{RST}")
    print()

    # ── Satellite channel ──────────────────────────────────────────────────
    print(f"  {BOLD}{C}[ SATELLITE CHANNEL ]{RST}")
    print(f"  Status      : {contact_str}")
    if ch['in_contact']:
        elev  = ch.get('elevation_deg', 0) or 0
        delay = ch.get('one_way_delay_ms', 0) or 0
        print(f"  Elevation   : {fmt_float(elev, 1, '°')}  {bar(elev, 90, 20, C)}")
        print(f"  One-way RTT : {fmt_float(delay, 2, 'ms')}")
    print(f"  Loss rate   : {fmt_float(ch['effective_loss_pct'], 2, '%')}")
    print(f"  Pkts Sent   : {W}{ch['total_sent']}{RST}   Lost: {R}{ch['total_lost']}{RST}")
    print()

    # ── Turbine telemetry ──────────────────────────────────────────────────
    if telem:
        print(f"  {BOLD}{Y}[ TURBINE TELEMETRY ]{RST}")
        wind  = telem.get('wind_speed_ms', 0)
        power = telem.get('power_kw', 0)
        rpm   = telem.get('rotor_rpm', 0)
        temp  = telem.get('nacelle_temp_c', 0)
        pitch = telem.get('pitch_deg', 0)
        yaw   = telem.get('yaw_deg', 0)
        load  = telem.get('blade_load_pct', 0)
        vib   = telem.get('vibration_ms2', 0)
        sea   = telem.get('sea_state_m', 0)

        print(f"  Wind Speed  : {fmt_float(wind,  2, ' m/s')}  {bar(wind,  25, 20, C)}")
        print(f"  Power       : {fmt_float(power, 1, ' kW')}   {bar(power, 5000, 20, G)}")
        print(f"  Rotor RPM   : {fmt_float(rpm,   2, ' RPM')}  {bar(rpm,   16, 20, B)}")
        print(f"  Nacelle Temp: {fmt_float(temp,  1, ' °C')}   {bar(temp,  80, 20, R if temp > 60 else Y)}")
        print(f"  Blade Load  : {fmt_float(load,  1, ' %')}    {bar(load,  100, 20, R if load > 90 else Y)}")
        print(f"  Pitch Angle : {fmt_float(pitch, 1, '°')}")
        print(f"  Yaw Angle   : {fmt_float(yaw,   1, '°')}")
        print(f"  Vibration   : {fmt_float(vib,   3, ' m/s²')}")
        print(f"  Sea State   : {fmt_float(sea,   2, ' m')}")
    else:
        print(f"  {Y}Waiting for telemetry...{RST}")
    print()

    # ── Discovered nodes ───────────────────────────────────────────────────
    nodes = status['discovered_nodes']
    print(f"  {BOLD}{M}[ DISCOVERED NODES  ] ({len(nodes)}){RST}")
    if nodes:
        for n in nodes[-5:]:   # show last 5
            age = int(time.time() - n['seen_at'])
            print(f"  {G}•{RST} {W}{n['node_id']}{RST}  "
                  f"G{n['group']}  "
                  f"{n['addr']}  "
                  f"{DIM}{age}s ago{RST}")
    else:
        print(f"  {DIM}No nodes discovered yet. Broadcasting...{RST}")
    print()

    print(f"  {DIM}Station: {status['station_id']}  "
          f"Connected: {'Yes' if status['connected'] else 'No'}  "
          f"Press Ctrl+C to quit{RST}")


def run_dashboard(station: SpaceStationController):
    while True:
        try:
            render(station)
        except Exception as e:
            print(f"Dashboard error: {e}")
        time.sleep(2)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Wind Turbine Dashboard')
    parser.add_argument('--turbine-host',   default='127.0.0.1')
    parser.add_argument('--satellite-host', default=None)
    parser.add_argument('--group',          type=int, default=7)
    parser.add_argument('--id',             default='STATION-G7-01')
    args = parser.parse_args()

    station = SpaceStationController(
    turbine_host=args.turbine_host,
    satellite_host=args.satellite_host,
    node_id=args.id,
    group=args.group,
    )
    station.start()

    # Background control loop
    def control_loop():
        time.sleep(6)   # let connection establish
        cycle = 0
        while True:
            cycle += 1
            station.send_discover()
            station.send_pitch([5, 10, 15, 20, 45][cycle % 5])
            station.send_yaw((cycle * 45) % 360)
            time.sleep(15)

    threading.Thread(target=control_loop, daemon=True).start()

    try:
        run_dashboard(station)
    except KeyboardInterrupt:
        station.stop()
        print("\nDashboard closed.")
