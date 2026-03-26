# CSU33D03 Main Project — Wind Turbine Space Protocol
## Group 7–9

A custom networking protocol enabling communication from an offshore wind turbine
to a space station in Low Earth Orbit (LEO), via a simulated satellite link.

---

## Project Structure

```
wind_turbine_project/
│
├── protocol/
│   └── protocol.py         ← Custom binary protocol (framing, all msg types)
│
├── channel/
│   └── channel_model.py    ← LEO satellite channel: delay, loss, contact windows
│
├── turbine/
│   └── turbine_node.py     ← Wind turbine simulator (4 socket servers)
│
├── station/
│   └── station_controller.py ← Space station controller (commands + telemetry)
│
├── dashboard.py            ← Real-time terminal monitoring dashboard
└── README.md
```

---

## How to Run

### Requirements
- Python 3.10+
- No external libraries required (stdlib only)

### Step 1 — Start the Turbine (Terminal 1)
```bash
cd wind_turbine_project
python turbine/turbine_node.py --id TURBINE-G8-01 --group 8
```

### Step 2 — Start the Dashboard / Station (Terminal 2)
```bash
python dashboard.py --turbine-host 127.0.0.1 --group 8
```

Or run the station controller alone:
```bash
python station/station_controller.py --turbine-host 127.0.0.1
```

### Running on Raspberry Pi / separate machines
Change `--turbine-host` to the turbine machine's IP address:
```bash
python dashboard.py --turbine-host 192.168.x.x
```

---

## Architecture

### Protocol (`protocol/protocol.py`)
Custom binary framing protocol — **WTSP** (Wind Turbine Space Protocol).

Frame format:
```
[MAGIC 4B][VERSION 1B][MSG_TYPE 1B][SEQ 4B][TIMESTAMP 8B][PAYLOAD_LEN 4B][PAYLOAD NB][CHECKSUM 4B]
```

- Magic bytes: `WTSP`
- Checksum: first 4 bytes of SHA-256 (lightweight integrity)
- Payload: JSON-encoded dict (flexible, human-readable)
- All message types defined in `MsgType` enum

### Channel Model (`channel/channel_model.py`)
Realistic LEO satellite link simulation:

| Parameter | Value |
|-----------|-------|
| Altitude | 550 km (Starlink-style) |
| One-way delay | 1.8–8.8 ms (elevation dependent) |
| Contact window | 10 min per pass |
| Gap between passes | ~90 min orbit |
| Base packet loss | 2% |
| Extra loss near horizon | up to +15% |
| Ocean swell factor | ±5% additional loss |
| Burst fade events | 3% probability, +20% loss |

### Turbine Node — 4 Socket Services (`turbine/turbine_node.py`)

| Port | Protocol | Type | Service |
|------|----------|------|---------|
| 5001 | UDP | **Raw socket** | Telemetry broadcast (every 2s) |
| 5002 | TCP | **Raw socket** | Command receiver (pitch/yaw/ping) |
| 5003 | TCP | Socket | Sensor polling (on-demand queries) |
| 5004 | UDP | Socket | Discovery / heartbeat |

Sensors simulated: wind speed, power output, rotor RPM, nacelle temperature,
blade load, pitch angle, yaw angle, vibration, sea state.

**Autonomous controller**: if no remote command received for 30s, the turbine
switches to local MPPT-like pitch control for safety.

### Station Controller (`station/station_controller.py`)

- Connects to turbine via TCP with HELLO/HELLO_ACK handshake
- Sends pitch and yaw commands with ACK tracking
- Retransmits on timeout with **exponential backoff** (up to 4 retries)
- Listens to UDP telemetry broadcast
- Polls specific sensors on demand
- Broadcasts DISCOVER to find other group turbines

---

## Protocol Messages

| Type | Hex | Direction | Description |
|------|-----|-----------|-------------|
| HELLO | 0x01 | Both | Connection initiation |
| HELLO_ACK | 0x02 | Both | Accept/reject connection |
| TELEMETRY | 0x10 | Turbine→Station | Full sensor dump |
| TELEMETRY_ACK | 0x11 | Station→Turbine | Acknowledge telemetry |
| CMD_PITCH | 0x20 | Station→Turbine | Set blade pitch angle |
| CMD_YAW | 0x21 | Station→Turbine | Set nacelle yaw angle |
| CMD_ACK | 0x22 | Turbine→Station | Command accepted |
| CMD_NACK | 0x23 | Turbine→Station | Command rejected (safety) |
| SENSOR_REQ | 0x30 | Station→Turbine | Poll one sensor |
| SENSOR_RESP | 0x31 | Turbine→Station | Sensor value response |
| PING | 0x40 | Both | RTT measurement |
| PONG | 0x41 | Both | RTT response |
| RETRANSMIT | 0x42 | Both | Request missing frame |
| DISCOVER | 0x50 | Station→Turbine | Find nearby turbines |
| DISCOVER_RESP | 0x51 | Turbine→Station | Node advertisement |

---

## Interoperating with Other Groups

To connect to another group's turbine:
1. Confirm they are also running and advertising on port 5004 (DISCOVER)
2. Send a DISCOVER broadcast — their turbine will respond with its services
3. Connect to their command port (TCP) with a HELLO handshake
4. Exchange CMD_PITCH / CMD_YAW commands and receive telemetry

---

## Marking Criteria Coverage

| Requirement | File | How it's met |
|-------------|------|-------------|
| ≥4 separate socket processes | turbine_node.py | Ports 5001–5004 |
| ≥2 raw socket implementations | turbine_node.py | Ports 5001 (UDP) & 5002 (TCP) |
| Channel modelling | channel_model.py | Delay, loss, contact windows |
| Protocol design | protocol.py | Custom binary framing, 15 msg types |
| Yaw & pitch control | turbine_node.py / station_controller.py | CMD_PITCH, CMD_YAW |
| Sensor gathering | turbine_node.py | 9 sensors, polled + broadcast |
| Reliability | station_controller.py | ACK/NACK, retransmit, backoff |
| Autonomous fallback | turbine_node.py | AutonomousController class |
| Discovery | both | DISCOVER/DISCOVER_RESP on port 5004 |

---

## Demo Script (for live interview)

1. Start turbine: `python turbine/turbine_node.py`
2. Start dashboard: `python dashboard.py`
3. Show: channel contact window open → telemetry arriving
4. Show: pitch/yaw commands being sent and ACK'd
5. Show: sensor poll (on-demand query)
6. Show: channel gap (no satellite contact) → turbine switches to autonomous mode
7. Show: channel recovers → remote control resumes
8. (Optional) Discover another group's turbine and send commands to it
