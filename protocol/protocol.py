"""
CSU33D03 Main Project - Custom Communication Protocol
Group 7-9

Defines all message types, framing, serialisation, and validation
for the Wind Turbine <-> LEO Satellite <-> Space Station protocol.

Protocol Frame Format (binary-friendly, human-readable JSON payload):
  [MAGIC(4)] [VERSION(1)] [MSG_TYPE(1)] [SEQ(4)] [TIMESTAMP(8)] [PAYLOAD_LEN(4)] [PAYLOAD(N)] [CHECKSUM(4)]
"""

import json
import struct
import time
import hashlib
import enum


# ── Constants ──────────────────────────────────────────────────────────────────
MAGIC        = b'WTSP'   # Wind Turbine Space Protocol
VERSION      = 1
HEADER_FMT   = '!4sBBIdd I'   # magic, version, msg_type, seq, timestamp(double), payload_len
HEADER_SIZE  = struct.calcsize(HEADER_FMT)   # 4+1+1+4+8+4 = 22 bytes... let's confirm below
CHECKSUM_LEN = 4


# ── Message Types ──────────────────────────────────────────────────────────────
class MsgType(enum.IntEnum):
    # Handshake
    HELLO        = 0x01
    HELLO_ACK    = 0x02
    GOODBYE      = 0x03

    # Telemetry (turbine → station)
    TELEMETRY    = 0x10
    TELEMETRY_ACK= 0x11

    # Control (station → turbine)
    CMD_PITCH    = 0x20   # set blade pitch angle
    CMD_YAW      = 0x21   # set nacelle yaw angle
    CMD_ACK      = 0x22   # command acknowledged by turbine
    CMD_NACK     = 0x23   # command rejected (e.g. out of safe range)

    # Sensor polling
    SENSOR_REQ   = 0x30
    SENSOR_RESP  = 0x31

    # Channel / reliability
    PING         = 0x40
    PONG         = 0x41
    RETRANSMIT   = 0x42   # request retransmit of seq N

    # Discovery (basic)
    DISCOVER     = 0x50
    DISCOVER_RESP= 0x51


# ── Sequence number counter ────────────────────────────────────────────────────
class SequenceCounter:
    def __init__(self):
        self._seq = 0

    def next(self):
        self._seq = (self._seq + 1) & 0xFFFFFFFF
        return self._seq

    def current(self):
        return self._seq


_global_seq = SequenceCounter()


# ── Checksum ───────────────────────────────────────────────────────────────────
def compute_checksum(data: bytes) -> bytes:
    """Return first 4 bytes of SHA-256 digest — lightweight integrity check."""
    return hashlib.sha256(data).digest()[:CHECKSUM_LEN]


# ── Frame builder ──────────────────────────────────────────────────────────────
def build_frame(msg_type: MsgType, payload: dict, seq: int = None) -> bytes:
    """
    Serialise a message dict into a protocol frame.

    Args:
        msg_type:  MsgType enum value
        payload:   dict — will be JSON-encoded
        seq:       optional sequence number (auto-incremented if None)

    Returns:
        bytes — complete protocol frame
    """
    if seq is None:
        seq = _global_seq.next()

    payload_bytes = json.dumps(payload, separators=(',', ':')).encode('utf-8')
    timestamp     = time.time()

    header = struct.pack(
        '!4sBBIdI',
        MAGIC,
        VERSION,
        int(msg_type),
        seq,
        timestamp,
        len(payload_bytes)
    )

    body     = header + payload_bytes
    checksum = compute_checksum(body)
    return body + checksum


# ── Frame parser ───────────────────────────────────────────────────────────────
def parse_frame(data: bytes) -> dict | None:
    """
    Parse a raw bytes frame into a message dict.

    Returns None if the frame is corrupt or invalid.
    """
    header_fmt = '!4sBBIdI'
    hdr_size   = struct.calcsize(header_fmt)   # 4+1+1+4+8+4 = 22

    if len(data) < hdr_size + CHECKSUM_LEN:
        return None

    # Validate magic
    if data[:4] != MAGIC:
        return None

    magic, version, msg_type_raw, seq, timestamp, payload_len = struct.unpack(
        header_fmt, data[:hdr_size]
    )

    total_expected = hdr_size + payload_len + CHECKSUM_LEN
    if len(data) < total_expected:
        return None   # incomplete frame

    payload_bytes = data[hdr_size: hdr_size + payload_len]
    received_csum = data[hdr_size + payload_len: hdr_size + payload_len + CHECKSUM_LEN]
    expected_csum = compute_checksum(data[:hdr_size + payload_len])

    if received_csum != expected_csum:
        return None   # checksum mismatch — drop frame

    try:
        payload = json.loads(payload_bytes.decode('utf-8'))
    except (json.JSONDecodeError, UnicodeDecodeError):
        return None

    try:
        msg_type = MsgType(msg_type_raw)
    except ValueError:
        msg_type = msg_type_raw   # unknown type — pass raw int

    return {
        'magic'    : magic,
        'version'  : version,
        'msg_type' : msg_type,
        'seq'      : seq,
        'timestamp': timestamp,
        'payload'  : payload,
    }


# ── Convenience payload builders ───────────────────────────────────────────────
def make_hello(node_id: str, node_type: str, group: int) -> bytes:
    return build_frame(MsgType.HELLO, {
        'node_id'  : node_id,
        'node_type': node_type,   # 'turbine' | 'station' | 'satellite'
        'group'    : group,
        'proto_ver': VERSION,
    })


def make_hello_ack(node_id: str, accepted: bool) -> bytes:
    return build_frame(MsgType.HELLO_ACK, {
        'node_id' : node_id,
        'accepted': accepted,
    })


def make_telemetry(node_id: str, sensors: dict) -> bytes:
    return build_frame(MsgType.TELEMETRY, {
        'node_id': node_id,
        'sensors': sensors,
    })


def make_telemetry_ack(seq: int) -> bytes:
    return build_frame(MsgType.TELEMETRY_ACK, {'acked_seq': seq})


def make_cmd_pitch(angle_deg: float, node_id: str) -> bytes:
    return build_frame(MsgType.CMD_PITCH, {
        'target_node': node_id,
        'pitch_deg'  : angle_deg,
    })


def make_cmd_yaw(angle_deg: float, node_id: str) -> bytes:
    return build_frame(MsgType.CMD_YAW, {
        'target_node': node_id,
        'yaw_deg'    : angle_deg,
    })


def make_cmd_ack(seq: int, node_id: str) -> bytes:
    return build_frame(MsgType.CMD_ACK, {
        'acked_seq': seq,
        'node_id'  : node_id,
    })


def make_cmd_nack(seq: int, reason: str) -> bytes:
    return build_frame(MsgType.CMD_NACK, {
        'acked_seq': seq,
        'reason'   : reason,
    })


def make_sensor_req(sensor_name: str, node_id: str) -> bytes:
    return build_frame(MsgType.SENSOR_REQ, {
        'sensor'    : sensor_name,
        'requester' : node_id,
    })


def make_sensor_resp(sensor_name: str, value: float, unit: str) -> bytes:
    return build_frame(MsgType.SENSOR_RESP, {
        'sensor': sensor_name,
        'value' : value,
        'unit'  : unit,
    })


def make_ping(node_id: str) -> bytes:
    return build_frame(MsgType.PING, {'from': node_id, 'sent_at': time.time()})


def make_pong(node_id: str, ping_sent_at: float) -> bytes:
    return build_frame(MsgType.PONG, {
        'from'       : node_id,
        'ping_sent_at': ping_sent_at,
        'rtt_hint'   : time.time() - ping_sent_at,
    })


def make_retransmit_req(missing_seq: int) -> bytes:
    return build_frame(MsgType.RETRANSMIT, {'missing_seq': missing_seq})


def make_discover(node_id: str, group: int) -> bytes:
    return build_frame(MsgType.DISCOVER, {
        'node_id': node_id,
        'group'  : group,
    })


def make_discover_resp(node_id: str, group: int, services: list) -> bytes:
    return build_frame(MsgType.DISCOVER_RESP, {
        'node_id' : node_id,
        'group'   : group,
        'services': services,   # list of port/capability dicts
    })
