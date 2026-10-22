"""
UCI wire format: packet framing/segmentation, the app-config value codec, and
parsers for the responses and notifications the client hands back.

Control packet header (4 bytes), all multi-byte payload fields little-endian:

    byte 0: MT(3) | PBF(1) | GID(4)
    byte 1: RFU(2) | OID(6)
    byte 2: RFU
    byte 3: payload length (0-255)

A message whose payload is > 255 bytes is split into several packets; every
packet but the last has PBF=1.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Optional, Union

from .constants import (
    AppConfigId, DeviceConfigId, DeviceState, Gid, MessageType,
    RangingMeasurementType, ReasonCode, SessionState, Status,
    name_of,
)

HEADER_LEN = 4
MAX_PAYLOAD = 255


# ==============================================================================
# Packets
# ==============================================================================

@dataclass
class UciMessage:
    """One complete (reassembled) UCI message."""
    mt: int
    gid: int
    oid: int
    payload: bytes = b""

    def to_packets(self) -> list[bytes]:
        """Encode, splitting into PBF-flagged segments if the payload is > 255 bytes."""
        chunks = [self.payload[i:i + MAX_PAYLOAD]
                  for i in range(0, len(self.payload), MAX_PAYLOAD)] or [b""]
        packets = []
        for i, chunk in enumerate(chunks):
            pbf = 1 if i < len(chunks) - 1 else 0
            byte0 = ((self.mt & 0x07) << 5) | (pbf << 4) | (self.gid & 0x0F)
            packets.append(bytes([byte0, self.oid & 0x3F, 0x00, len(chunk)]) + chunk)
        return packets

    def encode(self) -> bytes:
        return b"".join(self.to_packets())

    def __repr__(self) -> str:
        mt = name_of(MessageType, self.mt)
        return f"UciMessage({mt} gid=0x{self.gid:X} oid=0x{self.oid:02X} payload={self.payload.hex()})"


class PacketReader:
    """
    Turns a raw byte stream into complete UciMessages: frames packets off the
    stream and reassembles PBF segments. Feed it whatever the serial port
    returns; it buffers partial packets between calls.
    """

    def __init__(self):
        self._buf = bytearray()
        self._partial: dict[tuple[int, int, int], bytearray] = {}

    def feed(self, data: bytes) -> list[UciMessage]:
        self._buf.extend(data)
        out = []
        while len(self._buf) >= HEADER_LEN:
            byte0 = self._buf[0]
            mt = (byte0 >> 5) & 0x07
            if mt == MessageType.DATA:
                # Data packets carry a 16-bit length in bytes 2-3.
                length = self._buf[2] | (self._buf[3] << 8)
            else:
                length = self._buf[3]
            if len(self._buf) < HEADER_LEN + length:
                break
            packet = bytes(self._buf[:HEADER_LEN + length])
            del self._buf[:HEADER_LEN + length]

            pbf = (byte0 >> 4) & 0x01
            gid = byte0 & 0x0F
            oid = packet[1] & 0x3F if mt != MessageType.DATA else 0
            key = (mt, gid, oid)
            self._partial.setdefault(key, bytearray()).extend(packet[HEADER_LEN:])
            if not pbf:
                out.append(UciMessage(mt, gid, oid, bytes(self._partial.pop(key))))
        return out


# ==============================================================================
# Errors
# ==============================================================================

class UciError(Exception):
    """Base class for everything this package raises."""


class UciTimeout(UciError):
    pass


class UciStatusError(UciError):
    """A response (or GENERIC_ERROR_NTF) came back with a non-OK status."""

    def __init__(self, what: str, status: int, detail: str = ""):
        self.status = status
        msg = f"{what} failed: {name_of(Status, status)}"
        super().__init__(f"{msg} ({detail})" if detail else msg)


# ==============================================================================
# Field helpers
# ==============================================================================

def u8(v: int) -> bytes:
    return struct.pack("<B", v)


def u16(v: int) -> bytes:
    return struct.pack("<H", v)


def u32(v: int) -> bytes:
    return struct.pack("<I", v)


def parse_tlvs(data: bytes, count: int, offset: int = 0) -> dict[int, bytes]:
    """`count` TLVs of the form id u8, len u8, value."""
    tlvs = {}
    for _ in range(count):
        tag, length = data[offset], data[offset + 1]
        tlvs[tag] = bytes(data[offset + 2:offset + 2 + length])
        offset += 2 + length
    return tlvs


def encode_tlvs(params: dict[int, bytes]) -> bytes:
    out = b""
    for tag, value in params.items():
        if len(value) > 255:
            raise ValueError(f"config 0x{tag:02X}: value longer than 255 bytes")
        out += bytes([tag, len(value)]) + value
    return out


def parse_config_status_list(data: bytes, offset: int) -> dict[int, int]:
    """SET_CONFIG / SET_APP_CONFIG response tail: n u8, then (id u8, status u8) * n."""
    n = data[offset] if len(data) > offset else 0
    return {data[offset + 1 + 2 * i]: data[offset + 2 + 2 * i] for i in range(n)}


# ==============================================================================
# App/device config value codec
# ==============================================================================

# struct format of each fixed-size integer parameter. Parameters missing here
# (MAC addresses, STATIC_STS_IV, vendor ones...) are passed through as bytes.
APP_CONFIG_FORMATS: dict[int, str] = {
    AppConfigId.DEVICE_TYPE: "<B",
    AppConfigId.RANGING_ROUND_USAGE: "<B",
    AppConfigId.STS_CONFIG: "<B",
    AppConfigId.MULTI_NODE_MODE: "<B",
    AppConfigId.CHANNEL_NUMBER: "<B",
    AppConfigId.NUMBER_OF_CONTROLEES: "<B",
    AppConfigId.SLOT_DURATION: "<H",          # RSTU
    AppConfigId.RANGING_DURATION: "<I",       # ms
    AppConfigId.STS_INDEX: "<I",
    AppConfigId.MAC_FCS_TYPE: "<B",
    AppConfigId.RANGING_ROUND_CONTROL: "<B",
    AppConfigId.AOA_RESULT_REQ: "<B",
    AppConfigId.SESSION_INFO_NTF_CONFIG: "<B",
    AppConfigId.NEAR_PROXIMITY_CONFIG: "<H",  # cm
    AppConfigId.FAR_PROXIMITY_CONFIG: "<H",   # cm
    AppConfigId.DEVICE_ROLE: "<B",
    AppConfigId.RFRAME_CONFIG: "<B",
    AppConfigId.RSSI_REPORTING: "<B",
    AppConfigId.PREAMBLE_CODE_INDEX: "<B",
    AppConfigId.SFD_ID: "<B",
    AppConfigId.PSDU_DATA_RATE: "<B",
    AppConfigId.PREAMBLE_DURATION: "<B",
    AppConfigId.SLOTS_PER_RR: "<B",
    AppConfigId.PRF_MODE: "<B",
    AppConfigId.SCHEDULE_MODE: "<B",
    AppConfigId.KEY_ROTATION: "<B",
    AppConfigId.KEY_ROTATION_RATE: "<B",
    AppConfigId.SESSION_PRIORITY: "<B",
    AppConfigId.MAC_ADDRESS_MODE: "<B",
    AppConfigId.VENDOR_ID: "<H",
    AppConfigId.NUMBER_OF_STS_SEGMENTS: "<B",
    AppConfigId.MAX_RR_RETRY: "<H",
    AppConfigId.HOPPING_MODE: "<B",
    AppConfigId.BLOCK_STRIDE_LENGTH: "<B",
    AppConfigId.RESULT_REPORT_CONFIG: "<B",
    AppConfigId.IN_BAND_TERMINATION_ATTEMPT_COUNT: "<B",
    AppConfigId.SUB_SESSION_ID: "<I",
    AppConfigId.MAX_NUMBER_OF_MEASUREMENTS: "<H",
}

DEVICE_CONFIG_FORMATS: dict[int, str] = {
    DeviceConfigId.DEVICE_STATE: "<B",
    DeviceConfigId.LOW_POWER_MODE: "<B",
}

ConfigValue = Union[int, bytes]


def encode_config(formats: dict[int, str], params: dict[int, ConfigValue]) -> dict[int, bytes]:
    encoded = {}
    for tag, value in params.items():
        if isinstance(value, (bytes, bytearray)):
            encoded[tag] = bytes(value)
        elif tag in formats:
            encoded[tag] = struct.pack(formats[tag], value)
        else:
            raise TypeError(f"config 0x{tag:02X} has no known integer format; pass it as bytes")
    return encoded


def decode_config(formats: dict[int, str], tlvs: dict[int, bytes]) -> dict[int, ConfigValue]:
    out: dict[int, ConfigValue] = {}
    for tag, value in tlvs.items():
        fmt = formats.get(tag)
        out[tag] = struct.unpack(fmt, value)[0] if fmt and len(value) == struct.calcsize(fmt) else value
    return out


def short_addresses(*addrs: int) -> bytes:
    """Encode one or more 2-byte MAC addresses (DEVICE_MAC_ADDRESS / DST_MAC_ADDRESS)."""
    return b"".join(u16(a) for a in addrs)


# ==============================================================================
# Responses
# ==============================================================================

def _version(data: bytes, off: int) -> Optional[tuple[int, int, int]]:
    """2-byte version field: major, then minor << 4 | maintenance."""
    if len(data) < off + 2:
        return None
    return data[off], data[off + 1] >> 4, data[off + 1] & 0x0F


@dataclass
class DeviceInfo:
    uci_version: Optional[tuple[int, int, int]]
    mac_version: Optional[tuple[int, int, int]]
    phy_version: Optional[tuple[int, int, int]]
    uci_test_version: Optional[tuple[int, int, int]]
    vendor_info: bytes

    @classmethod
    def parse(cls, p: bytes) -> "DeviceInfo":
        vendor_len = p[9] if len(p) >= 10 else 0
        return cls(_version(p, 1), _version(p, 3), _version(p, 5), _version(p, 7),
                   bytes(p[10:10 + vendor_len]))


# ==============================================================================
# Notifications
# ==============================================================================

@dataclass
class DeviceStatusNtf:
    state: int

    @classmethod
    def parse(cls, p: bytes) -> "DeviceStatusNtf":
        return cls(state=p[0])

    def __str__(self) -> str:
        return f"DeviceStatus({name_of(DeviceState, self.state)})"


@dataclass
class GenericErrorNtf:
    status: int

    @classmethod
    def parse(cls, p: bytes) -> "GenericErrorNtf":
        return cls(status=p[0])

    def __str__(self) -> str:
        return f"GenericError({name_of(Status, self.status)})"


@dataclass
class SessionStatusNtf:
    session_handle: int
    state: int
    reason: int

    @classmethod
    def parse(cls, p: bytes) -> "SessionStatusNtf":
        return cls(*struct.unpack_from("<IBB", p, 0))

    def __str__(self) -> str:
        return (f"SessionStatus(handle=0x{self.session_handle:08X} "
                f"{name_of(SessionState, self.state)} reason={name_of(ReasonCode, self.reason)})")


@dataclass
class TwoWayMeasurement:
    mac_address: int
    status: int
    nlos: int
    distance_cm: int
    aoa_azimuth_deg: float
    aoa_azimuth_fom: int
    aoa_elevation_deg: float
    aoa_elevation_fom: int
    slot_index: int
    rssi: int

    @property
    def ok(self) -> bool:
        return self.status == Status.OK

    @property
    def distance_m(self) -> float:
        return self.distance_cm / 100.0


def _q9_7(raw: int) -> float:
    """AoA fields are signed Q9.7 fixed-point degrees."""
    return raw / 128.0


# Header fields before the measurement list, and the size of one two-way
# measurement record (short and extended address forms are both 31 bytes:
# the extended form's 8-byte MAC eats into the trailing reserved bytes).
_SESSION_INFO_HEADER = struct.Struct("<IIBIBBB8xB")
_MEASUREMENT_LEN = 31


@dataclass
class SessionInfoNtf:
    """
    SESSION_INFO_NTF - one ranging round's results. `measurements` is only
    populated for two-way ranging; `raw` always holds the full payload.

    Qorvo's firmware may append vendor-specific data after the measurements;
    it's ignored here but still available in `raw`.
    """
    sequence_number: int
    session_handle: int
    current_ranging_interval_ms: int
    measurement_type: int
    extended_addresses: bool
    measurements: list[TwoWayMeasurement] = field(default_factory=list)
    raw: bytes = b""

    @classmethod
    def parse(cls, p: bytes) -> "SessionInfoNtf":
        (seq, handle, _rcr, interval, mtype, _rfu, mac_indicator,
         count) = _SESSION_INFO_HEADER.unpack_from(p, 0)
        extended = mac_indicator != 0
        ntf = cls(seq, handle, interval, mtype, extended, raw=bytes(p))
        if mtype != RangingMeasurementType.TWO_WAY:
            return ntf

        mac_fmt = "Q" if extended else "H"
        record = struct.Struct(f"<{mac_fmt}BBHhBhBhBhBBB")
        off = _SESSION_INFO_HEADER.size
        for _ in range(count):
            (mac, status, nlos, dist, az, az_fom, el, el_fom,
             _daz, _daz_fom, _del, _del_fom, slot, rssi) = record.unpack_from(p, off)
            ntf.measurements.append(TwoWayMeasurement(
                mac_address=mac, status=status, nlos=nlos, distance_cm=dist,
                aoa_azimuth_deg=_q9_7(az), aoa_azimuth_fom=az_fom,
                aoa_elevation_deg=_q9_7(el), aoa_elevation_fom=el_fom,
                slot_index=slot, rssi=rssi,
            ))
            off += _MEASUREMENT_LEN
        return ntf


@dataclass
class UnknownNtf:
    gid: int
    oid: int
    payload: bytes


Notification = Union[DeviceStatusNtf, GenericErrorNtf, SessionStatusNtf, SessionInfoNtf, UnknownNtf]

_NTF_PARSERS = {
    (Gid.CORE, 0x01): DeviceStatusNtf.parse,
    (Gid.CORE, 0x07): GenericErrorNtf.parse,
    (Gid.SESSION_CONFIG, 0x02): SessionStatusNtf.parse,
    (Gid.SESSION_CONTROL, 0x00): SessionInfoNtf.parse,
}


def parse_notification(msg: UciMessage) -> Notification:
    parser = _NTF_PARSERS.get((msg.gid, msg.oid))
    if parser is not None:
        try:
            return parser(msg.payload)
        except (struct.error, IndexError):
            pass  # truncated/vendor-shaped payload: hand it back raw
    return UnknownNtf(msg.gid, msg.oid, msg.payload)
