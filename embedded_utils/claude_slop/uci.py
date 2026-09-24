"""
uci.py - Minimal Python driver for the FiRa UCI (UWB Command Interface)
protocol, targeting the Qorvo DWM3001CDK running Qorvo's stock UCI
firmware (DW3_QM33_SDK "..._UCI-FreeRTOS.hex").

Transport
---------
The DWM3001CDK exposes UCI over the nRF52833's own USB-CDC serial port
(the UPPER micro-USB connector on the board, sometimes silkscreened J20 -
NOT the lower J-Link/debug port). On a Raspberry Pi it shows up as
/dev/ttyACM0 (or similar). See find_ports() below to help identify it
when you have several boards plugged into one Pi.

UCI packet format (FiRa UCI Generic Technical Specification), 4-byte
header, little-endian, as implemented by Qorvo/Android/AOSP `pica`:

    byte 0: MT(3) | PBF(1) | GID(4)
    byte 1: RFU(2) | OID(6)
    byte 2: RFU (extended-length high byte on 2.0 stacks; 0 for us)
    byte 3: Payload Length (0-255 for a non-segmented control packet)
    byte 4..: Payload

MT (Message Type):
    0 = Data, 1 = Command, 2 = Response, 3 = Notification

This module gives you:
  - UciFrame: pack/unpack of the 4-byte header + payload
  - UciTransport: serial port framer/reader running a background thread,
    dispatching RSP packets to whoever sent the matching CMD, and NTF
    packets to a queue you can read from.
  - UciDevice: convenience wrapper with the handful of calls you need to
    bring a session up and start ranging (device reset, get device info,
    session init, session set app config, range start/stop, session
    deinit) plus a RangeDataNtf parser.

This is deliberately small and dependency-light (only pyserial) so it
runs fine on a Raspberry Pi Zero 2 W. It is NOT a full UCI stack - it
covers the Core, Session Config and Ranging groups needed to run a basic
FiRa two-device ranging session (one Controller/Initiator +
one Controlee/Responder), which is the standard "two DWM3001CDKs
ranging" setup.

Sources for the wire format:
  - FiRa UCI packet layout: LINKTYPE_FIRA_UCI (tcpdump.org) and
    AOSP uci_defs.h (byte layout: MT/PBF/GID, OID, RFU, Length).
  - GID/OID/App-config-tag enumerations and RangeDataNtf/
    ShortAddressTwoWayRangingMeasurement layout: AOSP `pica`
    uci_packets.pdl (Google's reference UCI implementation), which
    implements the FiRa UCI spec byte-for-byte.
"""

from __future__ import annotations

import struct
import threading
import queue
import time
import glob
import logging
from dataclasses import dataclass, field
from typing import Optional, Callable

try:
    import serial
except ImportError as e:  # pragma: no cover
    raise ImportError(
        "pyserial is required: pip install pyserial (use --break-system-packages on the Pi if needed)"
    ) from e

log = logging.getLogger("uci")

# --------------------------------------------------------------------------
# Message Type (byte 0, bits 7-5)
# --------------------------------------------------------------------------
MT_DATA = 0
MT_CMD = 1
MT_RSP = 2
MT_NTF = 3

# --------------------------------------------------------------------------
# Group Identifiers (byte 0, bits 3-0)
# --------------------------------------------------------------------------
GID_CORE = 0x00
GID_SESSION_CONFIG = 0x01
GID_RANGING_SESSION_CONTROL = 0x02
GID_DATA_CONTROL = 0x03
GID_TEST = 0x0D
# 0x09-0x0C, 0x0E-0x0F are vendor-specific groups (Qorvo uses some of
# these for calibration / vendor extensions - not covered here).

# --------------------------------------------------------------------------
# Core group opcodes (GID_CORE)
# --------------------------------------------------------------------------
CORE_DEVICE_RESET = 0x00
CORE_DEVICE_STATUS_NTF = 0x01
CORE_GET_DEVICE_INFO = 0x02
CORE_GET_CAPS_INFO = 0x03
CORE_SET_CONFIG = 0x04
CORE_GET_CONFIG = 0x05
CORE_DEVICE_SUSPEND = 0x06
CORE_GENERIC_ERROR_NTF = 0x07

# --------------------------------------------------------------------------
# Session Config group opcodes (GID_SESSION_CONFIG)
# --------------------------------------------------------------------------
SESSION_INIT = 0x00
SESSION_DEINIT = 0x01
SESSION_STATUS_NTF = 0x02
SESSION_SET_APP_CONFIG = 0x03
SESSION_GET_APP_CONFIG = 0x04
SESSION_GET_COUNT = 0x05
SESSION_GET_STATE = 0x06
SESSION_UPDATE_CONTROLLER_MULTICAST_LIST = 0x07

# --------------------------------------------------------------------------
# Ranging Session Control group opcodes (GID_RANGING_SESSION_CONTROL)
# --------------------------------------------------------------------------
RANGE_START = 0x00          # also: RangeDataNtf arrives as MT_NTF w/ this OID
RANGE_STOP = 0x01
RANGE_INTERVAL_UPDATE_REQ = 0x02
RANGE_GET_RANGING_COUNT = 0x03

# --------------------------------------------------------------------------
# Status codes (subset - the ones you'll actually see)
# --------------------------------------------------------------------------
STATUS_OK = 0x00
STATUS_REJECTED = 0x01
STATUS_FAILED = 0x02
STATUS_SYNTAX_ERROR = 0x03
STATUS_INVALID_PARAM = 0x04
STATUS_INVALID_RANGE = 0x05

STATUS_NAMES = {
    0x00: "OK", 0x01: "REJECTED", 0x02: "FAILED", 0x03: "SYNTAX_ERROR",
    0x04: "INVALID_PARAM", 0x05: "INVALID_RANGE",
}


def status_name(code: int) -> str:
    return STATUS_NAMES.get(code, f"0x{code:02X}")


# --------------------------------------------------------------------------
# APP_CONFIG parameter tags (SESSION_SET_APP_CONFIG / SESSION_GET_APP_CONFIG)
# Table sourced from AOSP pica's uci_packets.pdl (FiRa UCI Table 45).
# --------------------------------------------------------------------------
CFG_DEVICE_TYPE = 0x00
CFG_RANGING_ROUND_USAGE = 0x01
CFG_STS_CONFIG = 0x02
CFG_MULTI_NODE_MODE = 0x03
CFG_CHANNEL_NUMBER = 0x04
CFG_NUMBER_OF_CONTROLEES = 0x05
CFG_DEVICE_MAC_ADDRESS = 0x06
CFG_DST_MAC_ADDRESS = 0x07
CFG_SLOT_DURATION = 0x08
CFG_RANGING_INTERVAL = 0x09          # aka RANGING_DURATION, in ms
CFG_STS_INDEX = 0x0A
CFG_MAC_FCS_TYPE = 0x0B
CFG_RANGING_ROUND_CONTROL = 0x0C
CFG_AOA_RESULT_REQ = 0x0D
CFG_RNG_DATA_NTF = 0x0E
CFG_RNG_DATA_NTF_PROXIMITY_NEAR = 0x0F
CFG_RNG_DATA_NTF_PROXIMITY_FAR = 0x10
CFG_DEVICE_ROLE = 0x11
CFG_RFRAME_CONFIG = 0x12
CFG_PREAMBLE_CODE_INDEX = 0x14
CFG_SFD_ID = 0x15
CFG_PSDU_DATA_RATE = 0x16
CFG_PREAMBLE_DURATION = 0x17
CFG_RANGING_TIME_STRUCT = 0x1A
CFG_SLOTS_PER_RR = 0x1B
CFG_TX_ADAPTIVE_PAYLOAD_POWER = 0x1C
CFG_RESPONDER_SLOT_INDEX = 0x1E
CFG_PRF_MODE = 0x1F
CFG_SCHEDULED_MODE = 0x22
CFG_KEY_ROTATION = 0x23
CFG_KEY_ROTATION_RATE = 0x24
CFG_SESSION_PRIORITY = 0x25
CFG_MAC_ADDRESS_MODE = 0x26
CFG_VENDOR_ID = 0x27
CFG_STATIC_STS_IV = 0x28
CFG_NUMBER_OF_STS_SEGMENTS = 0x29
CFG_MAX_RR_RETRY = 0x2A
CFG_UWB_INITIATION_TIME = 0x2B
CFG_HOPPING_MODE = 0x2C
CFG_BLOCK_STRIDE_LENGTH = 0x2D
CFG_RESULT_REPORT_CONFIG = 0x2E
CFG_IN_BAND_TERMINATION_ATTEMPT_COUNT = 0x2F

# Common enum values you'll actually use
DEVICE_TYPE_CONTROLEE = 0x00
DEVICE_TYPE_CONTROLLER = 0x01

DEVICE_ROLE_RESPONDER = 0x00
DEVICE_ROLE_INITIATOR = 0x01

RANGING_ROUND_USAGE_SS_TWR_DEFERRED = 0x01
RANGING_ROUND_USAGE_DS_TWR_DEFERRED = 0x02  # most common default

STS_CONFIG_STATIC = 0x00

MULTI_NODE_MODE_UNICAST = 0x00  # One-to-One

MAC_ADDRESS_MODE_SHORT = 0x00   # 2-byte short addresses

RNG_DATA_NTF_ENABLE = 0x01      # notify on every measurement


# ==========================================================================
# Frame packing / unpacking
# ==========================================================================

@dataclass
class UciFrame:
    mt: int
    gid: int
    oid: int
    payload: bytes = b""
    pbf: int = 0  # packet boundary flag; 0 = complete message

    def pack(self) -> bytes:
        if len(self.payload) > 255:
            raise ValueError(
                "payload > 255 bytes needs UCI packet segmentation "
                "(PBF fragmentation), which this minimal driver doesn't "
                "implement - shouldn't be needed for basic ranging setup"
            )
        byte0 = ((self.mt & 0x07) << 5) | ((self.pbf & 0x01) << 4) | (self.gid & 0x0F)
        byte1 = (self.oid & 0x3F) << 0  # top 2 bits RFU = 0
        byte2 = 0x00  # RFU
        byte3 = len(self.payload) & 0xFF
        return bytes([byte0, byte1, byte2, byte3]) + self.payload

    @classmethod
    def header_len(cls) -> int:
        return 4

    @classmethod
    def parse_header(cls, hdr: bytes) -> "UciFrame":
        assert len(hdr) == 4
        byte0, byte1, byte2, byte3 = hdr
        mt = (byte0 >> 5) & 0x07
        pbf = (byte0 >> 4) & 0x01
        gid = byte0 & 0x0F
        oid = byte1 & 0x3F
        length = byte3  # non-extended payload length
        f = cls(mt=mt, gid=gid, oid=oid, payload=b"", pbf=pbf)
        f._expected_len = length  # type: ignore[attr-defined]
        return f

    def mtname(self) -> str:
        return {MT_DATA: "DATA", MT_CMD: "CMD", MT_RSP: "RSP", MT_NTF: "NTF"}.get(self.mt, str(self.mt))

    def __repr__(self) -> str:
        return (f"UciFrame({self.mtname()} gid=0x{self.gid:02X} oid=0x{self.oid:02X} "
                f"len={len(self.payload)} payload={self.payload.hex()})")


def find_ports(vid_pid_hint: Optional[str] = None):
    """
    List candidate serial ports (helpful when several DWM3001CDKs are
    plugged into one Raspberry Pi). Prefer /dev/serial/by-id symlinks
    since they're stable across reboots/replugs and let you tell boards
    apart, unlike /dev/ttyACM0 / ttyACM1 whose numbering can shuffle.

    Run this interactively first to map each board's by-id path to a
    physical unit (e.g. label the boards, plug in one at a time).
    """
    import serial.tools.list_ports as list_ports
    by_id = sorted(glob.glob("/dev/serial/by-id/*"))
    ports = list(list_ports.comports())
    return {"by_id_symlinks": by_id, "comports": [(p.device, p.description, p.hwid) for p in ports]}


class UciTimeout(Exception):
    pass


class UciTransport:
    """
    Owns the serial port. Runs a background thread that reads bytes,
    frames them into UciFrame objects, and:
      - hands RSP frames to whichever thread is waiting on send_command()
      - pushes NTF frames onto self.notifications (a Queue) for the
        caller to consume (ranging results, session status changes, etc.)

    One UciTransport = one physical DWM3001CDK board.
    """

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 2.0):
        # Qorvo's UCI-over-USB-CDC firmware doesn't actually rate-gate on
        # baudrate (USB CDC ACM ignores it), but pyserial wants a value.
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=0.1)
        self.default_timeout = timeout
        self.notifications: "queue.Queue[UciFrame]" = queue.Queue()
        self._rsp_waiters: dict[tuple[int, int], "queue.Queue[UciFrame]"] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

    def close(self):
        self._stop.set()
        self._reader_thread.join(timeout=1.0)
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _reader_loop(self):
        buf = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(256)
            except serial.SerialException as e:
                log.error("serial read error: %s", e)
                return
            if chunk:
                buf.extend(chunk)
            # try to peel complete frames off the front of buf
            while len(buf) >= 4:
                length = buf[3]
                total = 4 + length
                if len(buf) < total:
                    break
                raw = bytes(buf[:total])
                del buf[:total]
                frame = UciFrame.parse_header(raw[:4])
                frame.payload = raw[4:]
                self._dispatch(frame)

    def _dispatch(self, frame: UciFrame):
        log.debug("RX %r", frame)
        if frame.mt == MT_RSP:
            key = (frame.gid, frame.oid)
            with self._lock:
                q = self._rsp_waiters.get(key)
            if q is not None:
                q.put(frame)
            else:
                log.warning("unmatched RSP (no one waiting): %r", frame)
        elif frame.mt == MT_NTF:
            self.notifications.put(frame)
        else:
            log.warning("unexpected MT on RX: %r", frame)

    def send_command(self, gid: int, oid: int, payload: bytes = b"",
                      timeout: Optional[float] = None) -> UciFrame:
        """
        Send a CMD and block for the matching RSP (same GID+OID).
        Raises UciTimeout if nothing comes back in time.
        """
        timeout = self.default_timeout if timeout is None else timeout
        frame = UciFrame(mt=MT_CMD, gid=gid, oid=oid, payload=payload)
        key = (gid, oid)
        q: "queue.Queue[UciFrame]" = queue.Queue()
        with self._lock:
            self._rsp_waiters[key] = q
        try:
            raw = frame.pack()
            log.debug("TX %r", frame)
            self.ser.write(raw)
            self.ser.flush()
            try:
                return q.get(timeout=timeout)
            except queue.Empty:
                raise UciTimeout(f"no RSP for gid=0x{gid:02X} oid=0x{oid:02X} within {timeout}s")
        finally:
            with self._lock:
                self._rsp_waiters.pop(key, None)

    def wait_notification(self, gid: Optional[int] = None, oid: Optional[int] = None,
                           timeout: Optional[float] = None) -> UciFrame:
        """Block until a notification matching (gid, oid) arrives (None = any)."""
        deadline = None if timeout is None else time.monotonic() + timeout
        while True:
            remaining = None if deadline is None else max(0.0, deadline - time.monotonic())
            if deadline is not None and remaining <= 0:
                raise UciTimeout("no matching notification in time")
            try:
                frame = self.notifications.get(timeout=remaining if remaining else 0.1)
            except queue.Empty:
                if deadline is None:
                    continue
                raise UciTimeout("no matching notification in time")
            if (gid is None or frame.gid == gid) and (oid is None or frame.oid == oid):
                return frame
            # not the one we wanted - drop it (or you could re-queue it if
            # you need multiple consumers)
            log.debug("discarding unrelated NTF while waiting: %r", frame)


# ==========================================================================
# TLV helpers for SESSION_SET_APP_CONFIG
# ==========================================================================

def tlv(tag: int, value: bytes) -> bytes:
    if len(value) > 255:
        raise ValueError("app config TLV value > 255 bytes not supported here")
    return bytes([tag & 0xFF, len(value)]) + value


def u8(v: int) -> bytes:
    return struct.pack("<B", v)


def u16(v: int) -> bytes:
    return struct.pack("<H", v)


def u32(v: int) -> bytes:
    return struct.pack("<I", v)


def mac_short(addr: int) -> bytes:
    """2-byte short MAC address, little-endian, e.g. mac_short(0x0001)."""
    return struct.pack("<H", addr)


# ==========================================================================
# High-level device wrapper
# ==========================================================================

@dataclass
class RangingMeasurement:
    mac_address: int
    status: int
    nlos: int
    distance_cm: int
    aoa_azimuth_deg: float
    aoa_elevation_deg: float
    slot_index: int

    @property
    def ok(self) -> bool:
        return self.status == 0


@dataclass
class RangeDataNtf:
    sequence_number: int
    session_id: int
    current_ranging_interval_ms: int
    ranging_measurement_type: int
    mac_address_short: bool
    measurements: list = field(default_factory=list)


def _aoa_q9_7_to_deg(raw: int) -> float:
    """AoA fields are Q9.7 fixed point radians per the FiRa spec; signed 16-bit."""
    if raw >= 0x8000:
        raw -= 0x10000
    radians = raw / 128.0
    return radians * 180.0 / 3.14159265358979


def parse_range_data_ntf(payload: bytes) -> RangeDataNtf:
    """
    Parse a RANGE_DATA (SESSION_INFO) notification payload
    (GID=RANGING_SESSION_CONTROL, OID=RANGE_START, MT=NTF).

    Layout (little-endian), from AOSP pica's uci_packets.pdl RangeDataNtf
    + ShortAddressTwoWayRangingMeasurement definitions:

        sequence_number          u32
        session_id               u32
        reserved                 u8
        current_ranging_interval u32   (ms)
        ranging_measurement_type u8
        reserved                 u8
        mac_address_indicator    u8    (0 = short/2B, 1 = extended/8B)
        reserved                 8 bytes
        number_of_measurements   u8
        measurements[N]:
          (short address form, 31 bytes each)
            mac_address              u16
            status                   u8
            nlos                     u8
            distance                 u16  (cm)
            aoa_azimuth              i16  (Q9.7 radians)
            aoa_azimuth_fom          u8
            aoa_elevation            i16  (Q9.7 radians)
            aoa_elevation_fom        u8
            aoa_dest_azimuth         i16
            aoa_dest_azimuth_fom     u8
            aoa_dest_elevation       i16
            aoa_dest_elevation_fom   u8
            slot_index               u8
            reserved                 12 bytes

    NOTE: Qorvo's firmware may add vendor-specific trailing TLVs after the
    spec-defined fields, or the field set present can vary with which
    APP_CONFIG options you enabled (e.g. AOA_RESULT_REQ, RNG_DATA_NTF
    proximity mode). If distances look wrong, dump payload.hex() and
    diff it against DW3_QM33_SDK's own UCI documentation/example host
    code for your exact SDK version before trusting the offsets below.
    """
    off = 0
    sequence_number, session_id = struct.unpack_from("<II", payload, off)
    off += 8
    off += 1  # reserved
    (current_ranging_interval,) = struct.unpack_from("<I", payload, off)
    off += 4
    (ranging_measurement_type,) = struct.unpack_from("<B", payload, off)
    off += 1
    off += 1  # reserved
    (mac_addr_indicator,) = struct.unpack_from("<B", payload, off)
    off += 1
    off += 8  # reserved
    (n_measurements,) = struct.unpack_from("<B", payload, off)
    off += 1

    short_addr = (mac_addr_indicator == 0)
    measurements = []
    rec_len = 31 if short_addr else 37  # extended: 8B mac vs 2B, reserved shrinks accordingly
    for _ in range(n_measurements):
        if short_addr:
            (mac, status, nlos, distance, az, az_fom, el, el_fom,
             daz, daz_fom, del_, del_fom, slot_index) = struct.unpack_from(
                "<HBBHhBhBhBhBB", payload, off
            )
        else:
            # extended address (8-byte MAC) variant - same trailing fields,
            # wider address; included for completeness if you switch
            # MAC_ADDRESS_MODE to extended.
            mac, status, nlos, distance, az, az_fom, el, el_fom, \
                daz, daz_fom, del_, del_fom, slot_index = struct.unpack_from(
                    "<QBBHhBhBhBhBB", payload, off
                )
        measurements.append(RangingMeasurement(
            mac_address=mac, status=status, nlos=nlos, distance_cm=distance,
            aoa_azimuth_deg=_aoa_q9_7_to_deg(az),
            aoa_elevation_deg=_aoa_q9_7_to_deg(el),
            slot_index=slot_index,
        ))
        off += rec_len

    return RangeDataNtf(
        sequence_number=sequence_number,
        session_id=session_id,
        current_ranging_interval_ms=current_ranging_interval,
        ranging_measurement_type=ranging_measurement_type,
        mac_address_short=short_addr,
        measurements=measurements,
    )


class UciDevice:
    """High-level convenience calls on top of UciTransport."""

    def __init__(self, transport: UciTransport):
        self.t = transport

    # ---- Core group ----------------------------------------------------

    def device_reset(self) -> int:
        rsp = self.t.send_command(GID_CORE, CORE_DEVICE_RESET, payload=b"\x00")
        (status,) = struct.unpack_from("<B", rsp.payload, 0)
        return status

    def get_device_info(self) -> dict:
        rsp = self.t.send_command(GID_CORE, CORE_GET_DEVICE_INFO)
        status = rsp.payload[0]
        if len(rsp.payload) >= 8:
            uci_gen_major = rsp.payload[1]
            uci_gen_minor = rsp.payload[2]
            mac_major = rsp.payload[3]
            mac_minor = rsp.payload[4]
            phy_major = rsp.payload[5]
            phy_minor = rsp.payload[6]
            vendor_len = rsp.payload[7]
            vendor_info = rsp.payload[8:8 + vendor_len]
        else:
            uci_gen_major = uci_gen_minor = mac_major = mac_minor = phy_major = phy_minor = None
            vendor_info = b""
        return dict(status=status, uci_version=(uci_gen_major, uci_gen_minor),
                    mac_version=(mac_major, mac_minor), phy_version=(phy_major, phy_minor),
                    vendor_info=vendor_info, raw=rsp.payload)

    # ---- Session config group ------------------------------------------

    def session_init(self, session_id: int, session_type: int = 0x00) -> int:
        payload = u32(session_id) + u8(session_type)
        rsp = self.t.send_command(GID_SESSION_CONFIG, SESSION_INIT, payload)
        return rsp.payload[0]

    def session_deinit(self, session_id: int) -> int:
        rsp = self.t.send_command(GID_SESSION_CONFIG, SESSION_DEINIT, u32(session_id))
        return rsp.payload[0]

    def session_set_app_config(self, session_id: int, tlvs: bytes) -> UciFrame:
        n = tlvs.count(b"") if False else None  # (unused placeholder)
        num_params = 0
        i = 0
        while i < len(tlvs):
            i += 2 + tlvs[i + 1]
            num_params += 1
        payload = u32(session_id) + u8(num_params) + tlvs
        rsp = self.t.send_command(GID_SESSION_CONFIG, SESSION_SET_APP_CONFIG, payload)
        return rsp  # rsp.payload[0] = status; rest = per-param cfg_status list on failure

    def session_get_state(self, session_id: int) -> int:
        rsp = self.t.send_command(GID_SESSION_CONFIG, SESSION_GET_STATE, u32(session_id))
        return rsp.payload[1]  # payload[0]=status, payload[1]=state

    # ---- Ranging control group ------------------------------------------

    def range_start(self, session_id: int) -> int:
        rsp = self.t.send_command(GID_RANGING_SESSION_CONTROL, RANGE_START, u32(session_id))
        return rsp.payload[0]

    def range_stop(self, session_id: int) -> int:
        rsp = self.t.send_command(GID_RANGING_SESSION_CONTROL, RANGE_STOP, u32(session_id))
        return rsp.payload[0]

    def wait_range_data(self, timeout: Optional[float] = None) -> RangeDataNtf:
        frame = self.t.wait_notification(GID_RANGING_SESSION_CONTROL, RANGE_START, timeout=timeout)
        return parse_range_data_ntf(frame.payload)

    def wait_session_status(self, timeout: Optional[float] = None):
        """SESSION_STATUS_NTF: payload = session_id(u32) state(u8) reason_code(u8)."""
        frame = self.t.wait_notification(GID_SESSION_CONFIG, SESSION_STATUS_NTF, timeout=timeout)
        session_id, state, reason = struct.unpack_from("<IBB", frame.payload, 0)
        return dict(session_id=session_id, state=state, reason=reason)


def build_fira_app_config_tlvs(
    device_type: int,
    device_role: int,
    device_short_addr: int,
    dst_short_addr: int,
    channel_number: int = 9,
    ranging_interval_ms: int = 200,
    multi_node_mode: int = MULTI_NODE_MODE_UNICAST,
    ranging_round_usage: int = RANGING_ROUND_USAGE_DS_TWR_DEFERRED,
) -> bytes:
    """
    Build the minimum APP_CONFIG TLV set for a basic static-STS FiRa
    ranging session between exactly two devices (1 controller + 1
    controlee). Both sides must agree on: channel_number, STS_CONFIG,
    ranging_interval, multi_node_mode, VENDOR_ID/STATIC_STS_IV (if you
    set custom static STS - omitted here so the firmware default is
    used, which is fine as long as both boards run the same firmware
    build), and mac_address_mode. They must differ on: device_type,
    device_role, device_mac_address <-> dst_mac_address (swapped), and
    (for the controller) number_of_controlees / dst_mac_address list.
    """
    tlvs = b""
    tlvs += tlv(CFG_DEVICE_TYPE, u8(device_type))
    tlvs += tlv(CFG_DEVICE_ROLE, u8(device_role))
    tlvs += tlv(CFG_RANGING_ROUND_USAGE, u8(ranging_round_usage))
    tlvs += tlv(CFG_STS_CONFIG, u8(STS_CONFIG_STATIC))
    tlvs += tlv(CFG_MULTI_NODE_MODE, u8(multi_node_mode))
    tlvs += tlv(CFG_CHANNEL_NUMBER, u8(channel_number))
    tlvs += tlv(CFG_MAC_ADDRESS_MODE, u8(MAC_ADDRESS_MODE_SHORT))
    tlvs += tlv(CFG_NUMBER_OF_CONTROLEES, u8(1))
    tlvs += tlv(CFG_DEVICE_MAC_ADDRESS, mac_short(device_short_addr))
    tlvs += tlv(CFG_DST_MAC_ADDRESS, mac_short(dst_short_addr))
    tlvs += tlv(CFG_RANGING_INTERVAL, u32(ranging_interval_ms))
    tlvs += tlv(CFG_RNG_DATA_NTF, u8(RNG_DATA_NTF_ENABLE))
    return tlvs
