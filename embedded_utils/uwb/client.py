"""
UciClient - one method per UCI command, for one DWM3001CDK.

    with UciClient("/dev/ttyACM0") as uwb:
        print(uwb.get_device_info())
        handle = uwb.session_init(42)
        uwb.set_app_config(handle, {AppConfigId.CHANNEL_NUMBER: 9, ...})

Every command blocks until its response arrives and raises UciStatusError on a
non-OK status or UciTimeout if the board doesn't answer. UCI allows only one
outstanding command at a time, so commands from several threads are
serialised.

Notifications are parsed in a background reader thread. The client keeps the
latest device state and per-session state from them (see wait_device_state /
wait_session_state), and hands every notification to listeners registered
with add_listener() or queues made by subscribe(). Listeners run on the reader
thread, so keep them quick.

Sessions are addressed by the *session handle* session_init() returns. On UCI
2.0 firmware (what the QM33 SDK ships) that differs from the session id; on
1.x firmware it's the session id itself.
"""

from __future__ import annotations

import glob
import logging
import queue
import struct
import threading
import time
from typing import Callable, Iterable, Optional

import serial

from .constants import (
    AppConfigId, CoreOid, DeviceConfigId, DeviceState, Gid, MessageType,
    MulticastAction, ResetConfig, SessionConfigOid, SessionControlOid,
    SessionState, SessionType, Status, name_of,
)
from .messages import (
    APP_CONFIG_FORMATS, DEVICE_CONFIG_FORMATS, ConfigValue, DeviceInfo,
    DeviceStatusNtf, GenericErrorNtf, Notification, PacketReader,
    SessionStatusNtf, UciError, UciMessage, UciStatusError, UciTimeout,
    decode_config, encode_config, encode_tlvs, parse_config_status_list,
    parse_notification, parse_tlvs, u8, u16, u32,
)

log = logging.getLogger(__name__)

Listener = Callable[[Notification], None]


def list_ports() -> dict:
    """
    Candidate serial ports. Prefer the /dev/serial/by-id paths: they stay put
    across reboots, unlike ttyACM numbering. The UCI port is the board's UPPER
    USB (J20); a by-id name containing SEGGER_J-Link is the debug port.
    """
    import serial.tools.list_ports
    return {
        "by_id": sorted(glob.glob("/dev/serial/by-id/*")),
        "comports": [(p.device, p.description, p.hwid) for p in serial.tools.list_ports.comports()],
    }


class UciClient:
    def __init__(self, port, command_timeout: float = 2.0):
        """
        `port` is a device path, or an already-open serial.Serial-like object
        (anything with read/write/close - handy for tests).
        """
        if isinstance(port, str):
            # USB CDC ignores the baud rate; pyserial just wants one.
            self._ser = serial.Serial(port, baudrate=115200, timeout=0.05)
        else:
            self._ser = port
        self.command_timeout = command_timeout

        self._cmd_lock = threading.Lock()
        self._pending: Optional[tuple[int, int]] = None
        self._rsp: "queue.Queue[UciMessage | UciError]" = queue.Queue()

        self._state_cv = threading.Condition()
        self._device_state: Optional[int] = None
        self._session_states: dict[int, SessionStatusNtf] = {}
        self._fatal: Optional[Exception] = None

        self._listeners: list[Listener] = []
        self._listeners_lock = threading.Lock()

        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, name="uci-reader", daemon=True)
        self._reader.start()

    # ---- lifecycle -----------------------------------------------------------

    def close(self):
        self._stop.set()
        self._reader.join(timeout=1.0)
        self._ser.close()

    def __enter__(self) -> "UciClient":
        return self

    def __exit__(self, *exc):
        self.close()

    # ---- notifications -------------------------------------------------------

    def add_listener(self, listener: Listener):
        with self._listeners_lock:
            self._listeners.append(listener)

    def remove_listener(self, listener: Listener):
        with self._listeners_lock:
            if listener in self._listeners:
                self._listeners.remove(listener)

    def subscribe(self, maxsize: int = 256) -> "queue.Queue[Notification]":
        """
        A queue that receives every notification from now on. When it fills
        up the oldest entries are dropped. Stop it with unsubscribe().
        """
        q: "queue.Queue[Notification]" = queue.Queue(maxsize)

        def put(ntf: Notification):
            while True:
                try:
                    q.put_nowait(ntf)
                    return
                except queue.Full:
                    try:
                        q.get_nowait()
                    except queue.Empty:
                        pass

        q._uci_listener = put  # type: ignore[attr-defined]
        self.add_listener(put)
        return q

    def unsubscribe(self, q: "queue.Queue[Notification]"):
        self.remove_listener(q._uci_listener)  # type: ignore[attr-defined]

    @property
    def device_state(self) -> Optional[int]:
        """Last DeviceState reported by DEVICE_STATUS_NTF (None = not heard yet)."""
        return self._device_state

    def session_status(self, handle: int) -> Optional[SessionStatusNtf]:
        """Last SESSION_STATUS_NTF for this session (None = not heard yet)."""
        return self._session_states.get(handle)

    def wait_device_state(self, state: int = DeviceState.READY, timeout: float = 2.0):
        self._wait_for(lambda: self._device_state == state, timeout,
                       f"device state {name_of(DeviceState, state)}")

    def wait_session_state(self, handle: int, state: int, timeout: float = 2.0) -> SessionStatusNtf:
        def reached():
            st = self._session_states.get(handle)
            return st is not None and st.state == state
        self._wait_for(reached, timeout, f"session 0x{handle:08X} state {name_of(SessionState, state)}")
        return self._session_states[handle]

    def _wait_for(self, predicate: Callable[[], bool], timeout: float, what: str):
        deadline = time.monotonic() + timeout
        with self._state_cv:
            while not predicate():
                self._raise_if_dead()
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise UciTimeout(f"timed out waiting for {what}")
                self._state_cv.wait(remaining)

    # ---- Core group ----------------------------------------------------------

    def device_reset(self, timeout: Optional[float] = None):
        """
        Reset the UWBS. The board then reports DEVICE_STATUS_NTF(READY); use
        wait_device_state() to wait for it. Some firmware builds reboot before
        answering, in which case this raises UciTimeout.
        """
        with self._state_cv:
            self._device_state = None
        self._command(Gid.CORE, CoreOid.DEVICE_RESET, u8(ResetConfig.UWBS_RESET),
                      "DEVICE_RESET", timeout)

    def get_device_info(self) -> DeviceInfo:
        return DeviceInfo.parse(self._command(Gid.CORE, CoreOid.GET_DEVICE_INFO, b"", "GET_DEVICE_INFO"))

    def get_caps_info(self) -> dict[int, bytes]:
        """Capability TLVs, keyed by capability id (values left raw)."""
        p = self._command(Gid.CORE, CoreOid.GET_CAPS_INFO, b"", "GET_CAPS_INFO")
        return parse_tlvs(p, p[1], offset=2)

    def set_config(self, params: dict[int, ConfigValue]):
        """Device-wide config (DeviceConfigId keys). Ints are packed to each id's size."""
        tlvs = encode_config(DEVICE_CONFIG_FORMATS, params)
        p = self._command(Gid.CORE, CoreOid.SET_CONFIG, u8(len(tlvs)) + encode_tlvs(tlvs),
                          "SET_CONFIG", check=False)
        _check_config_rsp("SET_CONFIG", p)

    def get_config(self, ids: Iterable[int]) -> dict[int, ConfigValue]:
        ids = list(ids)
        p = self._command(Gid.CORE, CoreOid.GET_CONFIG, u8(len(ids)) + bytes(ids), "GET_CONFIG")
        return decode_config(DEVICE_CONFIG_FORMATS, parse_tlvs(p, p[1], offset=2))

    def query_timestamp(self) -> int:
        """UWBS clock, in microseconds."""
        p = self._command(Gid.CORE, CoreOid.QUERY_UWBS_TIMESTAMP, b"", "QUERY_UWBS_TIMESTAMP")
        return struct.unpack_from("<Q", p, 1)[0]

    # ---- Session Config group ------------------------------------------------

    def session_init(self, session_id: int, session_type: int = SessionType.FIRA_RANGING) -> int:
        """Create a session; returns the handle every other session command takes."""
        p = self._command(Gid.SESSION_CONFIG, SessionConfigOid.INIT,
                          u32(session_id) + u8(session_type), "SESSION_INIT")
        return struct.unpack_from("<I", p, 1)[0] if len(p) >= 5 else session_id

    def session_deinit(self, handle: int):
        self._command(Gid.SESSION_CONFIG, SessionConfigOid.DEINIT, u32(handle), "SESSION_DEINIT")

    def set_app_config(self, handle: int, params: dict[int, ConfigValue]):
        """
        Session config (AppConfigId keys). Ints are packed to each id's size;
        bytes values (MAC addresses, STATIC_STS_IV, vendor ids) go out as-is.
        On rejection the error names each parameter the board refused.
        """
        tlvs = encode_config(APP_CONFIG_FORMATS, params)
        p = self._command(Gid.SESSION_CONFIG, SessionConfigOid.SET_APP_CONFIG,
                          u32(handle) + u8(len(tlvs)) + encode_tlvs(tlvs),
                          "SET_APP_CONFIG", check=False)
        _check_config_rsp("SET_APP_CONFIG", p)

    def get_app_config(self, handle: int, ids: Iterable[int] = ()) -> dict[int, ConfigValue]:
        """Read session config back. No ids = every parameter."""
        ids = list(ids)
        p = self._command(Gid.SESSION_CONFIG, SessionConfigOid.GET_APP_CONFIG,
                          u32(handle) + u8(len(ids)) + bytes(ids), "GET_APP_CONFIG")
        return decode_config(APP_CONFIG_FORMATS, parse_tlvs(p, p[1], offset=2))

    def get_session_count(self) -> int:
        p = self._command(Gid.SESSION_CONFIG, SessionConfigOid.GET_COUNT, b"", "SESSION_GET_COUNT")
        return p[1]

    def get_session_state(self, handle: int) -> int:
        p = self._command(Gid.SESSION_CONFIG, SessionConfigOid.GET_STATE, u32(handle), "SESSION_GET_STATE")
        return p[1]

    def update_controller_multicast_list(self, handle: int, action: int,
                                         controlees: Iterable[tuple[int, int]]):
        """
        Add/remove controlees of a running one-to-many session.
        `controlees` is (short_address, subsession_id) pairs; subsession_id is
        0 unless you're using provisioned STS.
        """
        controlees = list(controlees)
        payload = u32(handle) + u8(action) + u8(len(controlees))
        for addr, subsession_id in controlees:
            payload += u16(addr) + u32(subsession_id)
        self._command(Gid.SESSION_CONFIG, SessionConfigOid.UPDATE_CONTROLLER_MULTICAST_LIST,
                      payload, f"UPDATE_CONTROLLER_MULTICAST_LIST({name_of(MulticastAction, action)})")

    # ---- Session Control group -----------------------------------------------

    def session_start(self, handle: int):
        """Start ranging. Results then arrive as SessionInfoNtf notifications."""
        self._command(Gid.SESSION_CONTROL, SessionControlOid.START, u32(handle), "SESSION_START")

    def session_stop(self, handle: int):
        self._command(Gid.SESSION_CONTROL, SessionControlOid.STOP, u32(handle), "SESSION_STOP")

    def get_ranging_count(self, handle: int) -> int:
        """Number of ranging rounds run so far in this session."""
        p = self._command(Gid.SESSION_CONTROL, SessionControlOid.GET_RANGING_COUNT,
                          u32(handle), "SESSION_GET_RANGING_COUNT")
        return struct.unpack_from("<I", p, 1)[0]

    # ---- raw access ----------------------------------------------------------

    def send_raw(self, gid: int, oid: int, payload: bytes = b"",
                 timeout: Optional[float] = None) -> bytes:
        """Send any command (e.g. Qorvo vendor groups); returns the raw response payload."""
        return self._command(gid, oid, payload, f"CMD gid=0x{gid:X} oid=0x{oid:02X}",
                             timeout, check=False)

    # ---- internals -----------------------------------------------------------

    def _command(self, gid: int, oid: int, payload: bytes, what: str,
                 timeout: Optional[float] = None, check: bool = True) -> bytes:
        timeout = self.command_timeout if timeout is None else timeout
        with self._cmd_lock:
            self._raise_if_dead()
            while not self._rsp.empty():  # drop anything a timed-out command left behind
                self._rsp.get_nowait()
            self._pending = (gid, oid)
            try:
                msg = UciMessage(MessageType.COMMAND, gid, oid, payload)
                log.debug("TX %r", msg)
                self._ser.write(msg.encode())
                try:
                    rsp = self._rsp.get(timeout=timeout)
                except queue.Empty:
                    raise UciTimeout(f"{what}: no response within {timeout}s") from None
            finally:
                self._pending = None

        if isinstance(rsp, UciError):
            raise rsp
        if check and rsp.payload and rsp.payload[0] != Status.OK:
            raise UciStatusError(what, rsp.payload[0])
        return rsp.payload

    def _raise_if_dead(self):
        if self._fatal is not None:
            raise UciError(f"serial link is down: {self._fatal}") from self._fatal

    def _read_loop(self):
        reader = PacketReader()
        while not self._stop.is_set():
            try:
                data = self._ser.read(256)
            except Exception as e:  # unplugged, port closed under us, ...
                log.error("serial read failed: %s", e)
                self._fatal = e
                self._rsp.put(UciError(f"serial link is down: {e}"))
                with self._state_cv:
                    self._state_cv.notify_all()
                return
            if data:
                for msg in reader.feed(data):
                    self._dispatch(msg)

    def _dispatch(self, msg: UciMessage):
        log.debug("RX %r", msg)
        if msg.mt == MessageType.RESPONSE:
            if self._pending == (msg.gid, msg.oid):
                self._rsp.put(msg)
            else:
                log.warning("response nobody is waiting for: %r", msg)
            return
        if msg.mt != MessageType.NOTIFICATION:
            log.debug("ignoring %r", msg)
            return

        ntf = parse_notification(msg)
        with self._state_cv:
            if isinstance(ntf, DeviceStatusNtf):
                self._device_state = ntf.state
            elif isinstance(ntf, SessionStatusNtf):
                self._session_states[ntf.session_handle] = ntf
            self._state_cv.notify_all()

        if isinstance(ntf, GenericErrorNtf) and self._pending is not None:
            # The board couldn't parse/handle the command it was given.
            gid, oid = self._pending
            self._rsp.put(UciStatusError(f"CMD gid=0x{gid:X} oid=0x{oid:02X}", ntf.status,
                                         "reported via GENERIC_ERROR_NTF"))

        with self._listeners_lock:
            listeners = list(self._listeners)
        for listener in listeners:
            try:
                listener(ntf)
            except Exception:
                log.exception("notification listener raised")


def _check_config_rsp(what: str, p: bytes):
    """SET_CONFIG / SET_APP_CONFIG: status, then the ids that were refused and why."""
    if p[0] == Status.OK:
        return
    ids = AppConfigId if what == "SET_APP_CONFIG" else DeviceConfigId
    refused = ", ".join(f"{name_of(ids, tag)}: {name_of(Status, st)}"
                        for tag, st in parse_config_status_list(p, 1).items())
    raise UciStatusError(what, p[0], refused)
