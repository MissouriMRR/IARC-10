"""
RangingSession - a FiRa two-way-ranging session on top of UciClient, for code
that just wants distances.

    with UciClient("/dev/ttyACM0") as uwb, \\
         RangingSession(uwb, session_id=42, controller=True,
                        my_address=0x0001, peers=[0x0002]) as s:
        d = s.twr_range()          # metres, next fresh measurement
        for rnd in s.rounds():     # or every ranging round as it arrives
            ...

Both boards must use the same session_id, channel, ranging interval and round
usage; one is the controller (initiator), the others controlees (responders),
and each lists the other side's address as its peer. A controller with
several peers runs a one-to-many session.
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Iterator, Optional, Sequence

from .client import UciClient
from .constants import (
    AppConfigId, DeviceRole, DeviceType, MacAddressMode, MultiNodeMode,
    RangingRoundUsage, ReasonCode, SessionInfoNtfConfig, SessionState,
    StsConfig, name_of,
)
from .messages import (
    ConfigValue, Notification, SessionInfoNtf, SessionStatusNtf,
    TwoWayMeasurement, UciError, UciTimeout, short_addresses,
)


class RangingSession:
    def __init__(
        self,
        client: UciClient,
        *,
        session_id: int,
        controller: bool,
        my_address: int,
        peers: Sequence[int],
        channel: int = 9,
        ranging_interval_ms: int = 200,
        round_usage: int = RangingRoundUsage.DS_TWR_DEFERRED,
        extra_config: Optional[dict[int, ConfigValue]] = None,
        state_timeout: float = 2.0,
    ):
        """
        extra_config is merged over the defaults below, so it can override
        any of them or add more (e.g. VENDOR_ID and STATIC_STS_IV if the
        firmware rejects static STS without them).
        """
        if not peers:
            raise ValueError("need at least one peer address")
        if not controller and len(peers) != 1:
            raise ValueError("a controlee ranges with exactly one controller")

        self.client = client
        self.session_id = session_id
        self.peers = list(peers)
        self.state_timeout = state_timeout
        self.handle: Optional[int] = None

        self.config: dict[int, ConfigValue] = {
            AppConfigId.DEVICE_TYPE: DeviceType.CONTROLLER if controller else DeviceType.CONTROLEE,
            AppConfigId.DEVICE_ROLE: DeviceRole.INITIATOR if controller else DeviceRole.RESPONDER,
            AppConfigId.RANGING_ROUND_USAGE: round_usage,
            AppConfigId.STS_CONFIG: StsConfig.STATIC,
            AppConfigId.MULTI_NODE_MODE: (MultiNodeMode.ONE_TO_MANY if len(self.peers) > 1
                                          else MultiNodeMode.ONE_TO_ONE),
            AppConfigId.CHANNEL_NUMBER: channel,
            AppConfigId.MAC_ADDRESS_MODE: MacAddressMode.SHORT,
            AppConfigId.NUMBER_OF_CONTROLEES: len(self.peers),
            AppConfigId.DEVICE_MAC_ADDRESS: short_addresses(my_address),
            AppConfigId.DST_MAC_ADDRESS: short_addresses(*self.peers),
            AppConfigId.RANGING_DURATION: ranging_interval_ms,
            AppConfigId.SESSION_INFO_NTF_CONFIG: SessionInfoNtfConfig.ENABLE,
            **(extra_config or {}),
        }

        self._cv = threading.Condition()
        self._last_ok: dict[int, TwoWayMeasurement] = {}
        self._ok_count: dict[int, int] = {}
        self._rounds: "queue.Queue[Optional[SessionInfoNtf]]" = queue.Queue(maxsize=256)
        self._ended: Optional[SessionStatusNtf] = None
        self._active = False

    # ---- lifecycle -----------------------------------------------------------

    def start(self):
        """SESSION_INIT -> SET_APP_CONFIG -> START, waiting for each state change."""
        c = self.client
        c.add_listener(self._on_notification)
        try:
            self.handle = c.session_init(self.session_id)
            c.set_app_config(self.handle, self.config)
            c.wait_session_state(self.handle, SessionState.IDLE, self.state_timeout)
            self._ended = None
            c.session_start(self.handle)
            c.wait_session_state(self.handle, SessionState.ACTIVE, self.state_timeout)
        except Exception:
            self.stop()
            raise

    def stop(self):
        """Stop and tear down the session. Safe to call more than once."""
        c = self.client
        if self.handle is not None:
            st = c.session_status(self.handle)
            try:
                if st is not None and st.state == SessionState.ACTIVE:
                    c.session_stop(self.handle)
                    c.wait_session_state(self.handle, SessionState.IDLE, self.state_timeout)
            except UciError:
                pass  # deinit below still cleans up
            try:
                c.session_deinit(self.handle)
            except UciError:
                pass
            self.handle = None
        c.remove_listener(self._on_notification)

    def __enter__(self) -> "RangingSession":
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()

    # ---- results -------------------------------------------------------------

    def twr_range(self, peer: Optional[int] = None, timeout: float = 2.0) -> float:
        """
        Distance to `peer` in metres, from the first successful measurement
        that arrives after this call. `peer` may be omitted when there's only
        one. Raises UciTimeout if none arrives in time, UciError if the board
        ended the session.
        """
        return self.next_measurement(peer, timeout).distance_m

    def next_measurement(self, peer: Optional[int] = None, timeout: float = 2.0) -> TwoWayMeasurement:
        peer = self._resolve_peer(peer)
        deadline = time.monotonic() + timeout
        with self._cv:
            seen = self._ok_count.get(peer, 0)
            while self._ok_count.get(peer, 0) == seen:
                self._raise_if_ended()
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise UciTimeout(f"no successful measurement from 0x{peer:04X} within {timeout}s")
                self._cv.wait(remaining)
            return self._last_ok[peer]

    def latest(self, peer: Optional[int] = None) -> Optional[TwoWayMeasurement]:
        """Most recent successful measurement from `peer`, however old (None = none yet)."""
        with self._cv:
            return self._last_ok.get(self._resolve_peer(peer))

    def rounds(self, timeout: Optional[float] = None) -> Iterator[SessionInfoNtf]:
        """
        Every ranging round's raw results (including failed measurements), as
        they arrive. Ends when the session ends; raises UciTimeout if a round
        takes longer than `timeout`. Only buffers the 256 most recent rounds.
        """
        while True:
            try:
                rnd = self._rounds.get(timeout=timeout)
            except queue.Empty:
                raise UciTimeout(f"no ranging round within {timeout}s") from None
            if rnd is None:  # session ended
                return
            yield rnd

    # ---- internals -----------------------------------------------------------

    def _resolve_peer(self, peer: Optional[int]) -> int:
        if peer is not None:
            return peer
        if len(self.peers) != 1:
            raise ValueError("session has several peers; say which one")
        return self.peers[0]

    def _raise_if_ended(self):
        if self._ended is not None:
            raise UciError(f"session ended: {name_of(SessionState, self._ended.state)} "
                           f"({name_of(ReasonCode, self._ended.reason)})")

    def _on_notification(self, ntf: Notification):
        # Runs on the client's reader thread.
        if self.handle is None:
            return
        if isinstance(ntf, SessionInfoNtf) and ntf.session_handle == self.handle:
            with self._cv:
                for m in ntf.measurements:
                    if m.ok:
                        self._last_ok[m.mac_address] = m
                        self._ok_count[m.mac_address] = self._ok_count.get(m.mac_address, 0) + 1
                self._cv.notify_all()
            self._put_round(ntf)
        elif isinstance(ntf, SessionStatusNtf) and ntf.session_handle == self.handle:
            # Leaving ACTIVE, whether we asked (stop) or the board gave up
            # (e.g. MAX_RANGING_ROUND_RETRY_COUNT_REACHED).
            ended = self._active and ntf.state != SessionState.ACTIVE
            self._active = ntf.state == SessionState.ACTIVE
            if ended:
                with self._cv:
                    self._ended = ntf
                    self._cv.notify_all()
                self._put_round(None)

    def _put_round(self, rnd: Optional[SessionInfoNtf]):
        while True:
            try:
                self._rounds.put_nowait(rnd)
                return
            except queue.Full:
                try:
                    self._rounds.get_nowait()
                except queue.Empty:
                    pass
