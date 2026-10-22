"""
Python API for the FiRa UCI protocol spoken by Qorvo DWM3001CDK boards running
the QM33 SDK UCI firmware.

    client.py    UciClient: one method per UCI command, notifications, state
    session.py   RangingSession: two-way ranging session -> distances
    messages.py  wire format, config codec, response/notification types
    constants.py enums (GIDs/OIDs, statuses, states, app config ids/values)

Try it on two boards with:  python -m embedded_utils.uwb.demo --help
"""

from .client import UciClient, list_ports
from .constants import (
    AppConfigId, CoreOid, DeviceConfigId, DeviceRole, DeviceState, DeviceType,
    Gid, MacAddressMode, MulticastAction, MultiNodeMode, RangingRoundUsage,
    ReasonCode, SessionConfigOid, SessionControlOid, SessionInfoNtfConfig,
    SessionState, SessionType, Status, StsConfig, name_of,
)
from .messages import (
    DeviceInfo, DeviceStatusNtf, GenericErrorNtf, SessionInfoNtf,
    SessionStatusNtf, TwoWayMeasurement, UciError, UciStatusError, UciTimeout,
    UnknownNtf, short_addresses,
)
from .session import RangingSession
