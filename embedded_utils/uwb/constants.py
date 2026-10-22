"""
FiRa UCI 2.0 enumerations, as spoken by Qorvo's QM33 SDK UCI firmware on the
DWM3001CDK (GET_DEVICE_INFO reports UCI/MAC/PHY 2.0, UCI test 1.1).

Values follow the FiRa UCI Generic Technical Specification as implemented
byte-for-byte by AOSP `pica` (uci_packets.pdl). Anything a board sends that
isn't listed here is still delivered, just as a plain int.
"""

from enum import IntEnum


class MessageType(IntEnum):
    DATA = 0
    COMMAND = 1
    RESPONSE = 2
    NOTIFICATION = 3


class Gid(IntEnum):
    CORE = 0x0
    SESSION_CONFIG = 0x1
    SESSION_CONTROL = 0x2
    DATA_CONTROL = 0x3
    TEST = 0xD
    # 0x9-0xC, 0xE-0xF are vendor-specific (Qorvo calibration etc.)


class CoreOid(IntEnum):
    DEVICE_RESET = 0x00
    DEVICE_STATUS_NTF = 0x01
    GET_DEVICE_INFO = 0x02
    GET_CAPS_INFO = 0x03
    SET_CONFIG = 0x04
    GET_CONFIG = 0x05
    GENERIC_ERROR_NTF = 0x07
    QUERY_UWBS_TIMESTAMP = 0x08


class SessionConfigOid(IntEnum):
    INIT = 0x00
    DEINIT = 0x01
    STATUS_NTF = 0x02
    SET_APP_CONFIG = 0x03
    GET_APP_CONFIG = 0x04
    GET_COUNT = 0x05
    GET_STATE = 0x06
    UPDATE_CONTROLLER_MULTICAST_LIST = 0x07


class SessionControlOid(IntEnum):
    START = 0x00  # also the OID of SESSION_INFO_NTF (ranging results)
    STOP = 0x01
    GET_RANGING_COUNT = 0x03
    DATA_CREDIT_NTF = 0x04
    DATA_TRANSFER_STATUS_NTF = 0x05


SESSION_INFO_NTF = SessionControlOid.START


class Status(IntEnum):
    OK = 0x00
    REJECTED = 0x01
    FAILED = 0x02
    SYNTAX_ERROR = 0x03
    INVALID_PARAM = 0x04
    INVALID_RANGE = 0x05
    INVALID_MESSAGE_SIZE = 0x06
    UNKNOWN_GID = 0x07
    UNKNOWN_OID = 0x08
    READ_ONLY = 0x09
    UCI_MESSAGE_RETRY = 0x0A
    UNKNOWN = 0x0B
    NOT_APPLICABLE = 0x0C
    ERROR_SESSION_NOT_EXIST = 0x11
    ERROR_SESSION_DUPLICATE = 0x12
    ERROR_SESSION_ACTIVE = 0x13
    ERROR_MAX_SESSIONS_EXCEEDED = 0x14
    ERROR_SESSION_NOT_CONFIGURED = 0x15
    ERROR_ACTIVE_SESSIONS_ONGOING = 0x16
    ERROR_MULTICAST_LIST_FULL = 0x17
    ERROR_ADDRESS_ALREADY_PRESENT = 0x19
    ERROR_UWB_INITIATION_TIME_TOO_OLD = 0x1A
    OK_NEGATIVE_DISTANCE_REPORT = 0x1B
    # Per-measurement ranging statuses (show up in SESSION_INFO_NTF records)
    RANGING_TX_FAILED = 0x20
    RANGING_RX_TIMEOUT = 0x21
    RANGING_RX_PHY_DEC_FAILED = 0x22
    RANGING_RX_PHY_TOA_FAILED = 0x23
    RANGING_RX_PHY_STS_FAILED = 0x24
    RANGING_RX_MAC_DEC_FAILED = 0x25
    RANGING_RX_MAC_IE_DEC_FAILED = 0x26
    RANGING_RX_MAC_IE_MISSING = 0x27


class DeviceState(IntEnum):
    READY = 0x01
    ACTIVE = 0x02
    ERROR = 0xFF


class SessionState(IntEnum):
    INIT = 0x00
    DEINIT = 0x01
    ACTIVE = 0x02
    IDLE = 0x03


class ReasonCode(IntEnum):
    STATE_CHANGE_WITH_SESSION_MANAGEMENT_COMMANDS = 0x00
    MAX_RANGING_ROUND_RETRY_COUNT_REACHED = 0x01
    MAX_NUMBER_OF_MEASUREMENTS_REACHED = 0x02
    SESSION_SUSPENDED_DUE_TO_INBAND_SIGNAL = 0x03
    SESSION_RESUMED_DUE_TO_INBAND_SIGNAL = 0x04
    SESSION_STOPPED_DUE_TO_INBAND_SIGNAL = 0x05
    ERROR_INVALID_UL_TDOA_RANDOM_WINDOW = 0x1D
    ERROR_MIN_RFRAMES_PER_RR_NOT_SUPPORTED = 0x1E
    ERROR_TX_DELAY_NOT_SUPPORTED = 0x1F
    ERROR_SLOT_LENGTH_NOT_SUPPORTED = 0x20
    ERROR_INSUFFICIENT_SLOTS_PER_RR = 0x21
    ERROR_MAC_ADDRESS_MODE_NOT_SUPPORTED = 0x22
    ERROR_INVALID_RANGING_DURATION = 0x23
    ERROR_INVALID_STS_CONFIG = 0x24
    ERROR_INVALID_RFRAME_CONFIG = 0x25


class SessionType(IntEnum):
    FIRA_RANGING = 0x00
    FIRA_RANGING_AND_IN_BAND_DATA = 0x01
    FIRA_DATA_TRANSFER = 0x02
    DEVICE_TEST_MODE = 0xD0


class ResetConfig(IntEnum):
    UWBS_RESET = 0x00


class DeviceConfigId(IntEnum):
    """CORE_SET_CONFIG / CORE_GET_CONFIG parameter ids."""
    DEVICE_STATE = 0x00
    LOW_POWER_MODE = 0x01


class AppConfigId(IntEnum):
    """SESSION_SET_APP_CONFIG / SESSION_GET_APP_CONFIG parameter ids."""
    DEVICE_TYPE = 0x00
    RANGING_ROUND_USAGE = 0x01
    STS_CONFIG = 0x02
    MULTI_NODE_MODE = 0x03
    CHANNEL_NUMBER = 0x04
    NUMBER_OF_CONTROLEES = 0x05
    DEVICE_MAC_ADDRESS = 0x06
    DST_MAC_ADDRESS = 0x07
    SLOT_DURATION = 0x08
    RANGING_DURATION = 0x09
    STS_INDEX = 0x0A
    MAC_FCS_TYPE = 0x0B
    RANGING_ROUND_CONTROL = 0x0C
    AOA_RESULT_REQ = 0x0D
    SESSION_INFO_NTF_CONFIG = 0x0E
    NEAR_PROXIMITY_CONFIG = 0x0F
    FAR_PROXIMITY_CONFIG = 0x10
    DEVICE_ROLE = 0x11
    RFRAME_CONFIG = 0x12
    RSSI_REPORTING = 0x13
    PREAMBLE_CODE_INDEX = 0x14
    SFD_ID = 0x15
    PSDU_DATA_RATE = 0x16
    PREAMBLE_DURATION = 0x17
    LINK_LAYER_MODE = 0x18
    DATA_REPETITION_COUNT = 0x19
    RANGING_TIME_STRUCT = 0x1A
    SLOTS_PER_RR = 0x1B
    AOA_BOUND_CONFIG = 0x1D
    PRF_MODE = 0x1F
    CAP_SIZE_RANGE = 0x20
    TX_JITTER_WINDOW_SIZE = 0x21
    SCHEDULE_MODE = 0x22
    KEY_ROTATION = 0x23
    KEY_ROTATION_RATE = 0x24
    SESSION_PRIORITY = 0x25
    MAC_ADDRESS_MODE = 0x26
    VENDOR_ID = 0x27
    STATIC_STS_IV = 0x28
    NUMBER_OF_STS_SEGMENTS = 0x29
    MAX_RR_RETRY = 0x2A
    UWB_INITIATION_TIME = 0x2B
    HOPPING_MODE = 0x2C
    BLOCK_STRIDE_LENGTH = 0x2D
    RESULT_REPORT_CONFIG = 0x2E
    IN_BAND_TERMINATION_ATTEMPT_COUNT = 0x2F
    SUB_SESSION_ID = 0x30
    BPRF_PHR_DATA_RATE = 0x31
    MAX_NUMBER_OF_MEASUREMENTS = 0x32
    STS_LENGTH = 0x35


# ---- App config values -------------------------------------------------------

class DeviceType(IntEnum):
    CONTROLEE = 0x00
    CONTROLLER = 0x01


class DeviceRole(IntEnum):
    RESPONDER = 0x00
    INITIATOR = 0x01


class RangingRoundUsage(IntEnum):
    SS_TWR_DEFERRED = 0x01
    DS_TWR_DEFERRED = 0x02
    SS_TWR_NON_DEFERRED = 0x03
    DS_TWR_NON_DEFERRED = 0x04


class StsConfig(IntEnum):
    STATIC = 0x00
    DYNAMIC = 0x01
    DYNAMIC_FOR_RESPONDER_SUB_SESSION_KEY = 0x02
    PROVISIONED = 0x03
    PROVISIONED_FOR_RESPONDER_SUB_SESSION_KEY = 0x04


class MultiNodeMode(IntEnum):
    ONE_TO_ONE = 0x00
    ONE_TO_MANY = 0x01


class MacAddressMode(IntEnum):
    SHORT = 0x00          # 2-byte addresses everywhere
    EXTENDED_HEADER = 0x01  # 8-byte addresses in the MAC header only
    EXTENDED = 0x02       # 8-byte addresses everywhere


class SessionInfoNtfConfig(IntEnum):
    DISABLE = 0x00
    ENABLE = 0x01
    ENABLE_PROXIMITY_LEVEL_TRIG = 0x02


class MulticastAction(IntEnum):
    ADD = 0x00
    DELETE = 0x01


class RangingMeasurementType(IntEnum):
    ONE_WAY = 0x00
    TWO_WAY = 0x01
    DL_TDOA = 0x02
    OWR_AOA = 0x03


def name_of(enum_cls: type[IntEnum], value: int) -> str:
    """Enum member name, or hex for values the enum doesn't know (vendor codes)."""
    try:
        return enum_cls(value).name
    except ValueError:
        return f"0x{value:02X}"
