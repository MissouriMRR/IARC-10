# Outside Imports
from typing import Any, Final, override
from enum import Enum
from typing import TypeAlias
from dataclasses import dataclass, field
from flight.waypoint import Waypoint
import json
import warnings
from typing import cast, get_args, get_origin
import types


class MessageType(Enum):
    UNKNOWN = 0
    # App
    APP_TEST = 400
    APP_CONFIG = 401
    APP_DEBUG = 402
    SET_SCAN_STATUS = 410
    SET_HOVER_STATUS = 412
    REQUEST_MAP_DATA = 414
    REQUEST_DRONE_LOCATIONS = 415
    SEND_PATHS_TO_APP = 420
    SEND_APP_SCANNING_ERROR = 421
    SEND_DRONE_LOCATIONS = 425
    SEND_APP_COORDS = 435
    SEND_APP_COORDS_ACK = 436

    # Interdrone Communication
    HEARTBEAT = 504
    SERVER_DEFAULT_RESPONSE = 505
    SPEED_TEST_REQUEST = 513
    SPEED_TEST_RESPONSE = 514
    ARM = 520
    ARM_ACK = 521
    ARM_NACK = 522
    DISARM = 523
    PING = 525
    PING_ACK = 526
    PING_NACK = 527
    START_TAKEOFF = 530
    START_TAKEOFF_ACK = 531
    START_DEMO = 535
    START_DEMO_ACK = 536
    DEMO_DONE = 537
    START_MISSION = 540
    START_MISSION_ACK = 541
    NEW_WAYPOINTS = 545
    NEW_WAYPOINTS_ACK = 546
    REACHED_WAYPOINT = 550
    REACHED_WAYPOINT_ACK = 551
    EMERGENCY_LAND = 555
    LAND = 556
    RECONFIRM_WAYPOINTS = 560
    SURVEY_START = 565
    SURVEY_START_ACK = 566
    SURVEY_END = 570
    SURVEY_END_ACK = 571
    SHARE_PHOTOS = 575
    FIELD_CHECKSUM = 580
    MISSION_END = 585
    MISSION_END_ACK = 586


SchemaFieldType: TypeAlias = (
    type[int] | type[float] | type[str] | type[tuple[Any, ...]] | type[dict[str, Any]] | MessageType
)

# If you need documentation for message types, see this document:
# https://mailmissouri-my.sharepoint.com/:w:/g/personal/mwhmp_umsystem_edu/EfdTOupRQj5Gp0HQwgEW91gBd5BsWHrW52WVym3LdiCsEQ?e=lBb2jx
EXPECTED_SCHEMA: Final[dict[MessageType, dict[str, Any]]] = {
    MessageType.UNKNOWN: {
        "id": MessageType.UNKNOWN,
        "drones_to_send_data": tuple[int, ...],  # ... allows tuple to be any length
        "sender_id": int,
        "data": dict[str, object],
    },
    # For app messages, use (0) for drones_to_send_data too if you wish to send data to the app
    MessageType.APP_TEST: {
        "id": MessageType.APP_TEST,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
        "data": dict[str, object],  # TODO REMOVE DATA HERE
    },
    MessageType.APP_CONFIG: {
        "id": MessageType.APP_CONFIG,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
        "ip": str,
        "port": int,
    },
    MessageType.APP_DEBUG: {
        "id": MessageType.APP_DEBUG,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
        "embedded_debug_message": str,
    },
    MessageType.SET_SCAN_STATUS: {
        "id": MessageType.SET_SCAN_STATUS,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
        "set_scan_status": bool,
    },
    MessageType.SET_HOVER_STATUS: {
        "id": MessageType.SET_HOVER_STATUS,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
        "set_hover_status": bool,
        "height": float,
    },
    MessageType.REQUEST_MAP_DATA: {
        "id": MessageType.REQUEST_MAP_DATA,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
    MessageType.REQUEST_DRONE_LOCATIONS: {
        "id": MessageType.REQUEST_DRONE_LOCATIONS,
        "drones_to_send_data": tuple[int, ...],  # VERIFY WE DON'T NEED DATA
        "sender_id": int,
    },
    MessageType.SEND_PATHS_TO_APP: {
        "id": MessageType.SEND_PATHS_TO_APP,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
        "map_data_ready": bool,
    },
    MessageType.SEND_DRONE_LOCATIONS: {
        "id": MessageType.SEND_DRONE_LOCATIONS,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
        "drone1_data": dict[str, list[int | float]],  # Contains latLong and xYCoords
        "drone2_data": dict[str, list[int | float]],
    },
    MessageType.SEND_APP_SCANNING_ERROR: {
        "id": MessageType.SEND_APP_SCANNING_ERROR,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
        "errorType": int,
        "error_message": str,
    },
    MessageType.HEARTBEAT: {
        "id": MessageType.HEARTBEAT,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
        "payload": str,
    },
    MessageType.SERVER_DEFAULT_RESPONSE: {
        "id": MessageType.SERVER_DEFAULT_RESPONSE,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
        "payload": str,
    },
    MessageType.SEND_APP_COORDS: {
        "id": MessageType.SEND_APP_COORDS,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
        "app_latitude": float,
        "app_longitude": float,
    },
    MessageType.SEND_APP_COORDS_ACK: {
        "id": MessageType.SEND_APP_COORDS_ACK,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
    # Message: SPEED_TEST_REQUEST
    # Usage: Used in network_test. Sent to other drones, their server updates the data, and it's then sent back to the client for processing. Client outputs SPEED_TEST_RESPONSE
    # Description of Data: TODO Talk to team and see if we want to include this for message documentation
    MessageType.SPEED_TEST_REQUEST: {
        "id": MessageType.SPEED_TEST_REQUEST,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
        "initial_upload_time": float,
        "payload_size": int,
        "payload": str,
    },
    MessageType.SPEED_TEST_RESPONSE: {
        "id": MessageType.SPEED_TEST_RESPONSE,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
        "initial_upload_time": float,
        "final_upload_time": float,
        "initial_download_time": float,  # Don't need a final since it will be computed along with other values
        "target_id": int,
        "upload_rtt_ms": float,
        "upload_throughput_kbps": float,
        "download_rtt_ms": float,
        "download_throughput_kbps": float,
        "payload": str,
    },
    MessageType.ARM: {
        "id": MessageType.ARM,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
    MessageType.ARM_ACK: {
        "id": MessageType.ARM_ACK,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
    MessageType.ARM_NACK: {
        "id": MessageType.ARM_NACK,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
    MessageType.DISARM: {
        "id": MessageType.DISARM,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
    MessageType.PING: {
        "id": MessageType.PING,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
    MessageType.PING_ACK: {
        "id": MessageType.PING_ACK,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
    MessageType.PING_NACK: {
        "id": MessageType.PING_NACK,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
    MessageType.START_TAKEOFF: {
        "id": MessageType.START_TAKEOFF,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
    MessageType.START_TAKEOFF_ACK: {
        "id": MessageType.START_TAKEOFF_ACK,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
    MessageType.START_DEMO: {
        "id": MessageType.START_DEMO,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
    MessageType.START_DEMO_ACK: {
        "id": MessageType.START_DEMO_ACK,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
    MessageType.DEMO_DONE: {
        "id": MessageType.DEMO_DONE,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
    MessageType.START_MISSION: {
        "id": MessageType.START_MISSION,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
    MessageType.START_MISSION_ACK: {
        "id": MessageType.START_MISSION_ACK,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
    MessageType.NEW_WAYPOINTS: {
        "id": MessageType.NEW_WAYPOINTS,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
        "newWaypoints": list[Waypoint],
        "sender_drone_waypoints_checksum": int,
    },
    MessageType.NEW_WAYPOINTS_ACK: {
        "id": MessageType.NEW_WAYPOINTS_ACK,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
    MessageType.REACHED_WAYPOINT: {
        "id": MessageType.REACHED_WAYPOINT,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
        "reached_waypoint_id": int,
    },
    MessageType.REACHED_WAYPOINT_ACK: {
        "id": MessageType.REACHED_WAYPOINT_ACK,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
    MessageType.EMERGENCY_LAND: {
        "id": MessageType.EMERGENCY_LAND,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
    MessageType.LAND: {
        "id": MessageType.LAND,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
    MessageType.RECONFIRM_WAYPOINTS: {
        "id": MessageType.RECONFIRM_WAYPOINTS,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
        "all_waypoints": list[Waypoint],
        "need_response": bool,
    },
    MessageType.SURVEY_START: {
        "id": MessageType.SURVEY_START,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
    MessageType.SURVEY_START_ACK: {
        "id": MessageType.SURVEY_START_ACK,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
    MessageType.SURVEY_END: {
        "id": MessageType.SURVEY_END,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
    MessageType.SURVEY_END_ACK: {
        "id": MessageType.SURVEY_END_ACK,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
    MessageType.SHARE_PHOTOS: {
        "id": MessageType.SHARE_PHOTOS,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
        "photos": list[
            dict[str, Any]
        ],  # Each photo has cornerCoordinates (4 corner tuples) and mines (list with coordinate tuples)
        # TODO: Update once photos are defined better
    },
    MessageType.FIELD_CHECKSUM: {
        "id": MessageType.FIELD_CHECKSUM,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
        "checksum": int,
    },
    MessageType.MISSION_END: {
        "id": MessageType.MISSION_END,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
    MessageType.MISSION_END_ACK: {
        "id": MessageType.MISSION_END_ACK,
        "drones_to_send_data": tuple[int, ...],
        "sender_id": int,
    },
}


def _matches_type(value: object, expected_type: object) -> bool:
    # Treat Any as a wildcard
    if expected_type is Any:
        return True

    origin: object | None = get_origin(expected_type)
    args: tuple[object, ...] = cast(tuple[object, ...], get_args(expected_type))

    # Non-parameterized runtime types (int, str, dict, tuple, etc.)
    if origin is None:
        return isinstance(expected_type, type) and isinstance(value, expected_type)

    # PEP 604 unions: X | Y
    if origin is types.UnionType:
        return any(_matches_type(value, member_type) for member_type in args)

    # list[T]
    if origin is list:
        if not isinstance(value, list):
            return False
        items = cast(list[object], value)
        element_type: object = args[0] if args else object
        return all(_matches_type(item, element_type) for item in items)

    # dict[K, V]
    if origin is dict:
        if not isinstance(value, dict):
            return False
        mapping = cast(dict[object, object], value)
        key_type: object
        value_type: object
        if len(args) == 2:
            key_type, value_type = args
        else:
            key_type, value_type = object, object
        return all(
            _matches_type(k, key_type) and _matches_type(v, value_type) for k, v in mapping.items()
        )

    # tuple[T, ...] or tuple[T1, T2, ...]
    if origin is tuple:
        if not isinstance(value, tuple):
            return False
        items = cast(tuple[object, ...], value)

        if len(args) == 2 and args[1] is Ellipsis:
            return all(_matches_type(item, args[0]) for item in items)

        if len(args) != len(items):
            return False
        return all(_matches_type(item, expected) for item, expected in zip(items, args))

    # Fallback for other generic origins that are normal runtime classes
    if isinstance(origin, type):
        return isinstance(value, origin)

    return False


@dataclass(frozen=True)
class Message:
    """Message class for interdrone communication. Provides a way to create, validate, and serialize messages."""

    # ID, drones_to_send_data, and sender_id are required values for each message
    id: MessageType
    drones_to_send_data: tuple[int, ...]
    sender_id: int
    # The data variable contains all keys and values in a schema that aren't id and drones_to_send_data.
    data: dict[str, Any] = field(default_factory=dict)  # TODO long term remove any

    # When new Message object is created in code, this runs to prevent errors and init
    def __post_init__(self) -> None:
        if self.id not in EXPECTED_SCHEMA:
            # Set to UNKNOWN to prevent errors
            warnings.warn(f"Message type not in schema: {self.id}, setting to UNKNOWN")
            object.__setattr__(self, "id", MessageType.UNKNOWN)
            object.__setattr__(self, "drones_to_send_data", ())
            object.__setattr__(self, "sender_id", ())
            object.__setattr__(self, "data", {})
            return
        # Check to make sure all expected keys in schema are present in new message.
        expected = EXPECTED_SCHEMA[self.id]
        actual_keys = set(self.data.keys())
        expected_keys = set(expected.keys())

        expected_keys -= {"id", "drones_to_send_data", "sender_id"}

        # Check missing/extra keys
        missing = expected_keys - actual_keys
        extra = actual_keys - expected_keys

        errors: list[str] = []
        if missing:
            errors.append(f"missing keys: {sorted(missing)}")
        if extra:
            errors.append(f"extra keys: {sorted(extra)}")
        if errors:
            warnings.warn(f"Invalid data for message '{self.id}': {', '.join(errors)}")
            warnings.warn("Setting data to empty dict and skipping validation")
            object.__setattr__(self, "data", {})
            return

        # Iterate through schema and add keys to data and check for errors
        for key, allowed_types in expected.items():
            if key not in {"id", "drones_to_send_data", "sender_id"}:
                value = self.data[key]
                if not _matches_type(value, allowed_types):
                    warnings.warn(
                        f"Field '{key}' in '{self.id}' must be one of {allowed_types}, got {type(value).__name__}"
                    )
                    warnings.warn("Setting data to empty dict and skipping validation")
                    object.__setattr__(self, "data", {})
                    return
        object.__setattr__(self, "data", FrozenKeysDict(self.data.copy()))

    @classmethod
    def create(
        cls,
        id: MessageType,
        drones_to_send_data: tuple[int, ...],
        sender_id: int,
        data: dict[str, Any],
    ) -> "Message":
        return cls(
            id=id,
            drones_to_send_data=drones_to_send_data,
            sender_id=sender_id,
            data=data.copy(),
        )


# Ensures you can't change values of keys once a message is created, only their values (prevents potential issues with networking)
class FrozenKeysDict(dict[str, Any]):
    """Normal dict, but you can't add or remove keys after creation"""

    def __init__(self, data: dict[str, Any]):
        super().__init__(data)
        self._frozen_keys = frozenset(data.keys())

    @override
    def __setitem__(self, key, value):
        if key not in self._frozen_keys:
            warnings.warn(f"Cannot add new key '{key}' – message structure is frozen")
            return
        super().__setitem__(key, value)

    @override
    def __delitem__(self, key):
        warnings.warn("Cannot delete keys from Message.data")
        return

    @override
    def clear(self):
        warnings.warn("Cannot clear Message.data")
        return

    @override
    def pop(self, *args):
        warnings.warn("Cannot pop from Message.data")
        return

    @override
    def popitem(self):
        warnings.warn("Cannot popitem from Message.data")
        return tuple[str, None]()  # Prevent return type error

    @override
    def setdefault(self, key, default=None):
        if key not in self._frozen_keys:
            warnings.warn(f"Cannot add new key '{key}' via setdefault()")
            return
        return super().setdefault(key, default)

    @override
    def update(self, other=None, **kwargs):
        if other is not None:
            for k in other:
                if k not in self._frozen_keys:
                    warnings.warn(f"Cannot add new key '{k}' via update()")
                    return
        for k in kwargs:
            if k not in self._frozen_keys:
                warnings.warn(f"Cannot add new key '{k}' via update()")
                return
        super().update(other if other is not None else {}, **kwargs)

    def to_json(self) -> str:
        return json.dumps(self.__dict__)
