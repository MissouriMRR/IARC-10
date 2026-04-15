from typing import Any, Final, override
from enum import Enum
from typing import TypeAlias
from dataclasses import dataclass, field
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

    # Interdrone Communication
    HEARTBEAT = 504
    SERVER_DEFAULT_RESPONSE = 505
    SPEED_TEST_REQUEST = 513
    SPEED_TEST_RESPONSE = 514


SchemaFieldType: TypeAlias = (
    type[int] | type[float] | type[str] | type[tuple[Any, ...]] | type[dict[str, Any]] | MessageType
)

# If you need documentation for message types, see this document:
# https://mailmissouri-my.sharepoint.com/:w:/g/personal/mwhmp_umsystem_edu/EfdTOupRQj5Gp0HQwgEW91gBd5BsWHrW52WVym3LdiCsEQ?e=lBb2jx
EXPECTED_SCHEMA: Final[dict[MessageType, dict[str, Any]]] = {
    MessageType.UNKNOWN: {
        "id": MessageType.UNKNOWN,
        "dronesToSendData": tuple[int, ...],  # ... allows tuple to be any length
        "data": dict[str, object],
    },
    # For app messages, use (0) for dronesToSendData too if you wish to send data to the app
    MessageType.APP_TEST: {
        "id": MessageType.APP_TEST,
        "dronesToSendData": tuple[int, ...],
        "data": dict[str, object],  # TODO REMOVE DATA HERE
    },
    MessageType.APP_CONFIG: {
        "id": MessageType.APP_CONFIG,
        "dronesToSendData": tuple[int, ...],
        "IP": str,
        "Port": int,
    },
    MessageType.APP_DEBUG: {
        "id": MessageType.APP_DEBUG,
        "dronesToSendData": tuple[int, ...],
        "embeddedDebugMessage": str,
    },
    MessageType.SET_SCAN_STATUS: {
        "id": MessageType.SET_SCAN_STATUS,
        "dronesToSendData": tuple[int, ...],
        "setScanStatus": bool,
    },
    MessageType.SET_HOVER_STATUS: {
        "id": MessageType.SET_HOVER_STATUS,
        "dronesToSendData": tuple[int, ...],
        "setHoverStatus": bool,
        "height": float,
    },
    MessageType.REQUEST_MAP_DATA: {
        "id": MessageType.REQUEST_MAP_DATA,
        "dronesToSendData": tuple[int, ...],
    },
    MessageType.REQUEST_DRONE_LOCATIONS: {
        "id": MessageType.REQUEST_DRONE_LOCATIONS,
        "dronesToSendData": tuple[int, ...],  # VERIFY WE DON'T NEED DATA
    },
    MessageType.SEND_PATHS_TO_APP: {
        "id": MessageType.SEND_PATHS_TO_APP,
        "dronesToSendData": tuple[int, ...],
        "MapDataReady": bool,
    },
    MessageType.SEND_DRONE_LOCATIONS: {
        "id": MessageType.SEND_DRONE_LOCATIONS,
        "dronesToSendData": tuple[int, ...],
        "drone1Data": dict[str, list[int | float]],  # Contains latLong and xYCoords
        "drone2Data": dict[str, list[int | float]],
    },
    MessageType.SEND_APP_SCANNING_ERROR: {
        "id": MessageType.SEND_APP_SCANNING_ERROR,
        "dronesToSendData": tuple[int, ...],
        "errorType": int,
        "errorMessage": str,
    },
    MessageType.HEARTBEAT: {
        "id": MessageType.HEARTBEAT,
        "dronesToSendData": tuple[int, ...],
        "senderId": int,
        "payload": str,
    },
    MessageType.SERVER_DEFAULT_RESPONSE: {
        "id": MessageType.SERVER_DEFAULT_RESPONSE,
        "dronesToSendData": tuple[int, ...],
        "payload": str,
    },
    # Message: SPEED_TEST_REQUEST
    # Usage: Used in network_test. Sent to other drones, their server updates the data, and it's then sent back to the client for processing. Client outputs SPEED_TEST_RESPONSE
    # Description of Data: TODO Talk to team and see if we want to include this for message documentation
    MessageType.SPEED_TEST_REQUEST: {
        "id": MessageType.SPEED_TEST_REQUEST,
        "dronesToSendData": tuple[int, ...],
        "initialUploadTime": float,
        "finalUploadTime": float,
        "initialDownloadTime": float,
        "finalDownloadTime": float,
        "senderId": int,
        "payloadSize": int,
        "payload": str,
    },
    MessageType.SPEED_TEST_RESPONSE: {
        "id": MessageType.SPEED_TEST_RESPONSE,
        "dronesToSendData": tuple[int, ...],
        "target": str,
        "targetId": int,
        "uploadRttMs": float,
        "uploadThroughputKbps": float,
        "downloadRttMs": float,
        "downloadThroughputKbps": float,
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

    # ID and dronesToSendData are required values for each message
    id: MessageType
    dronesToSendData: tuple[int, ...]
    # The data variable contains all keys and values in a schema that aren't id and dronesToSendData.
    data: dict[str, Any] = field(default_factory=dict)  # TODO long term remove any

    # When new Message object is created in code, this runs to prevent errors and init
    def __post_init__(self) -> None:
        if self.id not in EXPECTED_SCHEMA:
            # Set to UNKNOWN to prevent errors
            warnings.warn(f"Message type not in schema: {self.id}, setting to UNKNOWN")
            object.__setattr__(self, "id", MessageType.UNKNOWN)
            object.__setattr__(self, "dronesToSendData", ())
            object.__setattr__(self, "data", {})
            return
        # Check to make sure all expected keys in schema are present in new message.
        expected = EXPECTED_SCHEMA[self.id]
        actual_keys = set(self.data.keys())
        expected_keys = set(expected.keys())

        expected_keys -= {"id", "dronesToSendData"}

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
            if key not in {"id", "dronesToSendData"}:
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
        cls, id: MessageType, dronesToSendData: tuple[int, ...], data: dict[str, Any]
    ) -> "Message":
        return cls(id=id, dronesToSendData=dronesToSendData, data=data.copy())


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
