from typing import Any, Final, override
from enum import Enum
from typing import TypeAlias
from dataclasses import dataclass, field
import json
import warnings


class MessageType(Enum):
    UNKNOWN = 0

    # App
    APP_TEST = 400
    APP_CONFIG = 401
    APP_DEBUG = 402

    # Interdrone Communication
    HEARTBEAT = 504
    SERVER_DEFAULT_RESPONSE = 505
    SPEED_TEST_REQUEST = 513
    SPEED_TEST_RESPONSE = 514


SchemaFieldType: TypeAlias = (
    type[int]
    | type[float]
    | type[str]
    | type[tuple[Any, ...]]
    | type[dict[str, Any]]
    | MessageType
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
    MessageType.APP_DEBUG: {
        "id": MessageType.APP_TEST,
        "dronesToSendData": tuple[int, ...],
        "embeddedDebugMessage": dict[str, Any],
    },
    MessageType.APP_CONFIG: {
        "id": MessageType.APP_CONFIG,
        "dronesToSendData": tuple[int, ...],
        "IP": str,
        "Port": int,
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
        "uploadRttMs": float,
        "uploadThroughputKbps": float,
        "downloadRttMs": float,
        "downloadThroughputKbps": float,
    },
}


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
                if not isinstance(value, allowed_types):
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
