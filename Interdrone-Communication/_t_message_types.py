from typing import Dict, List, Set, Type, Any, Final, experimental
from enum import Enum
from dataclasses import dataclass, field
from __future__ import annotations

import json



# This file is a work in progress. You may test it out, but do not use it for
# production code. It is not yet fully functional and may not work as expected.




class MessageType(Enum):
    UNKNOWN = 0

    # App
    APP_TEST = 400

    # Interdrone Communication
    HEARTBEAT = 504
    SPEED_TEST_REQUEST = 513
    SPEED_TEST_RESPONSE = 514

EXPECTED_SCHEMA: Final[Dict[MessageType, Dict[str, type]]] = {
    MessageType.UNKNOWN: {
        "id": MessageType.UNKNOWN,
        "drones": tuple[int, ...],
        "data": dict[str, Any]
    },
    MessageType.APP_TEST: {
        "id": MessageType.APP_TEST,
        "drones": tuple[int, ...],
        "data": dict[str, Any]
    },
    MessageType.HEARTBEAT: {
        "id": MessageType.HEARTBEAT,
        "drones": tuple[int, ...],
        "timestamp": float,
        "senderId": int,
        "payload": "Hello server!",
    },
    MessageType.SPEED_TEST_REQUEST: {
        "id": MessageType.SPEED_TEST_REQUEST,
        "drones": tuple[int, ...],
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
        "drones": tuple[int, ...],
        "target": str,
        "uploadRttMs": float,
        "uploadThroughputKbps": float,
        "downloadRttMs": float,
        "downloadThroughputKbps": float,
    },
}

@dataclass(frozen=True)
@experimental
class Message:
    """Message class for interdrone communication. Provides a way to create, validate, and serialize messages."""
    id: MessageType
    drones: tuple[int, ...]
    data: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.id is None:
            raise ValueError("ID is required, cannot be None")
        if self.drones is None:
            raise ValueError("Drones is required, cannot be None")
        if self.data is None:
            raise ValueError("Data is required, cannot be None")
        if self.id not in EXPECTED_SCHEMA:
            raise ValueError(f"Invalid message type: {self.id}")

        expected = EXPECTED_SCHEMA[self.id]
        actual_keys = set(self.data.keys())
        expected_keys = set(expected.keys())

        expected_keys -= {"id", "drones"}

        # Check missing/extra keys
        missing = expected_keys - actual_keys
        extra = actual_keys - expected_keys

        errors = []
        if missing:
            errors.append(f"missing keys: {sorted(missing)}")
        if extra:
            errors.append(f"extra keys: {sorted(extra)}")
        if errors:
            raise ValueError(f"Invalid data for message '{self.id}': {', '.join(errors)}")

        for key, allowed_types in expected.items():
            if key not in {"id", "drones"}:
                value = self.data[key]
                if not isinstance(value, allowed_types):
                    raise TypeError(
                        f"Field '{key}' in '{self.id}' must be one of {allowed_types}, "
                        f"got {type(value).__name__}"
                )
        object.__setattr__(self, "data", FrozenKeysDict(self.data.copy()))

    @classmethod
    def create(cls, id: str, drones: List[str], data: Dict[str, Any]) -> "Message":
        return cls(id=id, drones=tuple(drones), data=data.copy())

@experimental
class FrozenKeysDict(dict):
    """Normal dict, but you can't add or remove keys after creation"""
    def __init__(self, data: dict):
        super().__init__(data)
        self._frozen_keys = frozenset(data.keys())

    def __setitem__(self, key, value):
        if key not in self._frozen_keys:
            raise KeyError(f"Cannot add new key '{key}' – message structure is frozen")
        super().__setitem__(key, value)

    def __delitem__(self, key):
        raise KeyError("Cannot delete keys from Message.data")

    def clear(self): raise AttributeError("Cannot clear Message.data")
    def pop(self, *args): raise AttributeError("Cannot pop from Message.data")
    def popitem(self): raise AttributeError("Cannot popitem from Message.data")
    def setdefault(self, key, default=None):
        if key not in self._frozen_keys:
            raise KeyError(f"Cannot add new key '{key}' via setdefault()")
        return super().setdefault(key, default)

    def update(self, other=None, **kwargs):
        if other is not None:
            for k in other:
                if k not in self._frozen_keys:
                    raise KeyError(f"Cannot add new key '{k}' via update()")
        for k in kwargs:
            if k not in self._frozen_keys:
                raise KeyError(f"Cannot add new key '{k}' via update()")
        super().update(other, **kwargs)
        

    def to_json(self) -> str:
        return json.dumps(self.__dict__)
    
