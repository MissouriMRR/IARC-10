from __future__ import annotations
from dataclasses import dataclass, asdict, field
from enum import IntEnum
from typing import Any, Type, Dict, ClassVar
import json


# 1. Message IDs as IntEnum for JSON compatibility
class MessageIDs(IntEnum):
    UNKNOWN = 0
    # App
    APP_TEST = 400
    # Interdrone Communication
    HEARTBEAT = 504
    SPEED_TEST_REQUEST = 513
    SPEED_TEST_RESPONSE = 514


# Registry to map IDs to Classes for deserialization
_MESSAGE_TYPE_REGISTRY: Dict[int, Type[BaseMessage]] = {}


def register_message(cls):
    """Decorator to register message classes for deserialization."""
    # We instantiate a dummy version or inspect the type hint if possible,
    # but simplest is to trust the class has a 'ID' field or similar.
    # However, since messageId is an instance property in the base,
    # we enforce a ClassVar 'ID' for registration purposes.
    if hasattr(cls, "ID"):
        _MESSAGE_TYPE_REGISTRY[cls.ID] = cls
    return cls


# 2. Base Classes
@dataclass
class MessagePayload:
    """Base class for the 'data' dictionary content.
    Subclass this to define rigid fields for your data."""

    pass


# Top level message class
@dataclass
class BaseMessage:
    id: int = MessageIDs.UNKNOWN

    dronesToSendData: list[int] = field(default_factory=list)

    # Abstract-like field, should be defined by subclasses
    data: MessagePayload = field(default_factory=MessagePayload)

    @property
    def messageId(self) -> int:
        """Returns the ID associated with this message type."""
        if hasattr(self, "ID"):
            return self.ID  # type: ignore
        return MessageIDs.UNKNOWN

    def to_dict(self) -> dict[str, Any]:
        """Converts the message to a dictionary (JSON structure)."""
        return {
            "messageId": self.messageId,
            "dronesToSendData": self.dronesToSendData,
            "data": asdict(self.data),
        }

    def to_json(self) -> str:
        """Converts the message to a JSON string."""
        return json.dumps(self.to_dict())

    @staticmethod
    def from_json(json_str: str) -> BaseMessage:
        """Parses a JSON string into a specific BaseMessage subclass."""
        data_dict = json.loads(json_str)
        return BaseMessage.from_dict(data_dict)

    @staticmethod
    def from_dict(data_dict: dict[str, Any]) -> BaseMessage:
        """Factory method to create the correct message object from a dictionary."""
        msg_id = data_dict.get("messageId", 0)
        drones = data_dict.get("dronesToSendData", [])
        raw_data = data_dict.get("data", {})

        if msg_id in _MESSAGE_TYPE_REGISTRY:
            cls = _MESSAGE_TYPE_REGISTRY[msg_id]
            # Get the type of the 'data' field hint to instantiate the payload correctly
            # Note: This relies on the subclass iterating annotations or a simpler convention.
            # For simplicity here: we assume the subclass has a 'DataClass' attribute or we infer it.

            # Simple inference requires the subclass to define the type of 'data' clearly.
            # We can use the type hint of 'data' field in the subclass.
            payload_cls = cls.__annotations__["data"]

            # Create the payload object
            payload = payload_cls(**raw_data)

            # Create the message object
            return cls(dronesToSendData=drones, data=payload)

        # Fallback for unknown messages
        return BaseMessage(dronesToSendData=drones)


# 3. Example Implementations


# Example: Heartbeat
@dataclass
class HeartbeatPayload(MessagePayload):
    # Explicitly typed fields! No Any!
    status: str
    battery_level: float
    timestamp: float


@register_message
@dataclass
class HeartbeatMessage(BaseMessage):
    # Define the ID for this class
    ID: ClassVar[int] = MessageIDs.HEARTBEAT

    # Enforce strict typing for data
    data: HeartbeatPayload


# Example: Speed Test Request
@dataclass
class SpeedTestRequestPayload(MessagePayload):
    requestTime: float
    requesterName: str


@register_message
@dataclass
class SpeedTestRequestMessage(BaseMessage):
    ID: ClassVar[int] = MessageIDs.SPEED_TEST_REQUEST
    data: SpeedTestRequestPayload


# Example: Speed Test Response
@dataclass
class SpeedTestResponsePayload(MessagePayload):
    initialUploadTime: float | None = None
    finalUploadTime: float | None = None
    initialDownloadTime: float | None = None


@register_message
@dataclass
class SpeedTestResponseMessage(BaseMessage):
    ID: ClassVar[int] = MessageIDs.SPEED_TEST_RESPONSE
    data: SpeedTestResponsePayload
