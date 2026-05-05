# Outside Imports
import json
from typing import get_args, get_origin

# Interdrone Imports
from interdrone_communication.message_types import EXPECTED_SCHEMA, Message, MessageType
from flight.waypoint import Waypoint


# Contains message to help convert Messages to and from JSON
# NOTE please review this! These methods may be better implemented in message_types.py but this is just a clean way for now.
class JsonMessageUtilities:
    # Static method allows us to not take self parameter
    @staticmethod
    def message_from_json(payload: str) -> Message:
        data = json.loads(payload)
        messageIdValue = data.get("id")

        try:
            message_type = MessageType(messageIdValue)
        except ValueError:
            print(f"Unknown message id received: {messageIdValue!r}, defaulting to UNKNOWN")
            message_type = MessageType.UNKNOWN

        nested_data: dict = data.get("data", {})

        # Reconstruct Waypoint objects for fields that the schema declares as list[Waypoint]
        schema = EXPECTED_SCHEMA.get(message_type, {})
        for key, expected_type in schema.items():
            if key in nested_data and get_origin(expected_type) is list:
                args = get_args(expected_type)
                if args and args[0] is Waypoint:
                    nested_data[key] = [Waypoint.from_dict(wp) for wp in nested_data[key]]

        return Message.create(
            id=message_type,
            dronesToSendData=tuple(data.get("dronesToSendData", ())),
            senderId=data.get("senderId"),
            data=nested_data,
        )

    @staticmethod
    def message_to_json(message: Message) -> str:
        def _default(obj: object) -> object:
            if isinstance(obj, Waypoint):
                return obj.to_dict()
            raise TypeError(f"Object of type {type(obj).__name__} is not JSON serializable")

        return json.dumps(
            {
                "id": message.id.value,
                "dronesToSendData": message.dronesToSendData,
                "senderId": message.senderId,
                "data": message.data,
            },
            default=_default,
        )
