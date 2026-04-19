# Outside Imports
import json

# Interdrone Imports
from interdrone_communication.message_types import Message, MessageType


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

        return Message.create(
            id=message_type,
            dronesToSendData=tuple(data.get("dronesToSendData", ())),
            data=data.get("data", {}),
        )

    @staticmethod
    def message_to_json(message: Message) -> str:
        return json.dumps(
            {
                "id": message.id.value,
                "dronesToSendData": message.dronesToSendData,
                "data": message.data,
            }
        )
