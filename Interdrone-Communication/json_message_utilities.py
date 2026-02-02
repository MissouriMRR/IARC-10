from _t_message_types import Message, MessageType
import json


# Contains message to help convert Messages to and from JSON
# NOTE please review this! These methods may be better implemented in message_types.py but this is just a clean way for now.
class JsonMessageUtilities:
    # Static method allows us to not take self parameter
    @staticmethod
    def message_from_json(payload: str) -> Message:
        data = json.loads(payload)
        message_id_value = data.get("messageId", data.get("id"))
        return Message.create(
            id=MessageType(message_id_value),
            dronesToSendData=tuple(data.get("dronesToSendData", ())),
            data=data.get("data", {}),
        )

    @staticmethod
    def message_to_json(message: Message) -> str:
        return json.dumps(
            {
                "messageId": message.id.value,  # or "id" if your server expects that
                "dronesToSendData": message.dronesToSendData,
                "data": message.data,
            }
        )
