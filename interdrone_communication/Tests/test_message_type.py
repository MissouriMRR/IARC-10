from message_types import Message, MessageType
import time

# Idea: test what happens if you create a message with too many keys


def test_creating_message():
    msg = Message.create(
        id=MessageType.HEARTBEAT,
        dronesToSendData=(1, 2),
        data={
            "timestamp": time.time(),
            "senderId": 3,
            "payload": "Hello server!",
        },
    )
    print(msg)
