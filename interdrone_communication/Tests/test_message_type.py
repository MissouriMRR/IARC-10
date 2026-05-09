from message_types import Message, MessageType
import time

# Idea: test what happens if you create a message with too many keys


def test_creating_message():
    msg = Message.create(
        id=MessageType.HEARTBEAT,
        drones_to_send_data=(1, 2),
        sender_id = 3,
        data={
            "timestamp": time.time(),
            "payload": "Hello server!",
        },
    )
    print(msg)
