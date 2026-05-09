# Outside Imports
import unittest

# Interdrone Imports
from interdrone_communication.message_types import Message, MessageType


class TestMessages(unittest.TestCase):
    def test_message_creation(self):
        message = Message.create(
            id=MessageType.HEARTBEAT,
            drones_to_send_data=(),
            sender_id = 1,
            data={
                "payload": "Hello, world!",
            },
        )
        self.assertEqual(message.id, MessageType.HEARTBEAT)
        self.assertEqual(message.drones_to_send_data, ())
        self.assertEqual(
            message.data,
            {
                "sender_id": 1,
                "payload": "Hello, world!",
            },
        )

    # This test doesn't work, and isn't really necessary since the MessageType
    # enum will throw an error if it is not a valid MessageType. However,
    # I will leave it here in case that ends up changing/I find a way to trigger
    # the warning without also throwing an error.

    # def test_wrong_ID(self):
    #     with self.assertWarns(Warning):
    #         message = Message.create(
    #             id=MessageType(-1),
    #             drones_to_send_data=(),
    #             data={
    #                 "sender_id": 1,
    #                 "payload": "Hello, world!",
    #             },
    #         )
    #         self.assertEqual(message.id, MessageType.UNKNOWN)
    #         self.assertEqual(message.data, {})

    def test_missing_keys(self):
        with self.assertWarns(Warning):
            message = Message.create(
                id=MessageType.HEARTBEAT,
                drones_to_send_data=(),
                data={
                    "sender_id": 1,
                },
            )
            self.assertEqual(message.data, {})

    def test_extra_keys(self):
        with self.assertWarns(Warning):
            message = Message.create(
                id=MessageType.HEARTBEAT,
                drones_to_send_data=(),
                data={
                    "sender_id": 1,
                    "payload": "Hello, world!",
                    "extra": "extra",
                },
            )
            self.assertEqual(message.data, {})

    def test_wrong_type(self):
        with self.assertWarns(Warning):
            message = Message.create(
                id=MessageType.HEARTBEAT,
                drones_to_send_data=(),
                data={"sender_id": 1, "payload": 42},
            )
            self.assertEqual(message.data, {})

    def test_frozen_keys(self):
        message = Message.create(
            id=MessageType.HEARTBEAT,
            drones_to_send_data=(),
            data={
                "sender_id": 1,
                "payload": "Hello, world!",
            },
        )
        with self.assertWarns(Warning):
            message.data["extra"] = "extra"
        with self.assertWarns(Warning):
            del message.data["sender_id"]
        with self.assertWarns(Warning):
            message.data.clear()
        with self.assertWarns(Warning):
            message.data.pop("sender_id")
        with self.assertWarns(Warning):
            _ = message.data.popitem()
        with self.assertWarns(Warning):
            message.data.setdefault("extra", "extra")
        with self.assertWarns(Warning):
            message.data.update({"extra": "extra"})


if __name__ == "__main__":
    unittest.main()
