from typing import TypedDict


class MessageData(TypedDict):
    messageId: int
    # Update and add types here as needed
    data: dict[str, float | str | int | bool]
