from typing import TypedDict


# Class used to structure the dictionaries for messaging via JSON
class MessageData(TypedDict):
    messageId: int
    # Update and add types here as needed
    data: dict[str, float | str | int | bool]
