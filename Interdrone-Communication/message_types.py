from typing import TypedDict


# Class used to structure the dictionaries for messaging via JSON
class MessageData(TypedDict):
    messageId: int
    # Update and add types here as needed
    data: dict[
        str, float | str | int | bool
    ]  # str is dict key. Value can be float, str, int, or bool
