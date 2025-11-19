from typing import TypedDict


# Class used to structure the dictionaries for messaging via JSON
class MessageData(TypedDict):
    messageId: int
    dronesToSendData: list[int]
    # Update and add types here as needed
    data: dict[
        str, float | str | int | bool
    ]  # str is dict key. Value can be float, str, int, or bool


class JsonConfigData(TypedDict):
    drones: dict[str, dict[str, str]]
    localInfo: dict[str, int]
