from typing import Any, TypedDict


# Class used to structure the dictionaries for messaging via JSON
class MessageData(TypedDict):
    messageId: int
    dronesToSendData: list[int]
    # Update and add types here as needed
    data: dict[str, Any]  # Data has a str type and can be any type


class JsonConfigData(TypedDict):
    drones: dict[str, dict[str, str]]
    app: dict[str, str | int]
    localInfo: dict[str, int]
