"""
Velocity in units of ft/s
Acceleration in units of ft/s^2
Time in seconds
"""
from Task import Task
from collections.abc import Iterator


"""
Class for representing a drone and its function. If needed a subclass can be made for different drones.
"""
class Drone:
    def __init__(self, max_speed: float = 2, acc: float = 1, pos: tuple[float, float] = (0, 0)):
        self.max_speed: float = max_speed
        self.acc: float = acc
        self.pos: tuple[float, float] = pos

    """
    Contains the logic for the drone. Yields tasks when necassary and operates on task results if necessary.
    """
    def task_generator() -> Iterator["Task"]:
        pass
