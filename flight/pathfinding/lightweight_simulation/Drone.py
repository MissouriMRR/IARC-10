"""
Velocity in units of ft/s
Acceleration in units of ft/s^2
Time in seconds
"""
from Task import Task
from typing import Any, Generator
import math
from enum import Enum


"""
Class for representing a drone and its function. If needed a subclass can be made for different drones.
"""
class Drone:
    drone_id = 0
    def __init__(self, max_speed: float = 2, acc: float = 1, pos: tuple[float, float] = (0, 0), alt: float = 0) -> None:
        self.max_speed: float = max_speed
        self.speed: float = 0
        self.acc: float = acc
        self.pos: tuple[float, float] = pos
        self.alt: float = alt
        Drone.drone_id += 1
        self.id: int = Drone.drone_id

    """
    Contains the logic for the drone. Yields tasks when necassary and operates on task results if necessary.
    
    """
    def task_generator(self) -> Generator[Task, Any, Any]:
        yield ChangeAltitudeTask(self, 10)
        

    def set_position(self, pos: tuple[float, float]) -> None:
        self.pos: tuple[float, float] = pos

    def get_position(self) -> tuple[float, float]:
        return self.pos

    def set_altitude(self, alt: float) -> None:
        self.alt: float = alt

    def get_altitude(self) -> float:
        return self.alt

    def get_max_speed(self) -> float:
        return self.max_speed

    def get_acceleration(self) -> float:
        return self.acc

    def set_speed(self, speed: float) -> None:
        self.speed: float = speed

    def get_speed(self) -> float:
        return self.speed


"""
Task class for moving a drone between two points horizontally (constant altitude)
"""
class GoToTask(Task):
    def __init__(self, drone: Drone, new_pos: tuple[float, float]) -> None:
        self.drone: Drone = drone
        self.start_pos: tuple[float, float] = self.drone.get_position()
        self.new_pos: tuple[float, float] = new_pos
        self.finished: bool = False
        self.time: float = 0
        self.movement_sequence: MovementSequence = MovementSequence(self.distance(self.start_pos, self.new_pos), self.drone.get_acceleration(), self.drone.get_max_speed())
        self.duration: float = self.movement_sequence.get_duration()

    def distance(self, point1: tuple[float, float], point2: tuple[float, float]) -> float:
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

    def calc_remaining_distance(self) -> float:
        return self.distance(self.drone.get_position(), self.new_pos)

    def get_remaining_time(self) -> float:
        return self.duration - self.time

    def run_for_time(self, time: float) -> tuple[bool, None]:
        self.time += time
        self.drone.set_position((self.start_pos[0] + (self.movement_sequence.get_dist_at_time(time)/self.distance(self.start_pos, self.new_pos))*(self.new_pos[0] - self.start_pos[0]),self.start_pos[1] + (self.movement_sequence.get_dist_at_time(time)/self.distance(self.start_pos, self.new_pos))*(self.new_pos[1] - self.start_pos[1])))
        self.drone.set_speed(self.movement_sequence.get_speed_at_time(self.time))
        if self.time >= self.duration:
            self.finished = True
            return (True, None)
        return (False, None)

"""
Task class for moving a drone to a new altitude
"""
class ChangeAltitudeTask(Task):
    def __init__(self, drone: Drone, new_alt: float) -> None:
        self.drone: Drone = drone
        self.start_alt: float = self.drone.get_altitude()
        self.new_alt: float = new_alt
        self.finished: bool = False
        self.time: float = 0
        self.movement_sequence: MovementSequence = MovementSequence(self.new_alt - self.start_alt, self.drone.get_acceleration(), self.drone.get_max_speed())
        self.duration: float = self.movement_sequence.get_duration()

    def calc_remaining_distance(self) -> float:
        return abs(self.drone.get_altitude() - self.new_alt)

    def get_remaining_time(self) -> float:
        return self.duration - self.time

    def run_for_time(self, time: float) -> tuple[bool, None]:
        self.time += time
        self.drone.set_altitude(self.start_alt + ((self.new_alt-self.start_alt)/(abs(self.new_alt-self.start_alt)))*self.movement_sequence.get_dist_at_time(self.time))
        self.drone.set_speed(self.movement_sequence.get_speed_at_time(self.time))
        if self.time >= self.duration:
            self.finished = True
            return (True, None)
        return (False, None)

class MovementStage(Enum):
    ACCELERATING = 1
    CONSTANT_SPEED = 2
    DECELERATING = 3

class MovementSequence:
    def __init__(self, dist: float, acc:float , max_speed: float) -> None:
        self.dist: float = dist
        self.acc: float = acc
        self.max_speed: float = max_speed
        self.sequence: list[tuple[MovementStage, float, float]] = self.gen_sequence()
        self.duration: float = self.sequence[-1][2]

    """
    Generates a sequence of moves for the drone, each move is a tuple of the type of move, the move start time, and the move end time
    """
    def gen_sequence(self) -> list[tuple[MovementStage, float, float]]:
        sequence: list[tuple[MovementStage, float, float]] = []
        max_accel_time: float = self.max_speed/self.acc
        max_accel_distance: float = 0.5*(self.acc*(max_accel_time**2)) # Distance the drone travels accelerating from 0 to max speed
        if 2*max_accel_distance < self.dist:
            sequence.append((MovementStage.ACCELERATING, 0, max_accel_time))
            constant_speed_time: float = (self.dist - 2*max_accel_distance)/self.max_speed
            sequence.append((MovementStage.CONSTANT_SPEED, max_accel_time, max_accel_time + constant_speed_time))
            sequence.append((MovementStage.DECELERATING, max_accel_time + constant_speed_time, 2*max_accel_time + constant_speed_time))
        else:
            accel_time: float = math.sqrt(self.dist/self.acc)
            sequence.append((MovementStage.ACCELERATING, 0, accel_time))
            sequence.append((MovementStage.DECELERATING, accel_time, 2*accel_time))
        return sequence

    def get_dist_at_time(self, time: float) -> float:
        dist: float = 0
        for i in range(len(self.sequence)): # The current time falls somewhere within this movement
            if time < self.sequence[i][2]:
                if self.sequence[i][0] == MovementStage.CONSTANT_SPEED:
                    dist += self.max_speed*(time - self.sequence[i][1])
                elif self.sequence[i][0] == MovementStage.ACCELERATING:
                    dist += 0.5*self.acc*((time - self.sequence[i][1])**2)
                elif self.sequence[i][0] == MovementStage.DECELERATING:
                    v0: float = self.acc*(self.sequence[i][2] - self.sequence[i][1]) # calculate the initial velocity required for the drone to reach zero speed at the end of this movement
                    dist += v0*(time - self.sequence[i][1]) + 0.5*self.acc*((time - self.sequence[i][1])**2)
                return dist
            elif i == len(self.sequence) - 1: # In this case the sequence of moves is done, so the drone should be at its final position
                return self.dist
            elif self.sequence[i][0] == MovementStage.ACCELERATING or self.sequence[i][0] == MovementStage.DECELERATING:
                dist += 0.5*self.acc*((self.sequence[i][2] - self.sequence[i][1])**2)
            elif self.sequence[i][0] == MovementStage.CONSTANT_SPEED:
                dist += self.max_speed*(self.sequence[i][2] - self.sequence[i][1])
        return 0 # Will never be reached (For type checking)

    def get_speed_at_time(self, time: float) -> float:
        for i in range(len(self.sequence)):
            if time < self.sequence[i][2]:
                if self.sequence[i][0] == MovementStage.CONSTANT_SPEED:
                    return self.max_speed
                elif self.sequence[i][0] == MovementStage.ACCELERATING:
                    return self.acc*(time - self.sequence[i][1])
                elif self.sequence[i][0] == MovementStage.DECELERATING:
                    v0: float = self.acc*(self.sequence[i][2] - self.sequence[i][1]) # calculate the initial velocity required for the drone to reach zero speed at the end of this movement
                    return v0 - self.acc*(time - self.sequence[i][1])
            elif i == len(self.sequence):
                return 0
        return 0 # Will never be reached (For type checking)
    
    def get_duration(self) -> float:
        return self.duration
