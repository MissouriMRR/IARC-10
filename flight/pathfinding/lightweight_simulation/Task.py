from collections.abc import Iterator
from typing import Any, Generator

"""
Class representing a generic task, For actual tasks a child class should be created that sets the runtime of the task and contains any logic/return values of the task
"""
class Task:
    def __init__(self) -> None:
        self.remaining_time: float = 0
        self.finished: bool = False

    """
    Runs the task for the provided amount of time and returns a tuple containing a boolean indicating whether the task is finished and a return value of the task if applicable or None if the task is not finished or does not contain a return value
    """
    def run_for_time(self, time: float) -> tuple[bool, Any | None]:
        self.remaining_time -= time
        if self.remaining_time <= 0:
            self.finished = True
            return (True, None)
        return (False, None)


    """
    Returns remaining time of execution for task
    """
    def get_remaining_time(self) -> float:
        return self.remaining_time

"""
Class representing a wrapper for containing a task and its generator
"""
class TaskWrapper:
    def __init__(self, task: Task, gen: Generator[Task, Any, Any]) -> None:
        self.task: Task = task
        self.gen: Generator[Task, Any, Any] = gen
        self.finished: bool = False

    """
    Calls tasks run for time function, and if the task finishes, passes the return value to the generator
    """
    def run_for_time(self, time) -> None:
        rv: tuple[bool, Any | None] = self.task.run_for_time(time)
        if rv[0] == True:
            self.finished = True
            self.gen.send(rv[1])

    def get_task(self) -> Task:
        return self.task

    def check_finished(self) -> bool:
        return self.finished
