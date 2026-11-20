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
    def run_for_time(self, time: float) -> Any | None:
        self.remaining_time -= time
        if self.remaining_time <= 0:
            self.finished = True
            return None
        return None

    def get_finished(self) -> bool:
        return self.finished

    def __str__(self) -> str:
        return "Base task class"


    """
    Returns remaining time of execution for task
    """
    def get_remaining_time(self) -> float:
        return self.remaining_time

    def debug_task_finished(self) -> str:
        return "Task finished"

"""
Class representing a wrapper for containing a task and its generator
"""
class TaskWrapper:
    def __init__(self, gen: Generator[Task, Any, Any]) -> None:
        self.task: Task | None = None
        self.gen: Generator[Task, Any, Any] = gen
        self.finished: bool = False
        self.gen_finished: bool = False

    """
    Calls tasks run for time function, and if the task finishes, passes the return value to the generator and gets the next task
    """
    def run_for_time(self, time: float) -> None:
        if self.task is None:
            self.task = next(self.gen)
            self.debug_task_start()
        rv: Any | None = self.task.run_for_time(time)
        if self.task.get_finished():
            self.finished = True
            self.debug_task_finished()
            try:
                self.task = self.gen.send(rv)
                self.debug_task_start()
                self.finished = False
            except StopIteration:
                self.gen_finished = True

    def task_init(self) -> None:
        if self.task is None:
            self.task = next(self.gen)
            self.debug_task_start()

    """
    Return current task for this task wrapper
    """
    def get_task(self) -> Task | None:
        return self.task

    """
    Return the generator for this task wrapper
    """
    def get_gen(self) -> Generator[Task, Any, Any]:
        return self.gen

    """
    Check if generator has used up all tasks
    """
    def check_gen_finished(self) -> bool:
        return self.gen_finished

    def debug_task_start(self) -> None:
        if self.task is not None:
            print(self.task)
        else:
            print("No task")

    def debug_task_finished(self) -> None:
        curr_task = self.task
        if curr_task is None:
            print("None")
        else:
            print(curr_task.debug_task_finished())
