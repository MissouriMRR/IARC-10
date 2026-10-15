from Task import Task, TaskWrapper
from Drone import Drone
from collections.abc import Iterator


"""
Class for keeping track of Tasks and running them one step at a time
"""
class TaskManager:
    def __init__(self) -> None:
        self.tasks: list[TaskWrapper] = []

    """
    Adds a task and its generator to the task list
    """
    def addTask(self, task: TaskWrapper) -> None:
        self.tasks.append(task)

    """
    Runs all tasks in the task list for one step (The smallest remaining task time)
    """
    def step(self) -> None:
        min_task_step: float = self.get_min_task_step()
        if (min_task_step < float("inf")):
            for task in self.tasks:
                task.run_for_time(min_task_step)

    """
    Returns the smallest remaining time of the tasks in the self.task list
    """
    def get_min_task_step(self) -> float:
        if len(self.tasks == 0):
            return 0
        min_time: float = self.tasks[0][0].get_remaining_time()
        for task in self.tasks[1:]:
            if task[0].get_remaining_time() < min_time:
                min_time = task[0].get_remaining_time()
        return min_time

    def debug_print(self):
        for task in self.tasks:
            print(task.task.get_remaining_time(), end = " ")
        print()

