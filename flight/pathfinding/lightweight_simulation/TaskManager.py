from Task import Task, TaskWrapper
from Drone import Drone


"""
Class for keeping track of Tasks and running them one step at a time
"""
class TaskManager:
    def __init__(self) -> None:
        self.drones = []
        self.tasks: dict[Drone, TaskWrapper] = {}

    def add_drone(self, drone: Drone) -> None:
        self.drones.append(drone)
        gen = drone.task_generator()
        try:
            self.tasks[drone] = TaskWrapper(next(gen), gen)
        except StopIteration:
            self.tasks.pop(drone)

    """
    Adds a task and its generator to the task list
    """
    def next_task(self, drone: Drone) -> None:
        gen = self.tasks[drone].get_gen()
        try:
            self.tasks[drone] = TaskWrapper(next(gen), gen)
        except StopIteration:
            print("hello")
            self.tasks.pop(drone)


    def get_tasks(self) -> list[TaskWrapper]:
        tasks = []
        for value in self.tasks.values():
            tasks.append(value)
        return tasks

    """
    Runs all tasks in the task list for one step (The smallest remaining task time)
    """
    def step(self) -> None:
        min_task_step: float = self.get_min_task_step()
        if (min_task_step < float("inf")):
            for task in self.tasks.values():
                task.run_for_time(min_task_step)
        next_task_keys = []
        for drone, task in self.tasks.items():
            if task.check_finished():
                next_task_keys.append(drone)
        for drone in next_task_keys:
                self.next_task(drone)

        
        

    """
    Returns the smallest remaining time of the tasks in the self.task list
    """
    def get_min_task_step(self) -> float:
        if len(self.tasks) == 0:
            return 0
        min_time: float = float('inf')
        for task in self.tasks.values():
            if task.get_task().get_remaining_time() < min_time:
                min_time = task.get_task().get_remaining_time()
        return min_time

    def debug_print(self):
        for task in self.tasks.values():
            print(task.get_task().get_remaining_time(), end = " ")
        print()

