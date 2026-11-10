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
            self.tasks[drone] = TaskWrapper(gen)
        except StopIteration:
            self.tasks.pop(drone)

    """
    Removes drones that have finished
    """
    def prune_drones(self) -> None:
        for drone in range(len(self.drones) - 1, -1, -1):
            if self.tasks[self.drones[drone]].check_gen_finished():
                self.tasks.pop(self.drones[drone])
                self.drones.pop(drone)


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
        self.prune_drones()
        
        

    """
    Returns the smallest remaining time of the tasks in the self.task list
    """
    def get_min_task_step(self) -> float:
        if len(self.tasks) == 0:
            return 0
        min_time: float = float('inf')
        for task in self.tasks.values():
            curr_task = task.get_task()
            if curr_task is None:
                task.task_init()
                curr_task = task.get_task()
            if curr_task is not None and curr_task.get_remaining_time() < min_time:
                min_time = curr_task.get_remaining_time()
        return min_time

    def debug_print(self):
        for task in self.tasks.values():
            curr_task = task.get_task()
            if curr_task is None:
                print(None, end = " ")
            else:
                print(curr_task.get_remaining_time(), end = " ")
        print()

