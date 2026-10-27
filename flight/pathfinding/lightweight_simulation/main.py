from Drone import Drone
from Task import *
from TaskManager import TaskManager
# from DroneController import DroneController
"""
Velocity in units of ft/s
Acceleration in units of ft/s^2
Time in seconds
"""

drone_1 = Drone(10, 1,(0,0),0)
drone_2 = Drone(10, 2,(5,0),0)
drone_3 = Drone(10, 3,(0,5),0)
drone_4 = Drone(10, 4,(5,5),0)
drones = [drone_1,drone_2,drone_3,drone_4]
drone_generators = [drone_1.task_generator(), drone_2.task_generator(), drone_3.task_generator(), drone_4.task_generator()]
taskManager = TaskManager()

for drone_gen in drone_generators:
    try:
        taskManager.addTask(TaskWrapper(next(drone_gen), drone_gen))
    except StopIteration:
        pass

while len(taskManager.tasks) > 0:
    taskManager.debug_print()
    taskManager.step()
taskManager.debug_print()