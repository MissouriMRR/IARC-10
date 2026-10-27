from Drone import Drone
from Task import *
from TaskManager import TaskManager
# from DroneController import DroneController
"""
Velocity in units of ft/s
Acceleration in units of ft/s^2
Time in seconds
"""

drone_1 = Drone(10,0,(0,0),0)
drone_2 = Drone(10,0,(5,0),0)
drone_3 = Drone(10,0,(0,5),0)
drone_4 = Drone(10,0,(5,5),0)
drones = [drone_1,drone_2,drone_3,drone_4]
taskManager = TaskManager()

for drone in drones:
    taskManager.addTask(drone.task_generator())

for i in range(len(taskManager.tasks)):
    taskManager.step()
    taskManager.debug_print()