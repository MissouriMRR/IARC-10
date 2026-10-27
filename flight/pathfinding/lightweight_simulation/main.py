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
taskManager = TaskManager()
taskManager.add_drone(drone_1)
taskManager.add_drone(drone_2)
taskManager.add_drone(drone_3)
taskManager.add_drone(drone_4)

while len(taskManager.tasks) > 0:
    taskManager.debug_print()
    taskManager.step()
taskManager.debug_print()