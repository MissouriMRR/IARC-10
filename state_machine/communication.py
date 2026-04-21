import json
import logging
from state_machine import drone_state


def Ping_Drones(drones):
    """
    Placeholder for logic to ping the drone and update the ping_response attribute.
    """
    pass


def send_ARM(self, drone_id):
    """
    Placeholder for logic to send an ARM command to the drone.
    """
    pass


def Send_Arm_Ack(self, drone_id):
    """
    Placeholder for logic to send an acknowledgment back to the drone that it has been armed.
    """
    pass


def All_Armed(self, drones):
    """
    Placeholder for logic to check if all drones in the list are armed.
    """
    pass


def All_Takeoff(self, drones):
    """
    Placeholder for logic to check if all drones in the list have taken off.
    """
    pass


def All_Mission_Start(self, drones):
    """
    Placeholder for logic to check if all drones in the list have started their mission.
    """
    pass


def All_Demo_Start(self, drones):
    """
    Placeholder for logic to check if all drones in the list have started their demo mode.
    """
    pass


def Send_Takeoff(self, drone_id):
    """
    Placeholder for logic to send a TAKEOFF command to the drone.
    """
    pass


def Send_Takeoff_Ack(self, drone_id):
    """
    Placeholder for logic to send an acknowledgment back to the drone that it has taken off.
    """
    pass


def Send_Start_Demo(self, drone_id):
    """
    Placeholder for logic to send a command to start the demo mode to the drone.
    """
    pass


def Send_Demo_Ack(self, drone_id):
    """
    Placeholder for logic to send an acknowledgment back to the drone that it has started demo mode.
    """
    pass


def Send_Start_Mission(self, drone_id):
    """
    Placeholder for logic to send a command to start the mission to the drone.
    """
    pass


def Send_Mission_Ack(self, drone_id):
    """
    Placeholder for logic to send an acknowledgment back to the drone that it has started its mission.
    """
    pass
