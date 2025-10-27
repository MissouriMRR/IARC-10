import json

class json_class:
   
    def __init__(self):
        with open("config.json", "r") as file:
            self.config = json.load(file)
    
    # get drone ip address
    def get_drone_ip(self, droneID : int):
        return self.config["drones"][droneID]

    # get drone port
    def get_drone_port(self, dronePort : int):
        return self.config["drones"][dronePort]
    