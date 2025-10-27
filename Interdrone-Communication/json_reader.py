import json

class json_class:
   
    def __init__(self):
        with open("config.json", "r") as file:
            self.config = json.load(file)
    
    # get drone ip address
    def get_drone_ip(self, num : int):
        drone_id = str(num)
        drone_info = self.config["drones"][drone_id]
        return drone_info["ip"]
    
    # get drone port
    def get_drone_port(self, num : int):
        drone_id = str(num)
        drone_info = self.config["drones"][drone_id]
        return drone_info["port"]