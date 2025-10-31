import json

class json_class:
   
    def __init__(self):
        with open("config.json", "r") as file:
            self.config = json.load(file)
    
    # get drone ip address
    def get_drone_ip(self, droneId : int) -> str:
        return self.config["drones"][droneId]["ip"]

    # get drone port
    def get_drone_port(self, droneId : int) -> int:
        return self.config["drones"][droneId]["port"]
    
    #get own ip
    def get_self_id(self)-> str:
        return self.config["localInfo"]["selfId"]
    
    # TODO find here in code above is used, like selfID or
    # drone port