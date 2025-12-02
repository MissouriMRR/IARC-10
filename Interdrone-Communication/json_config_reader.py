import json
from typed_dicts_classes import JsonConfigData


class json_config_reader:
    def __init__(self) -> None:
        self.config: JsonConfigData
        with open("config.json", "r") as file:
            self.config = json.load(file)

    # get drone ip address
    def get_drone_ip(self, droneId: int) -> str:
        return str(self.config["drones"][str(droneId)]["ip"])

    # get drone port
    def get_drone_port(self, droneId: int) -> int:
        return int(self.config["drones"][str(droneId)]["port"])

    def get_app_ip(self) -> str:
        return str(self.config["app"]["ip"])

    def set_app_ip(self, newIP: int):
        # TODO test this with app
        self.config["app"]["ip"] = newIP

    def get_app_port(self) -> int:
        return int(self.config["app"]["port"])

    def set_app_port(self, newPort: int):
        # TODO test this with app
        self.config["app"]["port"] = newPort

    # get self id
    def get_self_id(self) -> int:
        return int(self.config["localInfo"]["selfId"])

    # set self id
    def set_self_id(self, newId: int):
        self.config["localInfo"]["selfId"] = newId

    # Get speed test data size for network test
    def get_speed_test_data_size(self) -> int:
        return int(self.config["localInfo"]["speedTestKbDataSize"])

    # TODO make this good after sprint
    # Gets current config file and edits selfId to be used for startup script
    def get_json_text_data_for_startup(self, droneId: int):
        ogId = self.get_self_id()
        self.config["localInfo"]["selfId"] = droneId
        jsonExport = json.dumps(self.config)
        self.config["localInfo"]["selfId"] = ogId
        return jsonExport

    # Gets number of drones
    def get_number_of_drones(self):
        return len(self.config["drones"])
