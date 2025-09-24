import socket
import time


# TODO implement list of drones then make it loop through to send data to them (or 3 threads :) )


class Client:
    # Client Class constructor. Used to pass in JSON Data
    def __init__(self, jsonData):
        # TODO Update to create a list of drones to ping
        self.jsonData = jsonData
        self.droneId: str = jsonData["localInfo"]["selfId"]

        # Parse JSON to get IPs and Ports of drones to connect to

        # Instantiate otherDrones lists
        self.otherDronesIps: list[str]
        self.otherDronesPorts: list[int]

        # Loop through drones 1-4
        for i in range(1, 5):
            # If drone is self (drone running this script) don't add them otherDrones list
            if i != int(self.droneId):
                # Add other drones IP and Ports to their respective lists
                self.otherDronesIps.append(
                    self.jsonData["drones"][str(i)]["ip"]
                )  # This will be simplified after JSON parser is implemented
                self.otherDronesPorts.append(
                    int(self.jsonData["drones"][str(i)]["port"])
                )
        print(self.otherDronesIps)
        print(self.otherDronesPorts)

    # Start client and try to send messages to servers
    # TODO. Once drone list is implemented, split into setup and runtime functions
    def start_client(self):
        while True:
            try:
                # TODO update to send messages to clients based on drone list
                self.send_data(
                    serverIP=str(self.jsonData["drones"]["drone2"]["ip"]),
                    serverPort=int(self.jsonData["drones"]["drone2"]["port"]),
                )
                time.sleep(5)
            except:
                print("no connection secured")
                time.sleep(5)

    # Send data to server based on IP and Port
    def send_data(self, serverIP: str, serverPort: int):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            client_socket.connect((serverIP, serverPort))
            client_socket.sendall(b"Hello, server!")
            data = client_socket.recv(1024)
            print(f"Received from server: {data.decode()}")
