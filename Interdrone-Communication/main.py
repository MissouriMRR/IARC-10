import threading
import server
import client

import json

# Get JSON Data
# TODO implement JSON class for easier data access
with open("config.json", "r") as file:
    data: dict[str, object] = json.load(file)


# Instantiate Server and Client
serverInstance = server.Server(jsonData=data)
clientInstance = client.Client(jsonData=data)

# Create threads properly (don't call the functions with parentheses)
serverThread = threading.Thread(target=serverInstance.start_server, name="ServerThread")
clientThread = threading.Thread(target=clientInstance.start_client, name="ClientThread")

# Start the threads
serverThread.start()
print("Server started")
clientThread.start()
print("Client started")

# Wait for threads to complete (if needed)
serverThread.join()
clientThread.join()
