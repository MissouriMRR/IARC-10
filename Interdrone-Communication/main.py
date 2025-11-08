from asyncio.queues import Queue
from typing import Any
from message_types import MessageData

import asyncio
import server
import client
import json
import sys

# DOCS: How to merge this with path finding:
"""
1: Change this to a async start_networking() function that runs as a task from real main function
2. Pass in serverInData, serverOutData 
"""


async def main():
    # Get JSON Data
    with open("config.json", "r") as file:
        data: dict[str, Any] = json.load(file)  # TODO verify and update this

    # Create Server and Client Data queues to pass data in and out of tasks
    serverData: Queue[str] = asyncio.Queue()  # May need to change queue type

    clientInData: Queue[MessageData] = asyncio.Queue()
    clientOutData: Queue[str] = asyncio.Queue()

    # Instantiate Server and Client
    serverInstance = server.Server(jsonData=data, serverOutData=serverData)
    clientInstance = client.Client(
        jsonData=data, clientInData=clientInData, clientOutData=clientOutData
    )

    # Run both server and client concurrently
    serverTask = asyncio.create_task(serverInstance.start_server_async())
    clientTask = asyncio.create_task(clientInstance.start_client_async())

    droneId: str
    try:
        droneId = sys.argv[1]
    except Exception:
        droneId = data["localInfo"]["selfId"]

    heartBeatMessage: MessageData = {
        "messageId": 504,
        "data": {
            "timestamp": 0.0,
            "senderId": droneId,
            "payload": "Hello server!",
        },
    }

    # Run both tasks concurrently
    try:
        # Both tasks are already running in the background
        print("Server and Client started")

        # Continuous loop for other functionality
        while True:
            # Add heartbeat message to clientQueue to send
            await clientInData.put(item=heartBeatMessage)

            # Check for serverData from the server task
            if not serverData.empty():
                # print(f"Server Data: {await serverData.get()}")
                pass

            # Check for clientOutData from the client task
            if not clientOutData.empty():
                clientMsg = await clientOutData.get()
                # Process speed test results if applicable
                try:
                    # result = json.loads(clientMsg)
                    print(f"Client Data: {clientMsg}")
                except:
                    print(f"Client Data: {clientMsg}")

            await asyncio.sleep(1)  # Adjust sleep time as needed

    except KeyboardInterrupt:
        print("Shutting down...")
        serverTask.cancel()
        clientTask.cancel()


if __name__ == "__main__":
    asyncio.run(main())
