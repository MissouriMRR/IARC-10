from asyncio.queues import Queue
from typing import Any
from message_types import MessageData

import asyncio
import server
import client
import json
import sys
import json_reader

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
    serverOutData: Queue[str] = asyncio.Queue()

    clientInData: Queue[MessageData] = asyncio.Queue()
    clientOutData: Queue[str] = asyncio.Queue()

    # Instantiate Server and Client
    jsonReaderInstance = json_reader()
    serverInstance = server.Server(jsonData=jsonReaderInstance, serverOutData=serverData)
    clientInstance = client.Client(
        jsonData=jsonReaderInstance, clientInData=clientInData, clientOutData=clientOutData
    )

    # Run both server and client concurrently
    serverTask = asyncio.create_task(serverInstance.start_server_async())
    clientTask = asyncio.create_task(clientInstance.start_client_async())

    droneId: str
    try:
        droneId = sys.argv[1]
    except Exception:
        droneId = jsonReaderInstance.get_self_id()

    heartBeatMessage: MessageData = {
        "messageId": 504,
        "data": {
            "timestamp": 0.0,
            "senderId": droneId,
            "payload": "Hello server!",
        },
    }

    try:
        print("Server and Client started")

        await clientInData.put(item=heartBeatMessage)

        # Continuous loop to send and receive data from server and client
        while True:
            # Add heartbeat message to clientQueue to send

            # Check for serverOutData from the server task
            if not serverOutData.empty():
                # print(f"Server Data: {await serverOutData.get()}")
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

            # If previous heartbeat message has been sent, add new one to queue to be sent
            if clientInData.empty():
                await clientInData.put(item=heartBeatMessage)

            await asyncio.sleep(1)  # Adjust sleep time as needed

    except KeyboardInterrupt:
        print("Shutting down...")
        serverTask.cancel()
        clientTask.cancel()


if __name__ == "__main__":
    asyncio.run(main())
