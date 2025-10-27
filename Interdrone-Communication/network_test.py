from asyncio.queues import Queue


import asyncio
from typing import Any
import server
import client
import json
import sys


# DOCS: How to merge this with path finding:
"""
1: Change this to a async start_networking() function that runs as a task from real main function
2. Pass in serverInData, serverOutData 
"""

# TODO List to get this ready to chat with path finding
"""
Move network test out of core functionality to stand alone test (may not be feasible/optimal)
"""


async def main():
    # Get JSON Data
    with open("config.json", "r") as file:
        data: dict[str, Any] = json.load(file)  # TODO verify and update this

    # Create Server and Client Data queues to pass data in and out of tasks
    serverData: Queue[str] = asyncio.Queue()  # May need to change queue type to any

    # TODO talk to Harper to see if we need a serverInData
    clientInData: Queue[dict[str, bool | float | str | Any]] = (
        asyncio.Queue()
    )  # TODO remove Any once we figure out how to structure our passed in data
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

    speedTestMessage: dict[str, bool | float | str | Any] = {
        "messageId": 13,
        "timestamp": 0.0,
        "senderId": droneId,
        "payload": "X"
        * (
            (int(data["localInfo"]["speedTestKbDataSize"])) * 1024
        ),  # Add payload of specified size by creating a string of specified size
    }

    # Run both tasks concurrently
    try:
        # Both tasks are already running in the background
        print("Server and Client started")

        # Continuous loop for other functionality
        while True:
            # Add heartbeat message to clientQueue to send
            await clientInData.put(item=speedTestMessage)

            # Check for serverData from the server task
            if not serverData.empty():
                # print(f"Server Data: {await serverData.get()}")
                pass

            # Check for clientOutData from the client task
            if not clientOutData.empty():
                print("we here fr")
                clientMsg = await clientOutData.get()
                # Process speed test results if applicable
                try:
                    result = json.loads(clientMsg)
                    print(
                        f"Network Test | Target: {result['target']} | RTT: {result['rttMs']}ms | Throughput: {result['throughputKbps']}Kbps"
                    )
                except:
                    print(f"Client Data: {clientMsg}")

            await asyncio.sleep(1)  # Adjust sleep time as needed

    except KeyboardInterrupt:
        print("Shutting down...")
        serverTask.cancel()
        clientTask.cancel()


if __name__ == "__main__":
    asyncio.run(main())
