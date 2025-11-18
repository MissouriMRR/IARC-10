from asyncio.queues import Queue
from typed_dicts_classes import MessageData
from json_config_reader import json_config_reader

import asyncio
import server
import client
import sys


# DOCS: How to merge this with path finding:
"""
1: Change this to a async start_networking() function that runs as a task from real main function
2. Pass in clientInData, clientOutData, and serverOutData 
3. May need to put this code on a separate thread or something and find a way to pass queues in across threads (gonna be a necessary pain :( )
"""


async def main():
    # Create jsonConfigData instance to get data from config file
    jsonConfigData: json_config_reader = json_config_reader()

    # Create Server and Client Data queues to pass data in and out of tasks
    serverOutData: Queue[str] = asyncio.Queue()
    clientInData: Queue[MessageData] = asyncio.Queue()
    clientOutData: Queue[str] = asyncio.Queue()

    # Instantiate Server and Client
    serverInstance = server.Server(
        jsonConfigData=jsonConfigData, serverOutData=serverOutData
    )
    clientInstance = client.Client(
        jsonConfigData=jsonConfigData,
        clientInData=clientInData,
        clientOutData=clientOutData,
    )

    # Run both server and client concurrently
    serverTask = asyncio.create_task(serverInstance.start_server_async())
    clientTask = asyncio.create_task(clientInstance.start_client_async())

    # Get our drones id (the sys arg here allows you pass in a self id from command line for efficient testing)
    droneId: int
    try:
        droneId = int(sys.argv[1])
    except Exception:
        droneId = jsonConfigData.get_self_id()

    # Create heartbeat message
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
                # data= serverOutData.get()
                print(f"Server Data: {await serverOutData.get()}")

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
