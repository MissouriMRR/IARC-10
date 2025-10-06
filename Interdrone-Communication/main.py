from asyncio.queues import Queue


import asyncio
import server
import client
import json


async def main():
    # Get JSON Data
    with open("config.json", "r") as file:
        data: dict[str, object] = json.load(file)

    # Create Server and Client Data queues to pass data out of tasks
    serverData: Queue[str] = asyncio.Queue()  # May need to change queue type to any
    clientData: Queue[str] = asyncio.Queue()

    # Instantiate Server and Client
    serverInstance = server.Server(jsonData=data, serverOutData=serverData)
    clientInstance = client.Client(jsonData=data, clientOutData=clientData)

    # Run both server and client concurrently
    server_task = asyncio.create_task(serverInstance.start_server_async())
    client_task = asyncio.create_task(clientInstance.start_client())

    # Run both tasks concurrently
    try:
        # Both tasks are already running in the background
        print("Server and Client started")

        # Continuous loop for other functionality
        while True:
            # Check for serverData from the server task
            if not serverData.empty():
                print(f"Server Data: {await serverData.get()}")

                # update ts to have a localServerData var that gets, then calls an async function to process

            # Check for clientData from the client task
            if not clientData.empty():
                print(f"Client Data: {await clientData.get()}")

            await asyncio.sleep(0.1)  # Adjust sleep time as needed

    except KeyboardInterrupt:
        print("Shutting down...")
        server_task.cancel()
        client_task.cancel()


if __name__ == "__main__":
    asyncio.run(main())
