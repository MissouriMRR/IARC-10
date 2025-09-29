from asyncio.queues import Queue


import asyncio
import server
import client
import json


async def main():
    # Get JSON Data
    with open("config.json", "r") as file:
        data: dict[str, object] = json.load(file)

    # Instantiate Server and Client
    serverData: Queue[str] = asyncio.Queue()
    serverInstance = server.Server(jsonData=data, serverOutData=serverData)
    clientInstance = client.Client(jsonData=data)

    # Run both server and client concurrently
    server_task = asyncio.create_task(serverInstance.start_server_async())
    client_task = asyncio.create_task(clientInstance.start_client())

    try:
        # Run both tasks concurrently
        try:
            # Both tasks are already running in the background
            print("Server and Client started")

            # Continuous loop for other functionality
            while True:
                # Your other functionality here
                if not serverData.empty():
                    print(await serverData.get())

                await asyncio.sleep(0.1)  # Adjust sleep time as needed

        except KeyboardInterrupt:
            print("Shutting down...")
            server_task.cancel()
            client_task.cancel()

    except KeyboardInterrupt:
        print("Shutting down...")
        server_task.cancel()
        client_task.cancel()


if __name__ == "__main__":
    asyncio.run(main())
