import asyncio
import server
import client
import json


async def main():
    # Get JSON Data
    with open("config.json", "r") as file:
        data: dict[str, object] = json.load(file)

    # Instantiate Server and Client
    serverInstance = server.Server(jsonData=data)
    clientInstance = client.Client(jsonData=data)

    # Run both server and client concurrently
    server_task = asyncio.create_task(serverInstance.start_server_async())
    client_task = asyncio.create_task(clientInstance.start_client())

    print("Server and Client started")

    # Implement other code here

    try:
        # Run both tasks concurrently
        await asyncio.gather(server_task, client_task)
    except KeyboardInterrupt:
        print("Shutting down...")
        server_task.cancel()
        client_task.cancel()


if __name__ == "__main__":
    asyncio.run(main())
