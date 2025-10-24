from asyncio.queues import Queue


import asyncio
import server
import client
import json


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
        data: dict[str, object] = json.load(file)

    # Create Server and Client Data queues to pass data out of tasks
    serverData: Queue[str] = asyncio.Queue()  # May need to change queue type to any
    clientData: Queue[str] = asyncio.Queue()

    # Instantiate Server and Client
    serverInstance = server.Server(jsonData=data, serverOutData=serverData)
    clientInstance = client.Client(jsonData=data, clientOutData=clientData)

    # Run both server and client concurrently
    serverTask = asyncio.create_task(serverInstance.start_server_async())
    clientTask = asyncio.create_task(clientInstance.start_client())

    # Run both tasks concurrently
    try:
        # Both tasks are already running in the background
        print("Server and Client started")

        # Continuous loop for other functionality
        while True:
            # Check for serverData from the server task
            if not serverData.empty():
                # print(f"Server Data: {await serverData.get()}")
                pass

            # Check for clientData from the client task
            if not clientData.empty():
                clientMsg = await clientData.get()
                # Process speed test results if applicable
                try:
                    result = json.loads(clientMsg)
                    if isinstance(result, dict) and "rtt_ms" in result:
                        print(
                            f"Network Test | Target: {result['target']} | RTT: {result['rttMs']}ms | Throughput: {result['throughputKbps']}Kbps"
                        )
                    else:
                        print(f"Client Data: {clientMsg}")
                except:
                    print(f"Client Data: {clientMsg}")

            await asyncio.sleep(0.1)  # Adjust sleep time as needed

    except KeyboardInterrupt:
        print("Shutting down...")
        serverTask.cancel()
        clientTask.cancel()


if __name__ == "__main__":
    asyncio.run(main())
