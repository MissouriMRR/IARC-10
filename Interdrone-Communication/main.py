from asyncio.events import AbstractEventLoop
from asyncio.queues import Queue as AsyncQueue
import queue
import threading
import asyncio

from typed_dicts_classes import MessageData
from json_config_reader import json_config_reader
from networking_interface import NetworkingInterface
import server
import client
import argparse
import time


# Run the async networking stack on its own thread
def run_networking_thread(
    resourcesReady: queue.Queue[NetworkingInterface],
    jsonConfigData: json_config_reader,
) -> None:
    loop: AbstractEventLoop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    clientIn: AsyncQueue[MessageData] = asyncio.Queue()
    clientOut: AsyncQueue[str] = asyncio.Queue()
    serverOut: AsyncQueue[str] = asyncio.Queue()

    # Provide interface to main thread
    resourcesReady.put(
        NetworkingInterface(
            loop=loop,
            clientIn=clientIn,
            clientOut=clientOut,
            serverOut=serverOut,
        )
    )

    loop.run_until_complete(
        start_networking(
            clientInData=clientIn,
            clientOutData=clientOut,
            serverOutData=serverOut,
            jsonConfigData=jsonConfigData,
        )
    )


# Async networking entry point - runs server and client
async def start_networking(
    clientInData: AsyncQueue[MessageData],
    clientOutData: AsyncQueue[str],
    serverOutData: AsyncQueue[str],
    jsonConfigData: json_config_reader,
) -> None:
    # Instantiate Server and Client
    serverInstance = server.Server(
        jsonConfigData=jsonConfigData, serverOutData=serverOutData
    )
    clientInstance = client.Client(
        jsonConfigData=jsonConfigData,
        clientInData=clientInData,
        clientOutData=clientOutData,
    )

    # Run both concurrently
    serverTask = asyncio.create_task(serverInstance.start_server_async())
    clientTask = asyncio.create_task(clientInstance.start_client_async())

    print("Server and Client started")

    try:
        # Keep the networking loop alive
        while True:
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        print("Networking shutting down...")
        serverTask.cancel()
        clientTask.cancel()


def main() -> None:
    # Parse arguments in main thread
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--id", help="Self ID", type=int)
    # parser.add_argument("-s", "--skip", help="Startup override (1=true)", type=int)
    args = parser.parse_args()

    # Load config
    jsonConfigData = json_config_reader()

    # Get drone ID
    if args.id is not None:
        droneId = args.id
        jsonConfigData.set_self_id(droneId)
    else:
        droneId = int(jsonConfigData.get_self_id())

    # Start networking thread
    resourcesReady: queue.Queue[NetworkingInterface] = queue.Queue(maxsize=1)
    networkingThread = threading.Thread(
        target=run_networking_thread,
        args=(resourcesReady, jsonConfigData),
        daemon=True,
    )
    networkingThread.start()

    # Wait for networking to be ready
    networking: NetworkingInterface = resourcesReady.get()
    print("Networking interface ready")

    # Message templates
    heartbeatMessage: MessageData = {
        "messageId": 504,
        "dronesToSendData": [],
        "data": {
            "timestamp": 0.0,
            "senderId": droneId,
            "payload": "Hello server!",
        },
    }

    networking.queue_client_message(heartbeatMessage)

    # Main loop
    msgNum = 0  # Used for testing
    try:
        while True:
            # Check for server messages
            serverMsg = networking.try_get_server_message(timeout=0.02)
            if serverMsg is not None:
                print(f"Server Data: {serverMsg}")

            # Check for client responses
            clientMsg = networking.try_get_client_response(timeout=0.02)
            if clientMsg is not None:
                msgNum += 1
                print(f"Client Data: {msgNum}")

            # Send heartbeat if queue is empty
            if networking.is_client_in_empty():
                networking.queue_client_message(heartbeatMessage)

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Shutting down...")


if __name__ == "__main__":
    main()
