from networking_thread import NetworkingThread


import queue
import threading

from _t_message_types import Message, MessageType
from json_config_reader import JsonConfigReader
from networking_interface import NetworkingInterface
import networking_thread
import argparse
import time


def main() -> None:
    # Parse arguments in main thread
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--id", help="Self ID", type=int)
    args = parser.parse_args()

    # Load config
    jsonConfigData = JsonConfigReader()

    # Get drone ID
    if args.id is not None:
        droneId = args.id
        jsonConfigData.set_self_id(droneId)
    else:
        droneId = int(jsonConfigData.get_self_id())
    # parallel
    # Create instance of NetworkingThread class and setup resourcesReadyVariable to pass in
    networkingThreadClassInstance: NetworkingThread = (
        networking_thread.NetworkingThread()
    )
    resourcesReady: queue.Queue[NetworkingInterface] = queue.Queue(maxsize=1)
    # Start networking thread
    networkingThread = threading.Thread(
        target=networkingThreadClassInstance.run_networking_thread,
        args=(resourcesReady, jsonConfigData),
        daemon=True,
    )
    networkingThread.start()

    # Wait for networking to be ready
    networking: NetworkingInterface = (
        resourcesReady.get()
    )  # Used to interface with networking thread
    print("Networking interface ready")

    # Message templates
    heartbeatMessage: Message = Message.create(
        id=MessageType.HEARTBEAT,
        dronesToSendData=(),
        data={
            "senderId": droneId,
            "payload": "Hello server!",
        },
    )

    networking.queue_client_message(heartbeatMessage)

    # Main loop
    msgNum = 0  # Used for testing
    startTime = time.time()
    try:
        while True:
            # Check for server messages
            serverMsg = networking.try_get_server_message(timeout=0.02)
            if serverMsg is not None:
                msgNum += 1
                # print(f"Server Data: {serverMsg}")
                print(f"Client Data: {msgNum}")

            # Check for client responses
            clientMsg = networking.try_get_client_response(timeout=0.02)
            if clientMsg is not None:
                # print(f"Client Data: {msgNum}")
                pass
            # Send heartbeat if queue is empty
            if networking.is_client_in_empty():
                heartbeatMessage.data["payload"] = str(
                    msgNum  # NOTE could be spot for error. Not sure if value can be edited. Also verify that copy of message is being uploaded
                )
                networking.queue_client_message(heartbeatMessage)

            time.sleep(0.1)

    except KeyboardInterrupt:
        print(f"Program ran for {time.time() - startTime}")
        print("Shutting down...")


if __name__ == "__main__":
    main()
