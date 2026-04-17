# Outside Imports
import queue
import threading
import argparse
import time

# Interdrone Imports
from interdrone_communication.message_types import Message, MessageType
from interdrone_communication.network_config import NetworkConfig
from interdrone_communication.networking_interface import NetworkingInterface
from interdrone_communication.networking_thread import NetworkingThread


# When running inner files from top level do
# uv run -m interdrone_communication.main -i 1 (runs as a module so it has top level access)
def main() -> None:
    # Parse arguments in main thread
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--id", help="Self ID", type=int)
    args = parser.parse_args()

    # Load config
    networkConfig = NetworkConfig()

    # Get drone ID
    if args.id is not None:
        droneId = args.id
        networkConfig.set_self_id(droneId)
    else:
        droneId = int(networkConfig.get_self_id())
    # parallel
    # Create instance of NetworkingThread class and setup resourcesReadyVariable to pass in
    networkingThreadClassInstance: NetworkingThread = NetworkingThread()
    resourcesReady: queue.Queue[NetworkingInterface] = queue.Queue(maxsize=1)
    # Start networking thread
    networkingThread = threading.Thread(
        target=networkingThreadClassInstance.run_networking_thread,
        args=(resourcesReady, networkConfig),
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
                heartbeatMessage.data["payload"] = str(msgNum)
                networking.queue_client_message(heartbeatMessage)

            time.sleep(0.1)

    except KeyboardInterrupt:
        print(f"Program ran for {time.time() - startTime}")
        print("Shutting down...")


if __name__ == "__main__":
    main()
