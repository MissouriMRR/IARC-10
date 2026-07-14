# Outside Imports
import queue
import threading
import argparse
import time

# Interdrone Imports
from interdrone_communication.message_types import Message, MessageType
from interdrone_communication.networking_interface import NetworkingInterface
from interdrone_communication.networking_thread import NetworkingThread
from state_machine.flight_settings import FlightSettings


# When running inner files from top level do
# uv run -m interdrone_communication.main -i 1 (runs as a module so it has top level access)
def main() -> None:
    # Parse arguments in main thread
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--id", help="Self ID", type=int)
    args = parser.parse_args()

    # Load config with full drone_info populated from mission_config.json
    flight_settings = FlightSettings.from_mission_config(self_id=args.id)
    drone_id = flight_settings.current_drone_ID
    # parallel
    # Create instance of NetworkingThread class and setup resources_ready variable to pass in
    networking_thread_obj: NetworkingThread = NetworkingThread()
    resources_ready: queue.Queue[NetworkingInterface] = queue.Queue(maxsize=1)
    # Start networking thread
    networking_thread = threading.Thread(
        target=networking_thread_obj.run_networking_thread,
        args=(resources_ready, flight_settings),
        daemon=True,
    )
    networking_thread.start()

    # Wait for networking to be ready
    networking: NetworkingInterface = (
        resources_ready.get()
    )  # Used to interface with networking thread
    print("Networking interface ready")

    # Message templates
    heartbeat_message: Message = Message.create(
        id=MessageType.HEARTBEAT,
        drones_to_send_data=(),
        sender_id=drone_id,
        data={
            "payload": "Hello server!",
        },
    )

    networking.queue_client_message(heartbeat_message)

    # Main loop
    msg_num = 0  # Used for testing
    start_time = time.time()
    try:
        while True:
            # Check for server messages
            server_msg = networking.try_get_server_message(timeout=0.02)
            if server_msg is not None:
                msg_num += 1
                # print(f"Server Data: {server_msg}")
                print(f"Client Data: {msg_num}")

            # Send heartbeat if queue is empty
            if networking.is_client_in_empty():
                heartbeat_message.data["payload"] = str(msg_num)
                networking.queue_client_message(heartbeat_message)

            time.sleep(0.1)

    except KeyboardInterrupt:
        print(f"Program ran for {time.time() - start_time}")
        print("Shutting down...")


if __name__ == "__main__":
    main()
