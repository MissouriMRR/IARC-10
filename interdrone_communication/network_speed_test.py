# Outside Imports
from asyncio import Task
import asyncio
import argparse
import queue
import threading
import traceback
import os

# Interdrone Imports
from interdrone_communication.message_types import Message, MessageType
from interdrone_communication.networking_thread import NetworkingThread
from interdrone_communication.networking_interface import NetworkingInterface
from state_machine.flight_settings import FlightSettings

SPEED_TEST_PAYLOAD_KB: int = 16


# Plan for network test updates: Filter logs based on self id and target
# uv run -m interdrone_communication.network_speed_test -i 1
async def main():
    # Parse arguments in main thread
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--id", help="Self ID", type=int)
    args = parser.parse_args()

    # Load config
    flight_settings = FlightSettings.from_mission_config(self_id=args.id)
    drone_id = flight_settings.current_drone_ID

    # Start networking thread
    networking_thread_obj: NetworkingThread = NetworkingThread()
    resources_ready: queue.Queue[NetworkingInterface] = queue.Queue(maxsize=1)
    networking_thread = threading.Thread(
        target=networking_thread_obj.run_networking_thread,
        args=(resources_ready, flight_settings),
        daemon=True,
    )
    networking_thread.start()
    background_tasks: set[Task[None]] = set[Task[None]]()  # Used for logging thread

    # Wait for networking to be ready
    networking: NetworkingInterface = resources_ready.get()
    print("Networking interface ready")

    speed_test_message: Message = Message.create(
        id=MessageType.SPEED_TEST_REQUEST,
        drones_to_send_data=(),  # Modify this for selective speed test
        sender_id=drone_id,
        data={
            "initial_upload_time": 0.0,  # Set when queued to send
            "payload_size": SPEED_TEST_PAYLOAD_KB * 1024,
            "payload": "X"
            * (
                SPEED_TEST_PAYLOAD_KB * 1024
            ),  # Multiply string by a specified size of Kb to create a payload size (It's just a very long string of X's to simulate data)
        },
    )
    continuous_speed_test = True
    speed_results: dict[int, list[Message]] = {0: [], 1: [], 2: [], 3: [], 4: []}
    num_tests_per_target: list[int] = [0, 0, 0, 0, 0]
    num_queries_per_test = 100
    networking.queue_client_message(message=speed_test_message)
    while continuous_speed_test:
        try:
            # Check for client responses
            server_msg = networking.try_get_server_message(timeout=0.02)
            if (
                server_msg is not None
                and server_msg.id == MessageType.SPEED_TEST_RESPONSE
            ):
                # Print speed test results
                try:  # Append client Message to dict list
                    target_id: int = server_msg.data["target_id"]
                    # print(target_id)
                    if target_id not in speed_results:
                        speed_results[target_id] = []
                    speed_results[target_id].append(server_msg)
                    if len(speed_results[target_id]) >= num_queries_per_test:
                        # Copy data and clear original list so we can keep receiving immediately
                        results_to_log = list(speed_results[target_id])
                        speed_results[target_id].clear()

                        # Increment test counter
                        current_test_num = num_tests_per_target[target_id]
                        num_tests_per_target[target_id] += 1

                        # Run logging in a separate thread
                        task: Task[None] = asyncio.create_task(
                            asyncio.to_thread(
                                log_data,
                                results_to_log,
                                flight_settings,
                                current_test_num,
                            )
                        )

                        # Add to set to prevent garbage collection
                        background_tasks.add(task)

                        # Remove from set when done
                        task.add_done_callback(background_tasks.discard)

                        print(
                            f"Test {current_test_num} completed from {flight_settings.current_drone_ID} -> {target_id}"
                        )
                    elif len(speed_results[target_id]) % 10 == 0:
                        print(
                            f"Test #{len(speed_results[target_id])} / {num_queries_per_test} complete to target #{target_id}"
                        )
                except Exception:
                    # print(f"Error processing result: {e}")
                    traceback.print_exc()

                    # print(f"Client Data: {server_msg}")
            # If previous message has been sent, add new one to queue
            if networking.is_client_in_empty():
                networking.queue_client_message(message=speed_test_message)

            await asyncio.sleep(
                0.1
            )  # Yield to event loop for background tasks (is good for performance)
        except (KeyboardInterrupt, asyncio.CancelledError):
            print("Shutting down...")
            break


def log_data(
    speed_results: list[Message], flight_settings: FlightSettings, test_number: int
):
    # Print results summary
    # Sanitize directory name (remove >) and create path structure
    folder_name = f"logs/Speed_Test/From_{flight_settings.current_drone_ID}_To_{speed_results[0].data['target_id']}"
    os.makedirs(folder_name, exist_ok=True)

    file_path = f"{folder_name}/test-results-{test_number}.txt"

    with open(file_path, "w") as log_file:

        def log_print(msg):
            print(msg)
            log_file.write(str(msg) + "\n")

        log_print("\n" + "=" * 70)
        log_print(
            f"NETWORK SPEED TEST RESULTS FROM {flight_settings.current_drone_ID} -> {speed_results[0].data['target_id']}"
        )
        log_print("=" * 70)

        if speed_results:
            upload_throughputs = [
                float(r.data["upload_throughput_kbps"]) for r in speed_results
            ]
            upload_rttms = [float(r.data["upload_rtt_ms"]) for r in speed_results]
            download_throughputs = [
                float(r.data["download_throughput_kbps"]) for r in speed_results
            ]
            download_rtt_ms = [float(r.data["download_rtt_ms"]) for r in speed_results]
            for i in range(len(download_rtt_ms)):
                if download_rtt_ms[i] > 1:
                    pass
                    # log_print(
                    #     f"Download rttms was {download_rtt_ms[i]} ms at index {i}"
                    # )
            # Calculate upload statistics
            avg_upload_throughput_kbps = sum(upload_throughputs) / len(upload_throughputs)
            avg_upload_throughput_mbps = avg_upload_throughput_kbps / 1000
            avg_upload_rtt_ms = sum(upload_rttms) / len(upload_rttms)
            min_upload_throughput = min(upload_throughputs) / 1000
            max_upload_throughput = max(upload_throughputs) / 1000
            min_upload_rtt = min(upload_rttms)
            max_upload_rtt = max(upload_rttms)

            # Calculate download statistics
            avg_download_throughput_kbps = sum(download_throughputs) / len(download_throughputs)
            avg_download_throughput_mbps = avg_download_throughput_kbps / 1000
            avg_download_rtt_ms = sum(download_rtt_ms) / len(download_rtt_ms)
            min_download_throughput = min(download_throughputs) / 1000
            max_download_throughput = max(download_throughputs) / 1000
            min_download_rtt = min(download_rtt_ms)
            max_download_rtt = max(download_rtt_ms)

            log_print(f"\nTotal Tests: {len(speed_results)}")
            # log_print(f"Target: {speed_results[0].data['target']}")

            log_print("\n--- UPLOAD STATISTICS ---")
            log_print("Throughput:")
            log_print(
                f"  Average: {avg_upload_throughput_mbps:.2f} Mbps ({avg_upload_throughput_kbps:.2f} Kbps)"
            )
            log_print(f"  Minimum: {min_upload_throughput:.2f} Mbps")
            log_print(f"  Maximum: {max_upload_throughput:.2f} Mbps")
            log_print("Round-Trip Time:")
            log_print(f"  Average: {avg_upload_rtt_ms:.2f} ms")
            log_print(f"  Minimum: {min_upload_rtt:.2f} ms")
            log_print(f"  Maximum: {max_upload_rtt:.2f} ms")

            log_print("\n--- DOWNLOAD STATISTICS ---")
            log_print("Throughput:")
            log_print(
                f"  Average: {avg_download_throughput_mbps:.2f} Mbps ({avg_download_throughput_kbps:.2f} Kbps)"
            )
            log_print(f"  Minimum: {min_download_throughput:.2f} Mbps")
            log_print(f"  Maximum: {max_download_throughput:.2f} Mbps")
            log_print("Round-Trip Time:")
            log_print(f"  Average: {avg_download_rtt_ms:.2f} ms")
            log_print(f"  Minimum: {min_download_rtt:.2f} ms")
            log_print(f"  Maximum: {max_download_rtt:.2f} ms")
        else:
            log_print("\nNo results collected!")

        log_print("=" * 70 + "\n")


if __name__ == "__main__":
    asyncio.run(main())
