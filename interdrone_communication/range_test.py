# Outside Imports
from asyncio import Task
import asyncio
import argparse
import queue
import threading
import traceback
import os
from pathlib import Path
import json
import subprocess

# Interdrone Imports
from interdrone_communication.message_types import Message, MessageType
from interdrone_communication.networking_thread import NetworkingThread
from interdrone_communication.networking_interface import NetworkingInterface
from state_machine.flight_settings import FlightSettings

SPEED_TEST_PAYLOAD_KB: int = 16


"""
General Functionality:
Prompt user for test type
Create test name
Create json file based on that (or append)
Runs one iteration of a network speed test
Logs on json with data from speed test.
Waits for user to prompt for next one

(spreadsheet can be created from JSON in a separate file)
"""


def parse_bool_flag(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "t", "yes", "y", "on"}:
        return True
    if normalized in {"0", "false", "f", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(
        f"Invalid boolean value '{value}'. Use one of: 0/1, true/false, yes/no"
    )


def parse_positive_int(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("Expected an integer") from exc

    if parsed <= 0:
        raise argparse.ArgumentTypeError("Value must be greater than 0")
    return parsed


def parse_batman_location(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "Expected 1 (pi network card) or 2 (wifi adapter)"
        ) from exc

    if parsed not in (1, 2):
        raise argparse.ArgumentTypeError(
            "Expected 1 (pi network card) or 2 (wifi adapter)"
        )
    return parsed


# Sample Run Command:
# uv run range_test.py -i 1 -perf true -t 2 3 4 -b 2
async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--id", help="Self ID", type=int)
    parser.add_argument(
        "-nd",
        "--numDrones",
        help="Number of target drones (not including host)",
        type=parse_positive_int,
    )  # else ask during input section
    parser.add_argument(
        "-perf",
        "--perf",
        help="Include performance in logs (1/0, true/false, yes/no)",
        type=parse_bool_flag,
        default=False,
    )
    parser.add_argument(
        "-uwb",
        "--uwb",
        help="Include range in logs (0/1, true/false, yes/no)",
        type=parse_bool_flag,
        default=False,
    )
    parser.add_argument(
        "-t",
        "--targets",
        help="Optional explicit target drone IDs (space-separated)",
        nargs="+",
        type=int,
    )
    parser.add_argument(
        "-b",
        "--batmanLocation",
        help="Batman location: 1 for pi network card, 2 for wifi adapter",
        type=parse_batman_location,
    )
    parser.add_argument(
        "-cont",
        "--continuousTesting",
        help="If True this disables needing to press the enter key when running a range test",
        type=parse_bool_flag,
        default=False,
    )

    args = parser.parse_args()
    # Load config
    flight_settings = FlightSettings.from_mission_config(self_id=args.id)

    # Declare flag variables
    drone_id: int
    num_drones: int
    drones_to_send_data: list[int]
    get_perf: bool
    uwb_enabled: bool
    continuous_testing: bool

    drone_id = flight_settings.current_drone_ID

    if args.targets is not None:
        drones_to_send_data = args.targets
        if args.numDrones is not None and len(drones_to_send_data) != args.numDrones:
            parser.error(
                "--numDrones must match the number of values passed to --targets"
            )
        num_drones = len(drones_to_send_data)
    elif args.numDrones is not None:
        num_drones = args.numDrones
        drones_to_send_data = []
        for i in range(num_drones):
            drone_id_input = int(
                input(f"Input ID of drone {i + 1} you are talking to in test: ")
            )
            drones_to_send_data.append(drone_id_input)
    else:
        drones_to_send_data = []
        num_drones = 3  # Needed for edge case later on

    get_perf = args.perf
    uwb_enabled = args.uwb
    continuous_testing = args.continuousTesting

    if args.batmanLocation is not None:
        batman_location = args.batmanLocation
    else:
        batman_location = parse_batman_location(
            input(
                "Is batman running on the pi's network card or the wifi adapter (1 or 2): "
            )
        )

    # Setup JSON logging file
    batman_location_str = ""
    if batman_location == 1:
        batman_location_str = "PI Network Card"
    else:
        batman_location_str = "Wifi Adapter"
    log_title = f"Range Test from {drone_id} to {drones_to_send_data} on {batman_location_str}. GetPerf: {get_perf}, UWB: {uwb_enabled} "
    file_name = (
        f"RT_{drone_id}_{str(drones_to_send_data)}_Perf({get_perf})_UWB({uwb_enabled}))"
    )

    # Start networking thread
    networking_thread_obj: NetworkingThread = NetworkingThread()
    resources_ready: queue.Queue[NetworkingInterface] = queue.Queue(maxsize=1)
    networking_thread = threading.Thread(
        target=networking_thread_obj.run_networking_thread,
        args=(resources_ready, flight_settings),
        kwargs={"range_test_toggle": True},
        daemon=True,
    )
    networking_thread.start()
    background_tasks: set[Task[None]] = set[Task[None]]()  # Used for logging thread

    # Wait for networking to be ready
    networking: NetworkingInterface = resources_ready.get()
    print("Networking interface ready")

    speed_test_message: Message = Message.create(
        id=MessageType.SPEED_TEST_REQUEST,
        dronesToSendData=tuple(
            drones_to_send_data
        ),  # Modify this for selective speed test
        senderId=drone_id,
        data={
            "initialUploadTime": 0.0,  # Set when queued to send
            "payloadSize": SPEED_TEST_PAYLOAD_KB * 1024,
            "payload": "X"
            * (
                SPEED_TEST_PAYLOAD_KB * 1024
            ),  # Multiply string by a specified size of Kb to create a payload size (It's just a very long string of X's to simulate data)
        },
    )

    speed_results: dict[int, list[Message]] = {0: [], 1: [], 2: [], 3: [], 4: []}
    num_tests_per_target: list[int] = [0, 0, 0, 0, 0]
    tests_finished_per_target: list[bool] = [False, False, False, False, False]
    num_queries_per_test = 100
    networking.queue_client_message(message=speed_test_message)
    test_finished = False

    print("Starting range test")
    while True:
        try:
            # Check for client responses
            server_msg = networking.try_get_server_message(timeout=0.02)
            if server_msg is not None:
                # Print speed test results
                try:  # Append client Message to dict list
                    target_id: int = server_msg.data["targetId"]
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
                                log_title,
                                file_name,
                                get_perf,
                            )
                        )
                        tests_finished_per_target[target_id] = True

                        # Add to set to prevent garbage collection
                        background_tasks.add(task)

                        # Remove from set when done
                        task.add_done_callback(background_tasks.discard)

                        print(
                            f"Test {current_test_num} completed from {flight_settings.current_drone_ID} -> {target_id}"
                        )
                        # If all drones in range tests tests have finished
                        if sum(tests_finished_per_target) >= num_drones:
                            test_finished = True
                    elif len(speed_results[target_id]) % 10 == 0:
                        print(
                            f"Test #{len(speed_results[target_id])} completed for drone {target_id}"
                        )

                except Exception:
                    # print(f"Error processing result: {e}")
                    traceback.print_exc()

                    # print(f"Client Data: {server_msg}")
            if test_finished:
                # Wait for logging tasks to finish
                while len(background_tasks) != 0:
                    await asyncio.sleep(0.05)
                _ = (
                    networking.empty_queues()
                )  # flush out old data to ensure next test is clean
                if not continuous_testing:
                    print("Individual test finished and data has been logged.")
                    _ = input(
                        "Please press enter when you wish to start another one or press ctrl + c to exit"
                    )
                    print("Next test is running!")
                tests_finished_per_target = [False, False, False, False, False]
                test_finished = False
            # If previous message has been sent, add new one to queue
            if networking.is_client_in_empty():
                networking.queue_client_message(message=speed_test_message)
            await asyncio.sleep(0.1)  # Adjust sleep time as needed
        except (KeyboardInterrupt, asyncio.CancelledError):
            print("Shutting down...")
            break


def log_data(
    speed_results: list[Message],
    flight_settings: FlightSettings,
    test_number: int,
    log_title: str,
    file_name: str,
    get_perf: bool,
):
    # Sanitize directory name (remove >) and create path structure
    folder_name = "Logs/Range_Test"
    os.makedirs(folder_name, exist_ok=True)

    json_path = Path(f"{folder_name}/{file_name}.json")

    if not json_path.exists():
        initial_data = {
            "title": log_title,
            "data": {
                "targetDrone": [],  # Contains ID for drone getting sent data
                "avgUploadThroughputMbps": [],
                "avgUploadRttMs": [],
                "minUploadThroughput": [],
                "maxUploadThroughput": [],
                "minUploadRtt": [],
                "maxUploadRtt": [],
                "avgDownloadThroughputMbps": [],
                "avgDownloadRttMs": [],
                "minDownloadThroughput": [],
                "maxDownloadThroughput": [],
                "minDownloadRtt": [],
                "maxDownloadRtt": [],
                "uwbRange": [],
                "cpuLoad": [],
                "memoryUsage": [],
            },
        }  # Leave unused keys blank and filter out later
        json_path.parent.mkdir(
            parents=True, exist_ok=True
        )  # safe if parent folder already exists
        json_path.write_text(json.dumps(initial_data, indent=2), encoding="utf-8")

    # If json file does not exist, set it up then append

    with json_path.open("r", encoding="utf-8") as f:
        json_data = json.load(f)

    if json_data is None:
        print("Failed to read json data :(")
        return

    if speed_results:
        target_drone = speed_results[0].data["targetId"]
        upload_throughputs = [
            float(r.data["uploadThroughputKbps"]) for r in speed_results
        ]
        upload_rttms = [float(r.data["uploadRttMs"]) for r in speed_results]
        download_throughputs = [
            float(r.data["downloadThroughputKbps"]) for r in speed_results
        ]
        download_rtt_ms = [float(r.data["downloadRttMs"]) for r in speed_results]
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

        # Calculate range from UWB
        uwb_range = 0  # TODO SET THIS UP

        # Get processing specs from PI
        cpu_load = 0
        total_memory = 0
        available_memory = 0
        memory_usage = 0
        if get_perf:
            try:
                result = subprocess.run(
                    ["free", "-m"], capture_output=True, text=True, check=True
                )

                for line in result.stdout.splitlines():
                    if line.startswith("Mem:"):
                        parts = line.split()
                        total_memory = int(parts[1])
                        available_memory = int(parts[6])
                        break

                memory_usage = ((total_memory - available_memory) / total_memory) * 100

                # Get processor usage (takes 2 seconds to run due to delay from top)
                result = subprocess.run(
                    ["top", "-b", "-n", "2"], capture_output=True, text=True, check=True
                )  # Note: top can be inaccurate. If the CPU is under heavy load it can produce inaccurate results

                for line in result.stdout.splitlines():
                    if line.startswith("%Cpu(s):"):
                        parts = line.split()
                        cpu_load = parts[1]
            except Exception as e:  # Exceptions are likely due to code running powershell and not a linux based cli
                print(e)

        # log_print(f"Target: {speed_results[0].data['target']}")
        json_data["data"]["targetDrone"].append(target_drone)
        json_data["data"]["avgUploadThroughputMbps"].append(avg_upload_throughput_mbps)
        json_data["data"]["avgUploadRttMs"].append(avg_upload_rtt_ms)
        json_data["data"]["minUploadThroughput"].append(min_upload_throughput)
        json_data["data"]["maxUploadThroughput"].append(max_upload_throughput)
        json_data["data"]["minUploadRtt"].append(min_upload_rtt)
        json_data["data"]["maxUploadRtt"].append(max_upload_rtt)
        json_data["data"]["avgDownloadThroughputMbps"].append(avg_download_throughput_mbps)
        json_data["data"]["avgDownloadRttMs"].append(avg_download_rtt_ms)
        json_data["data"]["minDownloadThroughput"].append(min_download_throughput)
        json_data["data"]["maxDownloadThroughput"].append(max_download_throughput)
        json_data["data"]["minDownloadRtt"].append(min_download_rtt)
        json_data["data"]["maxDownloadRtt"].append(max_download_rtt)
        json_data["data"]["uwbRange"].append(uwb_range)
        json_data["data"]["cpuLoad"].append(cpu_load)
        json_data["data"]["memoryUsage"].append(memory_usage)

        with json_path.open("w", encoding="utf-8") as f:
            json.dump(json_data, f, indent=2)

        print("\n" + "=" * 70)
        print(
            f"RANGE TEST RESULTS (Test #{test_number}) FROM {flight_settings.current_drone_ID} -> {target_drone}"
        )
        print("=" * 70)

        print(f"\nTotal Samples: {len(speed_results)}")

        print("\n--- UPLOAD STATISTICS ---")
        print("Throughput:")
        print(
            f"  Average: {avg_upload_throughput_mbps:.2f} Mbps ({avg_upload_throughput_mbps * 1000:.2f} Kbps)"
        )
        print(f"  Minimum: {min_upload_throughput:.2f} Mbps")
        print(f"  Maximum: {max_upload_throughput:.2f} Mbps")
        print("Round-Trip Time:")
        print(f"  Average: {avg_upload_rtt_ms:.2f} ms")
        print(f"  Minimum: {min_upload_rtt:.2f} ms")
        print(f"  Maximum: {max_upload_rtt:.2f} ms")

        print("\n--- DOWNLOAD STATISTICS ---")
        print("Throughput:")
        print(
            f"  Average: {avg_download_throughput_mbps:.2f} Mbps ({avg_download_throughput_mbps * 1000:.2f} Kbps)"
        )
        print(f"  Minimum: {min_download_throughput:.2f} Mbps")
        print(f"  Maximum: {max_download_throughput:.2f} Mbps")
        print("Round-Trip Time:")
        print(f"  Average: {avg_download_rtt_ms:.2f} ms")
        print(f"  Minimum: {min_download_rtt:.2f} ms")
        print(f"  Maximum: {max_download_rtt:.2f} ms")

        print("\n--- RANGE & PERFORMANCE ---")
        print(f"  UWB Range: {uwb_range} m")
        if get_perf:
            print(f"  CPU Load:     {cpu_load}%")
            print(f"  Memory Total: {total_memory} MB")
            print(f"  Memory Avail: {available_memory} MB")
            print(f"  Memory Usage: {memory_usage:.1f}%")

        print("=" * 70 + "\n")


if __name__ == "__main__":
    asyncio.run(main())
