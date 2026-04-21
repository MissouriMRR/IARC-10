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
from interdrone_communication.network_config import NetworkConfig
from interdrone_communication.networking_thread import NetworkingThread
from interdrone_communication.networking_interface import NetworkingInterface


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
# TODO update logging to be more readable when no spreadsheet


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
    networkConfig = NetworkConfig()

    # Declare flag variables
    droneId: int
    numDrones: int
    dronesToSendData: list[int]
    getPerf: bool
    uwbEnabled: bool
    continuousTesting: bool

    # Get drone ID
    if args.id is not None:
        droneId = args.id
        networkConfig.set_self_id(droneId)
    else:
        droneId = int(networkConfig.get_self_id())

    # TODO decide how I want to set up num drones / other drones
    if args.targets is not None:
        dronesToSendData = args.targets
        if args.numDrones is not None and len(dronesToSendData) != args.numDrones:
            parser.error(
                "--numDrones must match the number of values passed to --targets"
            )
        numDrones = len(dronesToSendData)
    elif args.numDrones is not None:
        numDrones = args.numDrones
        dronesToSendData = []
        for i in range(numDrones):
            drone_id = int(
                input(f"Input ID of drone {i + 1} you are talking to in test: ")
            )
            dronesToSendData.append(drone_id)
    else:
        dronesToSendData = []
        numDrones = 3  # Needed for edge case later on

    getPerf = args.perf
    uwbEnabled = args.uwb
    continuousTesting = args.continuousTesting

    if args.batmanLocation is not None:
        batmanLocation = args.batmanLocation
    else:
        batmanLocation = parse_batman_location(
            input(
                "Is batman running on the pi's network card or the wifi adapter (1 or 2): "
            )
        )

    # Setup JSON logging file
    batmanLocationStr = ""
    if batmanLocation == 1:
        batmanLocationStr = "PI Network Card"
    else:
        batmanLocationStr = "Wifi Adapter"
    logTitle = f"Range Test from {droneId} to {dronesToSendData} on {batmanLocationStr}. GetPerf: {getPerf}, UWB: {uwbEnabled} "
    fileName = (
        f"RT_{droneId}_{str(dronesToSendData)}_Perf({getPerf})_UWB({uwbEnabled}))"
    )

    # Start networking thread
    networkingThreadClassInstance: NetworkingThread = NetworkingThread()
    resourcesReady: queue.Queue[NetworkingInterface] = queue.Queue(maxsize=1)
    networkingThread = threading.Thread(
        target=networkingThreadClassInstance.run_networking_thread,
        args=(resourcesReady, networkConfig),
        daemon=True,
    )
    networkingThread.start()
    backgroundTasks: set[Task[None]] = set[Task[None]]()  # Used for logging thread

    # Wait for networking to be ready
    networking: NetworkingInterface = resourcesReady.get()
    print("Networking interface ready")

    speedTestMessage: Message = Message.create(
        id=MessageType.SPEED_TEST_REQUEST,
        dronesToSendData=tuple(
            dronesToSendData
        ),  # Modify this for selective speed test
        senderId=droneId,
        data={
            "initialUploadTime": 0.0,  # Set when queued to send
            "payloadSize": networkConfig.get_speed_test_data_size() * 1024,
            "payload": "X"
            * (
                networkConfig.get_speed_test_data_size() * 1024
            ),  # Multiply string by a specified size of Kb to create a payload size (It's just a very long string of X's to simulate data)
        },
    )

    speedResults: dict[int, list[Message]] = {0: [], 1: [], 2: [], 3: [], 4: []}
    numTestsPerTarget: list[int] = [0, 0, 0, 0, 0]
    testsFinishedPerTarget: list[bool] = [False, False, False, False, False]
    numQueriesPerTest = 100
    networking.queue_client_message(message=speedTestMessage)
    testFinished = False

    print("Starting range test")
    while True:
        try:
            # Check for client responses
            serverMsg = networking.try_get_server_message(timeout=0.02)
            if serverMsg is not None:
                # Print speed test results
                try:  # Append client Message to dict list
                    targetId: int = serverMsg.data["targetId"]
                    # print(targetId)
                    if targetId not in speedResults:
                        speedResults[targetId] = []
                    speedResults[targetId].append(serverMsg)
                    if len(speedResults[targetId]) >= numQueriesPerTest:
                        # Copy data and clear original list so we can keep receiving immediately
                        results_to_log = list(speedResults[targetId])
                        speedResults[targetId].clear()

                        # Increment test counter
                        current_test_num = numTestsPerTarget[targetId]
                        numTestsPerTarget[targetId] += 1

                        # Run logging in a separate thread
                        task: Task[None] = asyncio.create_task(
                            asyncio.to_thread(
                                log_data,
                                results_to_log,
                                networkConfig,
                                current_test_num,
                                logTitle,
                                fileName,
                                getPerf,
                            )
                        )
                        testsFinishedPerTarget[targetId] = True

                        # Add to set to prevent garbage collection
                        backgroundTasks.add(task)

                        # Remove from set when done
                        task.add_done_callback(backgroundTasks.discard)

                        print(
                            f"Test {current_test_num} completed from {networkConfig.get_self_id()} -> {targetId}"
                        )
                        # If all drones in range tests tests have finished
                        if sum(testsFinishedPerTarget) >= numDrones:
                            testFinished = True
                    elif len(speedResults[targetId]) % 10 == 0:
                        print(
                            f"Test #{len(speedResults[targetId])} completed for drone {targetId}"
                        )

                except Exception:
                    # print(f"Error processing result: {e}")
                    traceback.print_exc()

                    # print(f"Client Data: {serverMsg}")
            if testFinished:
                # Wait for logging tasks to finish
                while len(backgroundTasks) != 0:
                    await asyncio.sleep(0.05)
                _ = (
                    networking.empty_queues()
                )  # flush out old data to ensure next test is clean
                if not continuousTesting:
                    print("Individual test finished and data has been logged.")
                    _ = input(
                        "Please press enter when you wish to start another one or press ctrl + c to exit"
                    )
                    print("Next test is running!")
                testsFinishedPerTarget = [False, False, False, False, False]
                testFinished = False
            # If previous message has been sent, add new one to queue
            if networking.is_client_in_empty():
                networking.queue_client_message(message=speedTestMessage)
            await asyncio.sleep(0.1)  # Adjust sleep time as needed
        except (KeyboardInterrupt, asyncio.CancelledError):
            print("Shutting down...")
            break


def log_data(
    speedResults: list[Message],
    networkConfig: NetworkConfig,
    testNumber: int,
    logTitle: str,
    fileName: str,
    getPerf: bool,
):
    # Sanitize directory name (remove >) and create path structure
    folderName = "Logs/Range_Test"
    os.makedirs(folderName, exist_ok=True)

    jsonPath = Path(f"{folderName}/{fileName}.json")

    if not jsonPath.exists():
        initialData = {
            "title": logTitle,
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
        jsonPath.parent.mkdir(
            parents=True, exist_ok=True
        )  # safe if parent folder already exists
        jsonPath.write_text(json.dumps(initialData, indent=2), encoding="utf-8")

    # If json file does not exist, set it up then append

    with jsonPath.open("r", encoding="utf-8") as f:
        jsonData = json.load(f)

    if jsonData is None:
        print("Failed to read json data :(")
        return

    if speedResults:
        targetDrone = speedResults[0].data["targetId"]
        uploadThroughputs = [
            float(r.data["uploadThroughputKbps"]) for r in speedResults
        ]
        uploadRttms = [float(r.data["uploadRttMs"]) for r in speedResults]
        downloadThroughputs = [
            float(r.data["downloadThroughputKbps"]) for r in speedResults
        ]
        downloadRttMs = [float(r.data["downloadRttMs"]) for r in speedResults]
        for i in range(len(downloadRttMs)):
            if downloadRttMs[i] > 1:
                pass
                # log_print(
                #     f"Download rttms was {downloadRttMs[i]} ms at index {i}"
                # )
        # Calculate upload statistics
        avgUploadThroughputKbps = sum(uploadThroughputs) / len(uploadThroughputs)
        avgUploadThroughputMbps = avgUploadThroughputKbps / 1000
        avgUploadRttMs = sum(uploadRttms) / len(uploadRttms)
        minUploadThroughput = min(uploadThroughputs) / 1000
        maxUploadThroughput = max(uploadThroughputs) / 1000
        minUploadRtt = min(uploadRttms)
        maxUploadRtt = max(uploadRttms)

        # Calculate download statistics
        avgDownloadThroughputKbps = sum(downloadThroughputs) / len(downloadThroughputs)
        avgDownloadThroughputMbps = avgDownloadThroughputKbps / 1000
        avgDownloadRttMs = sum(downloadRttMs) / len(downloadRttMs)
        minDownloadThroughput = min(downloadThroughputs) / 1000
        maxDownloadThroughput = max(downloadThroughputs) / 1000
        minDownloadRtt = min(downloadRttMs)
        maxDownloadRtt = max(downloadRttMs)

        # Calculate range from UWB
        uwbRange = 0  # TODO SET THIS UP

        # Get processing specs from PI
        cpuLoad = 0
        totalMemory = 0
        availableMemory = 0
        memoryUsage = 0
        if getPerf:
            try:
                result = subprocess.run(
                    ["free", "-m"], capture_output=True, text=True, check=True
                )

                for line in result.stdout.splitlines():
                    if line.startswith("Mem:"):
                        parts = line.split()
                        totalMemory = int(parts[1])
                        availableMemory = int(parts[6])
                        break

                memoryUsage = ((totalMemory - availableMemory) / totalMemory) * 100

                # Get processor usage (takes 2 seconds to run due to delay from top)
                result = subprocess.run(
                    ["top", "-b", "-n", "2"], capture_output=True, text=True, check=True
                )  # Note: top can be inaccurate. If the CPU is under heavy load it can produce inaccurate results

                for line in result.stdout.splitlines():
                    if line.startswith("%Cpu(s):"):
                        parts = line.split()
                        cpuLoad = parts[1]
            except Exception as e:  # Exceptions are likely due to code running powershell and not a linux based cli
                print(e)

        # log_print(f"Target: {speedResults[0].data['target']}")
        jsonData["data"]["targetDrone"].append(targetDrone)
        jsonData["data"]["avgUploadThroughputMbps"].append(avgUploadThroughputMbps)
        jsonData["data"]["avgUploadRttMs"].append(avgUploadRttMs)
        jsonData["data"]["minUploadThroughput"].append(minUploadThroughput)
        jsonData["data"]["maxUploadThroughput"].append(maxUploadThroughput)
        jsonData["data"]["minUploadRtt"].append(minUploadRtt)
        jsonData["data"]["maxUploadRtt"].append(maxUploadRtt)
        jsonData["data"]["avgDownloadThroughputMbps"].append(avgDownloadThroughputMbps)
        jsonData["data"]["avgDownloadRttMs"].append(avgDownloadRttMs)
        jsonData["data"]["minDownloadThroughput"].append(minDownloadThroughput)
        jsonData["data"]["maxDownloadThroughput"].append(maxDownloadThroughput)
        jsonData["data"]["minDownloadRtt"].append(minDownloadRtt)
        jsonData["data"]["maxDownloadRtt"].append(maxDownloadRtt)
        jsonData["data"]["uwbRange"].append(uwbRange)
        jsonData["data"]["cpuLoad"].append(cpuLoad)
        jsonData["data"]["memoryUsage"].append(memoryUsage)

        with jsonPath.open("w", encoding="utf-8") as f:
            json.dump(jsonData, f, indent=2)

        print("\n" + "=" * 70)
        print(
            f"RANGE TEST RESULTS (Test #{testNumber}) FROM {networkConfig.get_self_id()} -> {targetDrone}"
        )
        print("=" * 70)

        print(f"\nTotal Samples: {len(speedResults)}")

        print("\n--- UPLOAD STATISTICS ---")
        print("Throughput:")
        print(
            f"  Average: {avgUploadThroughputMbps:.2f} Mbps ({avgUploadThroughputMbps * 1000:.2f} Kbps)"
        )
        print(f"  Minimum: {minUploadThroughput:.2f} Mbps")
        print(f"  Maximum: {maxUploadThroughput:.2f} Mbps")
        print("Round-Trip Time:")
        print(f"  Average: {avgUploadRttMs:.2f} ms")
        print(f"  Minimum: {minUploadRtt:.2f} ms")
        print(f"  Maximum: {maxUploadRtt:.2f} ms")

        print("\n--- DOWNLOAD STATISTICS ---")
        print("Throughput:")
        print(
            f"  Average: {avgDownloadThroughputMbps:.2f} Mbps ({avgDownloadThroughputMbps * 1000:.2f} Kbps)"
        )
        print(f"  Minimum: {minDownloadThroughput:.2f} Mbps")
        print(f"  Maximum: {maxDownloadThroughput:.2f} Mbps")
        print("Round-Trip Time:")
        print(f"  Average: {avgDownloadRttMs:.2f} ms")
        print(f"  Minimum: {minDownloadRtt:.2f} ms")
        print(f"  Maximum: {maxDownloadRtt:.2f} ms")

        print("\n--- RANGE & PERFORMANCE ---")
        print(f"  UWB Range: {uwbRange} m")
        if getPerf:
            print(f"  CPU Load:     {cpuLoad}%")
            print(f"  Memory Total: {totalMemory} MB")
            print(f"  Memory Avail: {availableMemory} MB")
            print(f"  Memory Usage: {memoryUsage:.1f}%")

        print("=" * 70 + "\n")


if __name__ == "__main__":
    asyncio.run(main())
