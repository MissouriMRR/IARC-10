from asyncio import Task


from message_types import Message, MessageType
from json_config_reader import JsonConfigReader
from networking_thread import NetworkingThread

import asyncio
import argparse
import queue
import threading
import traceback
import os

from networking_interface import NetworkingInterface


# Plan for network test updates: Filter logs based on self id and target
async def main():
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

    # Start networking thread
    networkingThreadClassInstance: NetworkingThread = NetworkingThread()
    resourcesReady: queue.Queue[NetworkingInterface] = queue.Queue(maxsize=1)
    networkingThread = threading.Thread(
        target=networkingThreadClassInstance.run_networking_thread,
        args=(resourcesReady, jsonConfigData),
        daemon=True,
    )
    networkingThread.start()
    backgroundTasks: set[Task[None]] = set[Task[None]]()  # Used for logging thread

    # Wait for networking to be ready
    networking: NetworkingInterface = resourcesReady.get()
    print("Networking interface ready")

    speedTestMessage: Message = Message.create(
        id=MessageType.SPEED_TEST_REQUEST,
        dronesToSendData=(),  # Modify this for selective speed test
        data={
            "initialUploadTime": 0.0,  # Set when queued to send
            "finalUploadTime": 0.0,
            "initialDownloadTime": 0.0,
            "finalDownloadTime": 0.0,
            "senderId": droneId,
            "payloadSize": jsonConfigData.get_speed_test_data_size() * 1024,
            "payload": "X"
            * (
                jsonConfigData.get_speed_test_data_size() * 1024
            ),  # Multiply string by a specified size of Kb to create a payload size (It's just a very long string of X's to simulate data)
        },
    )

    continuousSpeedTest = True
    speedResults: dict[int, list[Message]] = {0: [], 1: [], 2: [], 3: [], 4: []}
    numTestsPerTarget: list[int] = [0, 0, 0, 0, 0]
    numQueriesPerTest = 100
    networking.queue_client_message(message=speedTestMessage)
    while continuousSpeedTest:
        try:
            # Check for client responses
            clientMsg = networking.try_get_client_response(timeout=0.02)
            if clientMsg is not None:
                # Print speed test results
                try:  # Append client Message to dict list
                    targetId: int = clientMsg.data["targetId"]
                    # print(targetId)
                    if targetId not in speedResults:
                        speedResults[targetId] = []
                    speedResults[targetId].append(clientMsg)
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
                                jsonConfigData,
                                current_test_num,
                            )
                        )

                        # Add to set to prevent garbage collection
                        backgroundTasks.add(task)

                        # Remove from set when done
                        task.add_done_callback(backgroundTasks.discard)

                        print(
                            f"Test {current_test_num} completed from {jsonConfigData.get_self_id()} -> {targetId}"
                        )
                except Exception:
                    # print(f"Error processing result: {e}")
                    traceback.print_exc()

                    # print(f"Client Data: {clientMsg}")
            # If previous message has been sent, add new one to queue
            if networking.is_client_in_empty():
                networking.queue_client_message(message=speedTestMessage)

            await asyncio.sleep(0.1)  # Adjust sleep time as needed
        except (KeyboardInterrupt, asyncio.CancelledError):
            print("Shutting down...")


# Do async function call to this
def log_data(
    speedResults: list[Message], jsonConfigData: JsonConfigReader, testNumber: int
):
    # Print results summary
    # Print results summary
    # Sanitize directory name (remove >) and create path structure
    folder_name = f"Logs/From_{jsonConfigData.get_self_id()}_To_{speedResults[0].data['targetId']}"
    os.makedirs(folder_name, exist_ok=True)

    file_path = f"{folder_name}/test-results-{testNumber}.txt"

    with open(file_path, "w") as log_file:

        def log_print(msg):
            print(msg)
            log_file.write(str(msg) + "\n")

        log_print("\n" + "=" * 70)
        log_print(
            f"NETWORK SPEED TEST RESULTS FROM {jsonConfigData.get_self_id()} -> {speedResults[0].data['targetId']}"
        )
        log_print("=" * 70)

        if speedResults:
            uploadThroughputs = [
                float(r.data["uploadThroughputKbps"]) for r in speedResults
            ]
            uploadRttms = [float(r.data["uploadRttMs"]) for r in speedResults]
            downloadThroughputs = [
                float(r.data["downloadThroughputKbps"]) for r in speedResults
            ]
            downloadRttMs = [float(r.data["downloadRttMs"]) for r in speedResults]
            # TODO investigate issue with 250ms+ times in first couple sends
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
            avgDownloadThroughputKbps = sum(downloadThroughputs) / len(
                downloadThroughputs
            )
            avgDownloadThroughputMbps = avgDownloadThroughputKbps / 1000
            avgDownloadRttMs = sum(downloadRttMs) / len(downloadRttMs)
            minDownloadThroughput = min(downloadThroughputs) / 1000
            maxDownloadThroughput = max(downloadThroughputs) / 1000
            minDownloadRtt = min(downloadRttMs)
            maxDownloadRtt = max(downloadRttMs)

            log_print(f"\nTotal Tests: {len(speedResults)}")
            # log_print(f"Target: {speedResults[0].data['target']}")

            log_print("\n--- UPLOAD STATISTICS ---")
            log_print("Throughput:")
            log_print(
                f"  Average: {avgUploadThroughputMbps:.2f} Mbps ({avgUploadThroughputKbps:.2f} Kbps)"
            )
            log_print(f"  Minimum: {minUploadThroughput:.2f} Mbps")
            log_print(f"  Maximum: {maxUploadThroughput:.2f} Mbps")
            log_print("Round-Trip Time:")
            log_print(f"  Average: {avgUploadRttMs:.2f} ms")
            log_print(f"  Minimum: {minUploadRtt:.2f} ms")
            log_print(f"  Maximum: {maxUploadRtt:.2f} ms")

            log_print("\n--- DOWNLOAD STATISTICS ---")
            log_print("Throughput:")
            log_print(
                f"  Average: {avgDownloadThroughputMbps:.2f} Mbps ({avgDownloadThroughputKbps:.2f} Kbps)"
            )
            log_print(f"  Minimum: {minDownloadThroughput:.2f} Mbps")
            log_print(f"  Maximum: {maxDownloadThroughput:.2f} Mbps")
            log_print("Round-Trip Time:")
            log_print(f"  Average: {avgDownloadRttMs:.2f} ms")
            log_print(f"  Minimum: {minDownloadRtt:.2f} ms")
            log_print(f"  Maximum: {maxDownloadRtt:.2f} ms")
        else:
            log_print("\nNo results collected!")

        log_print("=" * 70 + "\n")


if __name__ == "__main__":
    asyncio.run(main())
