from _t_message_types import Message, MessageType
from json_config_reader import JsonConfigReader
from networking_thread import NetworkingThread

import asyncio
import argparse
import queue
import threading


from networking_interface import NetworkingInterface


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

    # Wait for networking to be ready
    networking: NetworkingInterface = resourcesReady.get()
    print("Networking interface ready")

    speedTestMessage: Message = Message.create(
        id=MessageType.SPEED_TEST_REQUEST,
        dronesToSendData=(),
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

    # Run both tasks concurrently
    try:
        print("Server and Client started")

        numberOfQueries = 100
        i = 0
        speedResults: list[Message] = []
        # Add network test message to clientQueue to send
        networking.queue_client_message(message=speedTestMessage)
        # Send messages until numberOfQueries is hit
        while i < numberOfQueries:
            # Check for client responses
            clientMsg = networking.try_get_client_response(timeout=0.02)
            if clientMsg is not None:
                # Print speed test results
                try:
                    # TODO look into the extreme numbers appearing here and try to fix them
                    i += 1
                    speedResults.append(clientMsg)
                    print(f"Test {i}/{numberOfQueries} completed")
                except Exception as e:
                    print(f"Error processing result: {e}")
                    print(f"Client Data: {clientMsg}")
            # If previous message has been sent, add new one to queue
            if networking.is_client_in_empty():
                networking.queue_client_message(message=speedTestMessage)
            await asyncio.sleep(0.1)  # Adjust sleep time as needed

        # Print results summary
        print("\n" + "=" * 70)
        print("NETWORK SPEED TEST RESULTS")
        print("=" * 70)

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
                    print(f"Download rttms was {downloadRttMs[i]} ms at index {i}")
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

            print(f"\nTotal Tests: {len(speedResults)}")
            print(f"Target: {speedResults[0].data['target']}")

            print("\n--- UPLOAD STATISTICS ---")
            print("Throughput:")
            print(
                f"  Average: {avgUploadThroughputMbps:.2f} Mbps ({avgUploadThroughputKbps:.2f} Kbps)"
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
                f"  Average: {avgDownloadThroughputMbps:.2f} Mbps ({avgDownloadThroughputKbps:.2f} Kbps)"
            )
            print(f"  Minimum: {minDownloadThroughput:.2f} Mbps")
            print(f"  Maximum: {maxDownloadThroughput:.2f} Mbps")
            print("Round-Trip Time:")
            print(f"  Average: {avgDownloadRttMs:.2f} ms")
            print(f"  Minimum: {minDownloadRtt:.2f} ms")
            print(f"  Maximum: {maxDownloadRtt:.2f} ms")
        else:
            print("\nNo results collected!")

        print("=" * 70 + "\n")
    except (KeyboardInterrupt, asyncio.CancelledError):
        print("Shutting down...")


if __name__ == "__main__":
    asyncio.run(main())
