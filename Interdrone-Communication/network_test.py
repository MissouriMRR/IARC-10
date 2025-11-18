from asyncio.queues import Queue
from typed_dicts_classes import MessageData
from json_config_reader import json_config_reader

import asyncio
import server
import client
import json
import sys


async def main():
    # Create jsonConfigData instance to get data from config file
    jsonConfigData: json_config_reader = json_config_reader()

    # Create Server and Client Data queues to pass data in and out of tasks
    serverOutData: Queue[str] = asyncio.Queue()
    clientInData: Queue[MessageData] = asyncio.Queue()
    clientOutData: Queue[str] = asyncio.Queue()

    # Instantiate Server and Client
    serverInstance = server.Server(
        jsonConfigData=jsonConfigData, serverOutData=serverOutData
    )
    clientInstance = client.Client(
        jsonConfigData=jsonConfigData,
        clientInData=clientInData,
        clientOutData=clientOutData,
    )

    # Run both server and client concurrently
    serverTask = asyncio.create_task(serverInstance.start_server_async())
    clientTask = asyncio.create_task(clientInstance.start_client_async())

    # Get our drones id (the sys arg here allows you pass in a self id from command line for efficient testing)
    droneId: int
    try:
        droneId = int(sys.argv[1])
    except Exception:
        droneId = jsonConfigData.get_self_id()

    speedTestMessage: MessageData = {
        "messageId": 513,
        "data": {
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
    }

    # Run both tasks concurrently
    try:
        print("Server and Client started")

        numberOfQueries = 100
        i = 0
        speedResults: list[MessageData] = []
        # Add network test message to clientQueue to send
        await clientInData.put(item=speedTestMessage)
        # Send messages until numberOfQueries is hit
        while i < numberOfQueries:
            # Check for clientOutData from the client task
            if not clientOutData.empty():
                clientMsg = await clientOutData.get()
                # Print speed test results
                try:
                    result: MessageData = json.loads(clientMsg)
                    i += 1
                    speedResults.append(result)
                    print(f"Test {i}/{numberOfQueries} completed")
                except Exception as e:
                    print(f"Error processing result: {e}")
                    print(f"Client Data: {clientMsg}")
            # If previous message has been sent, add new one to queue
            if clientInData.empty():
                await clientInData.put(item=speedTestMessage)
            await asyncio.sleep(0.1)  # Adjust sleep time as needed

        # Print results summary
        print("\n" + "=" * 70)
        print("NETWORK SPEED TEST RESULTS")
        print("=" * 70)

        if speedResults:
            uploadThroughputs = [
                float(r["data"]["uploadThroughputKbps"]) for r in speedResults
            ]
            uploadRttms = [float(r["data"]["uploadRttMs"]) for r in speedResults]
            downloadThroughputs = [
                float(r["data"]["downloadThroughputKbps"]) for r in speedResults
            ]
            downloadRttMs = [float(r["data"]["downloadRttMs"]) for r in speedResults]

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
            print(f"Target: {speedResults[0]['data']['target']}")

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
    except KeyboardInterrupt:
        print("Shutting down...")
        serverTask.cancel()
        clientTask.cancel()


if __name__ == "__main__":
    asyncio.run(main())
