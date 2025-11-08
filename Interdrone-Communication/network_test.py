from asyncio.queues import Queue
from message_types import MessageData

import asyncio
from typing import Any
import server
import client
import json
import sys
import time


async def main():
    # Get JSON Data
    with open("config.json", "r") as file:
        data: dict[str, Any] = json.load(file)  # TODO verify and update this

    # Create Server and Client Data queues to pass data in and out of tasks
    serverData: Queue[str] = asyncio.Queue()  # May need to change queue type to any

    # TODO talk to Harper to see if we need a serverInData
    clientInData: Queue[MessageData] = (
        asyncio.Queue()
    )  # TODO remove Any once we figure out how to structure our passed in data
    clientOutData: Queue[str] = asyncio.Queue()

    # Instantiate Server and Client
    serverInstance = server.Server(jsonData=data, serverOutData=serverData)
    clientInstance = client.Client(
        jsonData=data, clientInData=clientInData, clientOutData=clientOutData
    )

    # Run both server and client concurrently
    serverTask = asyncio.create_task(serverInstance.start_server_async())
    clientTask = asyncio.create_task(clientInstance.start_client_async())

    droneId: str
    try:
        droneId = sys.argv[1]
    except Exception:
        droneId = data["localInfo"]["selfId"]

    speedTestMessage: MessageData = {
        "messageId": 513,
        "data": {
            "initialUploadTime": time.time(),  # Set when queued to send
            "finalUploadTime": 0.0,
            "initialDownloadTime": 0.0,
            "finalDownloadTime": 0.0,
            "senderId": droneId,
            "payloadSize": (int(data["localInfo"]["speedTestKbDataSize"])) * 1024,
            "payload": "X"
            * (
                (int(data["localInfo"]["speedTestKbDataSize"])) * 1024
            ),  # Multiply string by a specified size of Kb to create a payload size
        },
    }

    # Run both tasks concurrently
    try:
        print("Server and Client started")

        numberOfQueries = 100
        i = 0
        speedResults: list[MessageData] = []
        # Add network test message to clientQueue to send
        print("adding to queue")
        await clientInData.put(item=speedTestMessage)
        # Continuous loop for other functionality
        while i < numberOfQueries:
            # Check for clientOutData from the client task
            # if droneId == int(1):
            #     print(f"out here with i={i}")
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
            if clientInData.empty():
                await clientInData.put(item=speedTestMessage)
            await asyncio.sleep(0.1)  # Adjust sleep time as needed

        # Print results summary
        print("\n" + "=" * 70)
        print("NETWORK SPEED TEST RESULTS")
        print("=" * 70)

        if speedResults:
            upload_throughputs = [
                float(r["data"]["uploadThroughputKbps"]) for r in speedResults
            ]
            upload_rtts = [float(r["data"]["uploadRttMs"]) for r in speedResults]
            download_throughputs = [
                float(r["data"]["downloadThroughputKbps"]) for r in speedResults
            ]
            download_rtts = [float(r["data"]["downloadRttMs"]) for r in speedResults]

            # Calculate upload statistics
            avgUploadThroughputKbps = sum(upload_throughputs) / len(upload_throughputs)
            avgUploadThroughputMbps = avgUploadThroughputKbps / 1000
            avgUploadRttMs = sum(upload_rtts) / len(upload_rtts)
            minUploadThroughput = min(upload_throughputs) / 1000
            maxUploadThroughput = max(upload_throughputs) / 1000
            minUploadRtt = min(upload_rtts)
            maxUploadRtt = max(upload_rtts)

            # Calculate download statistics
            avgDownloadThroughputKbps = sum(download_throughputs) / len(
                download_throughputs
            )
            avgDownloadThroughputMbps = avgDownloadThroughputKbps / 1000
            avgDownloadRttMs = sum(download_rtts) / len(download_rtts)
            minDownloadThroughput = min(download_throughputs) / 1000
            maxDownloadThroughput = max(download_throughputs) / 1000
            minDownloadRtt = min(download_rtts)
            maxDownloadRtt = max(download_rtts)

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
