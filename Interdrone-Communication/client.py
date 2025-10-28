from asyncio.queues import Queue

import time
import asyncio
import sys
import json


class Client:
    # Client Class constructor. Used to pass in JSON Data
    def __init__(
        self,
        jsonData,
        clientInData: Queue[dict[str, int | float | str]],
        clientOutData: Queue[str],
    ):
        self.jsonData = jsonData
        self.clientInData: Queue[dict[str, bool | float | str]] = clientInData
        self.clientOutData: Queue[str] = clientOutData

        # Check for sys arg for drone selfId
        self.droneId: str
        try:
            self.droneId = sys.argv[1]
        except Exception:
            self.droneId = jsonData["localInfo"]["selfId"]

        # Instantiate otherDrones lists
        self.otherDronesIps: list[str] = []
        self.otherDronesPorts: list[int] = []

        # Parse JSON to get IPs and Ports of drones to connect to
        # Loop through drones 1-4
        for i in range(1, 5):
            # If drone is self (drone running this script) don't add them otherDrones list
            if i != int(self.droneId):
                # Add other drones IP and Ports to their respective lists
                self.otherDronesIps.append(
                    self.jsonData["drones"][str(i)]["ip"]
                )  # This will be simplified after JSON parser is implemented
                self.otherDronesPorts.append(
                    int(self.jsonData["drones"][str(i)]["port"])
                )

    # Start client and try to send messages to servers using asyncio
    async def start_client_async(self):
        while True:
            # Check for new input message
            if not self.clientInData.empty():
                # Get message from clientInData
                message = await self.clientInData.get()
                message["timestamp"] = time.time()

                # Serialize message to JSON to be able to send to servers
                jsonMessage = json.dumps(message)

                # Create coroutines for all drone connections
                coroutines = []
                for i in range(len(self.otherDronesIps)):
                    coroutine = self.send_data_async(
                        serverIP=self.otherDronesIps[i],
                        serverPort=self.otherDronesPorts[i],
                        jsonMessage=jsonMessage,
                    )
                    coroutines.append(coroutine)

                # Run all coroutines concurrently
                results = await asyncio.gather(*coroutines, return_exceptions=True)

                # Process results
                for i, result in enumerate(results):
                    if isinstance(result, Exception):
                        pass
                        # print(f"Connection to {self.otherDronesIps[i]} failed: {result}")
                    else:
                        pass
                        # print(
                        #     f"Successfully communicated with {self.otherDronesIps[i]}: {result}"
                        # )

            # Wait 0.05 second before checking for next message
            await asyncio.sleep(0.05)

    # Async coroutine to send data to one server
    # TODO add a data param
    async def send_data_async(self, serverIP: str, serverPort: int, jsonMessage: str):
        try:
            # Open async connection with timeout
            reader, writer = await asyncio.wait_for(
                asyncio.open_connection(serverIP, serverPort), timeout=1.0
            )

            # Send data
            writer.write(jsonMessage.encode())
            await writer.drain()

            # Receive response
            data = await asyncio.wait_for(reader.read(1024), timeout=1.0)
            receiveTime = time.time()

            # Close connection
            writer.close()
            await writer.wait_closed()

            # Call process_client_data
            await self.process_client_data(
                jsonMessage, data, receiveTime, serverIP, serverPort
            )

            return data.decode()

        except asyncio.TimeoutError:
            raise Exception(f"Timeout connecting to {serverIP}:{serverPort}")
        except ConnectionRefusedError:
            raise Exception(f"Connection refused by {serverIP}:{serverPort}")
        except Exception as e:
            raise Exception(f"Error connecting to {serverIP}:{serverPort}: {str(e)}")

    async def process_client_data(
        self,
        jsonMessage,
        data,
        receiveTime,
        serverIP: str,
        serverPort: int,
    ) -> None:
        originalMessage = json.loads(s=jsonMessage)
        match int(originalMessage["messageId"]):
            # Pathfinding Message
            case 1:
                pass
            # Heartbeat Message
            case 4:
                await self.clientOutData.put(item=data.decode())
                pass
            # Speedtest Message
            case 13:
                # Calculate Round-Trip Time (RTT) in seconds
                rtt_seconds: float = receiveTime - originalMessage["timestamp"]

                # Calculate the size of the JSON message that was sent
                size_bytes = len(jsonMessage.encode("utf-8"))

                # Calculate throughput in kilobits per second (kbps)
                # Avoid division by zero if RTT is extremely small
                if rtt_seconds > 0.0:
                    # (bytes * 8 bits/byte) / 1000 bits/kbit = kilobits
                    # kilobits / seconds = kbps
                    throughput_kbps = (size_bytes * 8 / 1000) / rtt_seconds
                else:
                    throughput_kbps = float("inf")

                result = {
                    "messageId": "13",
                    "target": f"{serverIP}:{serverPort}",
                    "rttMs": round(rtt_seconds * 1000, 2),
                    "sizeKb": round(size_bytes / 1024, 2),  # Size in KiloBytes (KB)
                    "throughputKbps": round(throughput_kbps, 2),
                }
                await self.clientOutData.put(item=json.dumps(result))
                pass
            case _:
                # If no message ID, return server response data
                await self.clientOutData.put(item=data)

    # Helper method to run the async client
    def run(self):
        asyncio.run(self.start_client_async())
