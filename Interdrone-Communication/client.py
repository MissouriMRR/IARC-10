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

    # Start client code and call the client_loop()
    async def start_client_async(self):
        await self.client_loop()

    # Check for new messages to send and create tasks to send them
    async def client_loop(self) -> None:
        # Keep track of background clientMessageTasks
        clientMessageTasks: set[asyncio.Task[None]] = set()

        while True:
            # Check for new message from clientInData
            if not self.clientInData.empty():
                # Get new message from clientInData
                message: dict[str, bool | float | str] = await self.clientInData.get()

                # Create a background task to handle this message (allows for asynchronous messaging)
                clientMessageTask = asyncio.create_task(self.handle_message(message))
                clientMessageTasks.add(clientMessageTask)
                clientMessageTask.add_done_callback(
                    clientMessageTasks.discard
                )  # Clean up completed tasks

            # Wait 0.05 second before checking for next message
            await asyncio.sleep(0.05)

    # Create messageTasks to send data to all other drones
    async def handle_message(self, message: dict[str, bool | float | str]):
        message["timestamp"] = (
            time.time()
        )  # TODO WITH TEST REWORK MOVE THIS WHEN MESSAGE IS ADDED TO CLIENT IN DATA
        clientMessage = json.dumps(message)

        # Create tasks for all drone connections
        messageTasks: list[asyncio.Task[str]] = []
        for i in range(len(self.otherDronesIps)):
            messageTask = asyncio.create_task(
                self.send_data_async(
                    serverIP=self.otherDronesIps[i],
                    serverPort=self.otherDronesPorts[i],
                    clientMessage=clientMessage,
                )
            )
            messageTasks.append(messageTask)

        # Run all messageTasks concurrently
        _ = await asyncio.gather(*messageTasks, return_exceptions=True)

    # Takes data and sends it passed in server
    async def send_data_async(
        self, serverIP: str, serverPort: int, clientMessage: str
    ) -> str:
        try:
            # Open async connection with timeout
            reader, writer = await asyncio.wait_for(
                asyncio.open_connection(serverIP, serverPort), timeout=1.0
            )

            # Send data with end of data char (\n)
            writer.write((clientMessage + "\n").encode())
            await writer.drain()

            # Receive response
            serverResponseData = await asyncio.wait_for(
                reader.readuntil(b"\n"), timeout=2.0
            )
            receiveTime = time.time()

            # Close connection
            writer.close()
            await writer.wait_closed()

            # Call process_client_data
            await self.process_client_data(
                clientMessage, serverResponseData, receiveTime, serverIP, serverPort
            )

            return serverResponseData.decode()

        except asyncio.TimeoutError:
            raise Exception(f"Timeout connecting to {serverIP}:{serverPort}")
        except ConnectionRefusedError:
            raise Exception(f"Connection refused by {serverIP}:{serverPort}")
        except Exception as e:
            raise Exception(f"Error connecting to {serverIP}:{serverPort}: {str(e)}")

    async def process_client_data(
        self,
        clientMessage: str,
        serverResponseData: bytes,
        receiveTime: float,
        serverIP: str,
        serverPort: int,
    ) -> None:
        originalMessage = json.loads(s=clientMessage)
        match int(originalMessage["messageId"]):
            # Pathfinding Message
            case 1:
                pass
            # Heartbeat Message
            case 4:
                await self.clientOutData.put(item=serverResponseData.decode())
                pass
            # Speedtest Message
            case 13:
                # Calculate Round-Trip Time (RTT) in seconds
                rttSeconds: float = receiveTime - float(originalMessage["timestamp"])

                # Calculate the size of the JSON message that was sent
                sizeBytes = len(clientMessage.encode("utf-8"))

                # Avoid division by zero if RTT is extremely small
                if rttSeconds > 0.0:
                    # (bytes * 8 bits/byte) / 1000 bits/kbit = kilobits
                    # kilobits / seconds = kbps
                    throughputKbps = (sizeBytes * 8 / 1000) / rttSeconds
                else:
                    throughputKbps = float("inf")

                result = {
                    "messageId": "13",
                    "target": f"{serverIP}:{serverPort}",
                    "rttMs": round(rttSeconds * 1000, 2),
                    "sizeKb": round(sizeBytes / 1024, 2),  # Size in KiloBytes (KB)
                    "throughputKbps": round(throughputKbps, 2),
                }
                await self.clientOutData.put(item=json.dumps(result))
                pass
            case 400:
                await self.clientOutData.put(item="OMG the app contacted us!")
            case _:
                # If no message ID, return server response data
                await self.clientOutData.put(item=serverResponseData.decode())

    # Helper method to run the async client
    def run(self):
        asyncio.run(self.start_client_async())
