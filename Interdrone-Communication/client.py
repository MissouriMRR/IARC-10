from asyncio.queues import Queue
from json_config_reader import json_config_reader

import time
import asyncio
import json
from typed_dicts_classes import MessageData


class Client:
    # Client Class constructor. Used to pass in JSON Data
    def __init__(
        self,
        jsonConfigData: json_config_reader,
        clientInData: Queue[MessageData],
        clientOutData: Queue[MessageData],
    ):
        self.jsonConfigData: json_config_reader = jsonConfigData
        self.clientInData: Queue[MessageData] = clientInData
        self.clientOutData: Queue[MessageData] = clientOutData

        # Check for droneId from flag in main.py
        self.droneId: int = jsonConfigData.get_self_id()

        # Instantiate otherDrones lists
        self.otherDronesIps: list[str] = []
        self.otherDronesPorts: list[int] = []
        self.otherDronesIds: list[int] = []

        # Loop through drones all drones to get IPs, Ports, and IDs of drones to connect to
        for i in range(1, self.jsonConfigData.get_number_of_drones() + 1):
            # If drone is self (drone running this script) don't add them otherDrones list
            if i != self.droneId:
                # Add other drones IP and Ports to their respective lists
                self.otherDronesIps.append(self.jsonConfigData.get_drone_ip(droneId=i))
                self.otherDronesPorts.append(
                    self.jsonConfigData.get_drone_port(droneId=i)
                )
                self.otherDronesIds.append(i)

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
                message: MessageData = await self.clientInData.get()
                # Create a background task to handle this message (allows for asynchronous messaging)
                clientMessageTask = asyncio.create_task(self.handle_message(message))
                clientMessageTasks.add(clientMessageTask)
                clientMessageTask.add_done_callback(
                    clientMessageTasks.discard
                )  # Clean up completed tasks

            # Wait 0.05 second before checking for next message
            await asyncio.sleep(0.05)

    # Create messageTasks to send data to all other drones
    async def handle_message(self, message: MessageData):
        # Determine which drones to send message to
        dronesToSendData: list[int] = []

        # If dronesToSendData list has id values, only send message to those drones
        sendToApp: bool = False
        if message["dronesToSendData"] != []:
            if message["dronesToSendData"] == ["app"]:
                sendToApp = True
            else:
                dronesToSendData = message["dronesToSendData"]
        # Else dronesToSendData list is empty, attempt to send data to all other drones
        else:
            dronesToSendData = self.otherDronesIds

        # Message Preprocessing
        if message["messageId"] == 513:
            message["data"]["initialUploadTime"] = time.perf_counter()
        clientMessageDump = json.dumps(message)

        # Create messageTasks list to store tasks for all drone connections
        messageTasks: list[asyncio.Task[str]] = []

        # Loop through otherDronesIds and create to task to send message data to them if they're in dronesToSendData to list
        if sendToApp:
            messageTask = asyncio.create_task(
                self.send_data_async(
                    serverIP=self.jsonConfigData.get_app_ip(),
                    serverPort=self.jsonConfigData.get_app_port(),
                    clientMessageDump=clientMessageDump,
                )
            )
            messageTasks.append(messageTask)
        else:
            for i in range(len(self.otherDronesIds)):
                if self.otherDronesIds[i] in dronesToSendData:
                    messageTask = asyncio.create_task(
                        self.send_data_async(
                            serverIP=self.otherDronesIps[i],
                            serverPort=self.otherDronesPorts[i],
                            clientMessageDump=clientMessageDump,
                        )
                    )
                    messageTasks.append(messageTask)

        # Run all messageTasks concurrently
        _ = await asyncio.gather(*messageTasks, return_exceptions=True)

    # Takes data and sends it passed in server
    async def send_data_async(
        self, serverIP: str, serverPort: int, clientMessageDump: str
    ) -> str:
        try:
            # Open async connection with timeout
            reader, writer = await asyncio.wait_for(
                asyncio.open_connection(serverIP, serverPort), timeout=1.0
            )

            # Send data with end of data char (\n)
            writer.write((clientMessageDump + "\n").encode())
            await writer.drain()

            # Receive response and decode
            serverResponseBytes = await asyncio.wait_for(
                reader.readuntil(b"\n"), timeout=2.0
            )
            serverResponseDump: str = serverResponseBytes.decode()

            # Close connection
            writer.close()
            await writer.wait_closed()

            # Call process_client_data
            await self.process_client_data(
                clientMessageDump, serverResponseDump, serverIP, serverPort
            )

            return serverResponseBytes.decode()

        except asyncio.TimeoutError:
            # If timeout and message is a JSON startup message (messageId = 501), resend message
            # print(f"Timeout connecting to {serverIP}:{serverPort}")
            try:
                clientMessageJson: MessageData = json.loads(clientMessageDump)
                if clientMessageJson["messageId"] == 501:
                    clientMessageJson["data"]["payload"] = "Timeout"
                    await self.clientOutData.put(clientMessageJson)
            except Exception as e:
                print(f"Error processing timeout message: {e}")
            raise Exception(f"Timeout connecting to {serverIP}:{serverPort}")
        except ConnectionRefusedError:
            raise Exception(f"Connection refused by {serverIP}:{serverPort}")
        except Exception as e:
            raise Exception(f"Error connecting to {serverIP}:{serverPort}: {str(e)}")

    async def process_client_data(
        self,
        clientMessageDump: str,
        serverResponseDump: str,
        serverIP: str,
        serverPort: int,
    ) -> None:
        clientMessage: MessageData = json.loads(s=clientMessageDump)
        serverResponse: MessageData = json.loads(s=serverResponseDump)

        match int(
            serverResponse["messageId"]
        ):  # TODO may want to change this to serverResponse. Need to standardize this and talk to team. Server should send client MessageData instead of just a string and we should base processing on that
            # Default Server Response Message
            case 505:
                # await self.clientOutData.put(item=serverResponseDump)  # Keep commented out for performance unless you want to see default server message
                pass
            # Network Speedtest Message (Needs a client response)\
            case 513:
                # Client receives response - set final download time
                receiveTime = time.perf_counter()

                # Calculate Server Processing Time (Delta on Server Clock)
                serverProcessingTime = float(
                    serverResponse["data"]["initialDownloadTime"]
                ) - float(serverResponse["data"]["finalUploadTime"])

                # Calculate Total Round Trip Time (Delta on Client Clock)
                totalRtt = receiveTime - float(
                    clientMessage["data"]["initialUploadTime"]
                )

                # Calculate Network RTT (Total - Processing)
                networkRtt = totalRtt - serverProcessingTime

                # Since the server echoes the payload, upload and download sizes are roughly equal,
                # so we estimate upload and download time as half of the network RTT.
                estimatedOneWayTime = networkRtt / 2

                uploadTime = estimatedOneWayTime
                downloadTime = estimatedOneWayTime

                uploadSizeBytes = len(clientMessageDump.encode("utf-8"))
                uploadThroughputKbps = (
                    (uploadSizeBytes * 8 / 1000) / uploadTime
                    if uploadTime > 0
                    else float("inf")
                )

                downloadSizeBytes = len(serverResponseDump)
                downloadThroughputKbps = (
                    (downloadSizeBytes * 8 / 1000) / downloadTime
                    if downloadTime > 0
                    else float("inf")
                )

                # For now, this will be a separate message from 513 to maintain
                # the immutability of the fields in 513. This is an ugly hack,
                # but it works for now before I refactor messages and make them
                # their own class in order to enforce immutability.
                result: MessageData = {
                    "messageId": 514,
                    "dronesToSendData": [],
                    "data": {
                        "target": f"{serverIP}:{serverPort}",
                        "uploadRttMs": round(uploadTime * 1000, 2),
                        "uploadThroughputKbps": round(uploadThroughputKbps, 2),
                        "downloadRttMs": round(downloadTime * 1000, 2),
                        "downloadThroughputKbps": round(downloadThroughputKbps, 2),
                    },
                }
                await self.clientOutData.put(item=result)
            case _:
                # If no message ID, return server response data
                await self.clientOutData.put(item=serverResponse)

    # Helper method to run the async client
    def run(self):
        asyncio.run(self.start_client_async())
