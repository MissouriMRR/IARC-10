from asyncio.queues import Queue
from json_config_reader import json_config_reader

import time
import asyncio
from _t_message_types import Message, MessageType
from json_message_utilities import JsonMessageUtilities


class Client:
    # Client Class constructor. Used to pass in JSON Data
    def __init__(
        self,
        jsonConfigData: json_config_reader,
        clientInData: Queue[Message],
        clientOutData: Queue[Message],
    ):
        self.jsonConfigData: json_config_reader = jsonConfigData
        self.clientInData: Queue[Message] = clientInData
        self.clientOutData: Queue[Message] = clientOutData

        # Check for droneId from flag in main.py
        self.droneId: int = jsonConfigData.get_self_id()

        # Instantiate otherDrones lists
        self.otherDronesIps: list[str] = []
        self.otherDronesPorts: list[int] = []
        self.otherDronesIds: tuple[
            int, ...
        ] = ()  # Needs to be a tuple to align with dronesToSendData in Message class

        # Create a temporary list to update tuple with otherDronesIds
        tempOtherDronesIds: list[int] = list[int](self.otherDronesIds)
        # Loop through drones all drones to get IPs, Ports, and IDs of drones to connect to
        for i in range(1, self.jsonConfigData.get_number_of_drones() + 1):
            # If drone is self (drone running this script) don't add them otherDrones list
            if i != self.droneId:
                # Add other drones IP and Ports to their respective lists
                self.otherDronesIps.append(self.jsonConfigData.get_drone_ip(droneId=i))
                self.otherDronesPorts.append(
                    self.jsonConfigData.get_drone_port(droneId=i)
                )
                tempOtherDronesIds.append(i)
        # Update otherDronesIds tuple with tempOtherDronesIds values
        self.otherDronesIds = tuple[int, ...](tempOtherDronesIds)

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
                message: Message = await self.clientInData.get()
                # Create a background task to handle this message (allows for asynchronous messaging)
                clientMessageTask = asyncio.create_task(self.handle_message(message))
                clientMessageTasks.add(clientMessageTask)
                clientMessageTask.add_done_callback(
                    clientMessageTasks.discard
                )  # Clean up completed tasks

            # Wait 0.05 second before checking for next message
            await asyncio.sleep(0.05)

    # Create messageTasks to send data to all other drones
    async def handle_message(self, message: Message):
        # Determine which drones to send message to
        dronesToSendData: tuple[int, ...] = ()  # NOTE COULD BE SPOT OF ERROR

        # If dronesToSendData list has id values, only send message to those drones
        sendToApp: bool = False
        if message.dronesToSendData != ():
            # If you to send data to the app, use ID 0
            if message.dronesToSendData == (0,):
                sendToApp = True
            else:
                dronesToSendData = message.dronesToSendData
        # Else dronesToSendData list is empty, attempt to send data to all other drones
        else:
            dronesToSendData = self.otherDronesIds

        # Message Preprocessing
        # Update time value for Network Speed Test message
        if message.id == MessageType.SPEED_TEST_REQUEST:
            message.data["initialUploadTime"] = time.perf_counter()
        clientMessageDump: str = JsonMessageUtilities.message_to_json(message=message)

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
            # NOTE: In the future if you need the client to resend a message after a timeout view commit history,
            # 671f673486ea4c57ad2320c1506bafef70aa5e15 to see how we did it for the deprecated startup message
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
        clientMessage: Message = JsonMessageUtilities.message_from_json(
            payload=clientMessageDump
        )
        serverResponse: Message = JsonMessageUtilities.message_from_json(
            payload=serverResponseDump
        )

        match serverResponse.id:
            case MessageType.SERVER_DEFAULT_RESPONSE:
                # await self.clientOutData.put(item=serverResponseDump)  # Keep commented out for performance unless you want to see default server message
                pass
            # Network Speedtest Message (Needs a client response)
            case MessageType.SPEED_TEST_REQUEST:
                # Client receives response - set final download time
                receiveTime = time.perf_counter()

                # Calculate Server Processing Time (Delta on Server Clock)
                serverProcessingTime: float = float(
                    serverResponse.data["initialDownloadTime"]
                ) - float(serverResponse.data["finalUploadTime"])

                # Calculate Total Round Trip Time (Delta on Client Clock)
                totalRtt = receiveTime - clientMessage.data["initialUploadTime"]

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
                result: Message = Message.create(
                    id=MessageType.SPEED_TEST_RESPONSE,
                    dronesToSendData=(),
                    data={
                        "target": f"{serverIP}:{serverPort}",
                        "uploadRttMs": round(uploadTime * 1000, 2),
                        "uploadThroughputKbps": round(uploadThroughputKbps, 2),
                        "downloadRttMs": round(downloadTime * 1000, 2),
                        "downloadThroughputKbps": round(downloadThroughputKbps, 2),
                    },
                )
                await self.clientOutData.put(item=result)
            case _:
                # If no message ID, return server response data
                await self.clientOutData.put(item=serverResponse)

    # Helper method to run the async client
    def run(self):
        asyncio.run(self.start_client_async())
