from asyncio.queues import Queue
from json_config_reader import JsonConfigReader
from asyncio import StreamReader, StreamWriter
from dataclasses import dataclass

import time
import asyncio
from message_types import Message, MessageType
from json_message_utilities import JsonMessageUtilities


# Used by connectionPool to store TCP connections to other servers.
@dataclass
class PersistentConnection:
    reader: StreamReader
    writer: StreamWriter
    lock: asyncio.Lock
    lastUsed: float


class Client:
    # Client Class constructor. Used to pass in JSON Data
    def __init__(
        self,
        jsonConfigData: JsonConfigReader,
        clientInData: Queue[Message],
        clientOutData: Queue[Message],
    ):
        self.jsonConfigData: JsonConfigReader = jsonConfigData
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

        # Variables used
        self.connectionPool: dict[tuple[str, int], PersistentConnection] = {}
        self.connectionIdleTimeoutSec: float = 30.0
        self.lastCleanupTime: float = time.monotonic()

        # Cap concurrent per message handlers to avoid unbounded task buildup (buildup kills performance).
        self.maxInFlightMessageTasks: int = 6

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

        # Get range test toggle variable
        self.rangeTestEnabled: bool = self.jsonConfigData.get_range_test_toggle()

    # Start client code and call the client_loop()
    async def start_client_async(self):
        await self.client_loop()

    # Check for new messages to send and create tasks to send them
    async def client_loop(self) -> None:
        # Keep track of background clientMessageTasks
        clientMessageTasks: set[asyncio.Task[None]] = set()
        while True:
            handledMessage = False
            # Check for new message from clientInData
            while not self.clientInData.empty():
                handledMessage = True
                # Get new message from clientInData
                message: Message = await self.clientInData.get()

                # If too many messages tasks are currently running, wait for one to finish before adding next one
                while len(clientMessageTasks) >= self.maxInFlightMessageTasks:
                    done, _ = await asyncio.wait(
                        clientMessageTasks,
                        return_when=asyncio.FIRST_COMPLETED,
                    )
                    # Consume task exceptions so they do not get logged as un-retrieved.
                    for finishedTask in done:
                        try:
                            finishedTask.result()
                        except Exception:
                            pass
                # Create a background task to handle this message (allows for asynchronous messaging)
                clientMessageTask = asyncio.create_task(self.handle_message(message))
                clientMessageTasks.add(clientMessageTask)
                clientMessageTask.add_done_callback(
                    clientMessageTasks.discard
                )  # Clean up completed tasks

            # Remove any idle TCP connections
            await self._cleanup_idle_connections()

            # If no message was handled, pause briefly to avoid busy-waiting.
            if not handledMessage:
                await asyncio.sleep(0.001)

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
        # NOTE: If we await here, we block the loop. This is fine if we want to throttle sending to connection speed.
        # But if we want to send fast, we should not await. However, if we don't await, we might spawn too many tasks.
        # TODO: Look into what to do here. Could be optimizations
        if messageTasks:
            _ = await asyncio.gather(*messageTasks, return_exceptions=True)

    # Takes data and sends it passed in server
    async def send_data_async(
        self, serverIP: str, serverPort: int, clientMessageDump: str
    ) -> str:
        try:
            # Get the connection passed in ip and port
            conn = await self._get_or_create_connection(serverIP, serverPort)

            async with conn.lock:  # conn.lock is used to reserve the socket so two threads/tasks don't send data at the same time
                conn.writer.write((clientMessageDump + "\n").encode())
                await conn.writer.drain()

                serverResponseBytes = await asyncio.wait_for(
                    conn.reader.readuntil(b"\n"), timeout=2.0
                )
                conn.lastUsed = time.monotonic()  # Needed for dropping idle connections

            serverResponseDump: str = serverResponseBytes.decode()
            await self.process_client_data(
                clientMessageDump, serverResponseDump, serverIP, serverPort
            )
            return serverResponseDump

        except (
            asyncio.TimeoutError,
            asyncio.IncompleteReadError,
            ConnectionResetError,
            BrokenPipeError,
        ):
            await self._drop_connection(serverIP, serverPort)
            if self.rangeTestEnabled:
                print(
                    f"Timeout error sending data from drone #{self.jsonConfigData.get_self_id()} to #{serverPort - 5000}"
                )
            return "timeout"

        except ConnectionRefusedError as e:
            await self._drop_connection(serverIP, serverPort)
            print(f"ConnectionRefusedError: {e}")
            return str(e)

        except Exception as e:
            await self._drop_connection(serverIP, serverPort)
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
                result: Message = Message.create(
                    id=MessageType.SPEED_TEST_RESPONSE,
                    dronesToSendData=(),
                    data={
                        "target": f"{serverIP}:{serverPort}",
                        "targetId": serverPort
                        - 5000,  # Port is 500* with start being id
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

    # Used to get or create a TCP connection to a specific ip and port
    async def _get_or_create_connection(
        self, serverIP: str, serverPort: int
    ) -> PersistentConnection:
        key = (serverIP, serverPort)
        conn = self.connectionPool.get(key)

        # If connection already exists, return it
        if conn is not None and not conn.writer.is_closing():
            conn.lastUsed = time.monotonic()
            return conn

        # Else, establish a new connection
        reader, writer = await asyncio.wait_for(
            asyncio.open_connection(serverIP, serverPort), timeout=1.0
        )
        conn = PersistentConnection(
            reader=reader,
            writer=writer,
            lock=asyncio.Lock(),
            lastUsed=time.monotonic(),
        )
        self.connectionPool[key] = conn
        return conn  # Return new connections

    # Used to kill a TCP connection to a specific ip and port
    async def _drop_connection(self, serverIP: str, serverPort: int) -> None:
        key = (serverIP, serverPort)
        conn = self.connectionPool.pop(key, None)
        if conn is None:
            return
        conn.writer.close()
        await conn.writer.wait_closed()

    # Drops connections that are closing or have been idle > 30s
    async def _cleanup_idle_connections(self) -> None:
        now = time.monotonic()

        # Only perform cleanup every 5 seconds (saves performance)
        if now - self.lastCleanupTime < 5.0:
            return
        self.lastCleanupTime = now

        keysToClose: list[tuple[str, int]] = []
        for key, conn in self.connectionPool.items():
            # If connection is closing or is over idle time, flag connection to be closed
            if (
                conn.writer.is_closing()
                or (now - conn.lastUsed) > self.connectionIdleTimeoutSec
            ):
                keysToClose.append(key)

        # Close all connections flagged above
        for serverIP, serverPort in keysToClose:
            await self._drop_connection(serverIP, serverPort)

    # Called when program ends to close all connections
    # TODO INTEGRATE THIS WHEN WE CREATE A SHUTDOWN STATE!!!
    async def close_all_connections(self) -> None:
        keys = list(self.connectionPool.keys())
        for serverIP, serverPort in keys:
            await self._drop_connection(serverIP, serverPort)

    # Helper method to run the async client
    def run(self):
        asyncio.run(self.start_client_async())
