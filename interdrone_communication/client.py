# Outside Imports
from asyncio.queues import Queue
from asyncio import StreamReader, StreamWriter
from dataclasses import dataclass
import time
import asyncio

# Interdrone Imports
from interdrone_communication.message_types import Message, MessageType
from interdrone_communication.json_message_utilities import JsonMessageUtilities
from state_machine.flight_settings import FlightSettings


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
        flight_settings: FlightSettings,
        clientInData: Queue[Message],
        range_test_toggle: bool = False,
    ):
        self.flight_settings: FlightSettings = flight_settings
        self.clientInData: Queue[Message] = clientInData

        # Check for droneId from flag in main.py
        self.droneId: int = flight_settings.current_drone_ID

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
        # Loop through other drones to get IPs, Ports, and IDs of drones to connect to
        for drone in flight_settings.other_drone_info:
            self.otherDronesIps.append(str(drone["IP"]))
            self.otherDronesPorts.append(int(drone["port"]))
            tempOtherDronesIds.append(drone["id"])
        # Update otherDronesIds tuple with tempOtherDronesIds values
        self.otherDronesIds = tuple[int, ...](tempOtherDronesIds)
        self.rangeTestEnabled: bool = range_test_toggle

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
        sendToSelf: bool = False
        if message.dronesToSendData != ():
            # If you to send data to the app, use ID 0
            if message.dronesToSendData == (0,):
                sendToApp = True
            elif message.dronesToSendData == (self.droneId,):
                sendToSelf = True
            else:
                dronesToSendData = message.dronesToSendData
        # Else dronesToSendData list is empty, attempt to send data to all other drones
        else:
            dronesToSendData = self.otherDronesIds

        # Message Preprocessing
        # Update time value for Network Speed Test message
        if message.id == MessageType.SPEED_TEST_REQUEST:
            message.data["initialUploadTime"] = time.perf_counter()

        # Create messageTasks list to store tasks for all drone connections
        messageTasks: list[asyncio.Task[None]] = []

        # Loop through otherDronesIds and create to task to send message data to them if they're in dronesToSendData to list
        if sendToApp:
            messageTask = asyncio.create_task(
                self.send_data_async(
                    serverIP=self.flight_settings.app_IP,
                    serverPort=self.flight_settings.app_port,
                    message=message,
                )
            )
            messageTasks.append(messageTask)
        # Allow for messages to be sent to self in some special cases
        elif sendToSelf:
            messageTask = asyncio.create_task(
                self.send_data_async(
                    serverIP=str(self.flight_settings.get_drone_by_id(self.droneId)["IP"]),
                    serverPort=int(self.flight_settings.get_drone_by_id(self.droneId)["port"]),
                    message=message,
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
                            message=message,
                        )
                    )
                    messageTasks.append(messageTask)

        # Run all messageTasks concurrently
        if messageTasks:
            _ = await asyncio.gather(*messageTasks, return_exceptions=True)

    # Takes Message and sends it to passed in server
    async def send_data_async(self, serverIP: str, serverPort: int, message: Message) -> None:
        try:
            clientMessageDump: str = JsonMessageUtilities.message_to_json(message=message)
            # Get the connection passed in ip and port
            conn = await self._get_or_create_connection(serverIP, serverPort)

            async with (
                conn.lock
            ):  # conn.lock is used to reserve the socket so two threads/tasks don't send data at the same time
                conn.writer.write((clientMessageDump + "\n").encode())
                await conn.writer.drain()

        except (
            asyncio.TimeoutError,
            asyncio.IncompleteReadError,
            ConnectionResetError,
            BrokenPipeError,
            ConnectionRefusedError,
        ):
            await self._drop_connection(serverIP, serverPort)
            # Messages that need to be resent if they fail to send
            messages_that_need_resend: set[MessageType] = {
                MessageType.ARM,
                MessageType.ARM_ACK,
                MessageType.ARM_NACK,
                MessageType.START_DEMO,
                MessageType.START_DEMO_ACK,
                MessageType.DEMO_DONE,
                MessageType.START_MISSION,
                MessageType.START_MISSION_ACK,
                MessageType.REACHED_WAYPOINT,
                MessageType.REACHED_WAYPOINT_ACK,
                MessageType.RECONFIRM_WAYPOINTS,
                MessageType.EMERGENCY_LAND,
                MessageType.LAND,
            }
            match message.id:
                # If ping message failed to send, send a PING_NACK to self server
                case MessageType.PING:
                    await self.clientInData.put(
                        Message.create(
                            id=MessageType.PING_NACK,
                            dronesToSendData=(self.droneId,),
                            senderId=(
                                serverPort - 5000
                            ),  # NACK is coming from drone it failed to contact
                            data={},
                        )
                    )
                case _ if message.id in messages_that_need_resend:
                    print(f"Failed to send message. dronesToSendData = {message.dronesToSendData}")
                    await self.clientInData.put(message)
            if self.rangeTestEnabled:
                print(
                    f"Timeout error sending data from drone #{self.flight_settings.current_drone_ID} to #{serverPort - 5000}"
                )

        except Exception as e:
            await self._drop_connection(serverIP, serverPort)
            raise Exception(f"Error connecting to {serverIP}:{serverPort}: {str(e)}")

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
            if conn.writer.is_closing() or (now - conn.lastUsed) > self.connectionIdleTimeoutSec:
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
