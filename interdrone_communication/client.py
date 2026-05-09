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


# Used by connection_pool to store TCP connections to other servers.
@dataclass
class PersistentConnection:
    reader: StreamReader
    writer: StreamWriter
    lock: asyncio.Lock
    last_used: float


class Client:
    # Client Class constructor. Used to pass in JSON Data
    def __init__(
        self,
        flight_settings: FlightSettings,
        client_in_data: Queue[Message],
        range_test_toggle: bool = False,
    ):
        self.flight_settings: FlightSettings = flight_settings
        self.client_in_data: Queue[Message] = client_in_data

        # Check for drone_id from flag in main.py
        self.drone_id: int = flight_settings.current_drone_ID

        # Instantiate other_drones lists
        self.other_drones_ips: list[str] = []
        self.other_drones_ports: list[int] = []
        self.other_drones_ids: tuple[
            int, ...
        ] = ()  # Needs to be a tuple to align with drones_to_send_data in Message class

        # Variables used
        self.connection_pool: dict[tuple[str, int], PersistentConnection] = {}
        self.connection_idle_timeout_sec: float = 30.0
        self.last_cleanup_time: float = time.monotonic()

        # Cap concurrent per message handlers to avoid unbounded task buildup (buildup kills performance).
        self.max_in_flight_message_tasks: int = 6

        # Create a temporary list to update tuple with other_drones_ids
        temp_other_drones_ids: list[int] = list[int](self.other_drones_ids)
        # Loop through other drones to get IPs, Ports, and IDs of drones to connect to
        for drone in flight_settings.other_drone_info:
            self.other_drones_ips.append(str(drone["IP"]))
            self.other_drones_ports.append(int(drone["port"]))
            temp_other_drones_ids.append(drone["id"])
        # Update other_drones_ids tuple with temp_other_drones_ids values
        self.other_drones_ids = tuple[int, ...](temp_other_drones_ids)
        self.range_test_enabled: bool = range_test_toggle

    # Start client code and call the client_loop()
    async def start_client_async(self):
        await self.client_loop()

    # Check for new messages to send and create tasks to send them
    async def client_loop(self) -> None:
        # Keep track of background client_message_tasks
        client_message_tasks: set[asyncio.Task[None]] = set()
        while True:
            handled_message = False
            # Check for new message from client_in_data
            while not self.client_in_data.empty():
                handled_message = True
                # Get new message from client_in_data
                message: Message = await self.client_in_data.get()

                # If too many messages tasks are currently running, wait for one to finish before adding next one
                while len(client_message_tasks) >= self.max_in_flight_message_tasks:
                    done, _ = await asyncio.wait(
                        client_message_tasks,
                        return_when=asyncio.FIRST_COMPLETED,
                    )
                    # Consume task exceptions so they do not get logged as un-retrieved.
                    for finished_task in done:
                        try:
                            finished_task.result()
                        except Exception:
                            pass
                # Create a background task to handle this message (allows for asynchronous messaging)
                client_message_task = asyncio.create_task(self.handle_message(message))
                client_message_tasks.add(client_message_task)
                client_message_task.add_done_callback(
                    client_message_tasks.discard
                )  # Clean up completed tasks

            # Remove any idle TCP connections
            await self._cleanup_idle_connections()

            # If no message was handled, pause briefly to avoid busy-waiting.
            if not handled_message:
                await asyncio.sleep(0.001)

    # Create message_tasks to send data to all other drones
    async def handle_message(self, message: Message):
        # Determine which drones to send message to
        drones_to_send_data: tuple[int, ...] = ()  # NOTE COULD BE SPOT OF ERROR

        # If drones_to_send_data list has id values, only send message to those drones
        send_to_app: bool = False
        send_to_self: bool = False
        if message.drones_to_send_data != ():
            # If you to send data to the app, use ID 0
            if message.drones_to_send_data == (0,):
                send_to_app = True
            elif message.drones_to_send_data == (self.drone_id,):
                send_to_self = True
            else:
                drones_to_send_data = message.drones_to_send_data
        # Else drones_to_send_data list is empty, attempt to send data to all other drones
        else:
            drones_to_send_data = self.other_drones_ids

        # Message Preprocessing
        # Update time value for Network Speed Test message
        if message.id == MessageType.SPEED_TEST_REQUEST:
            message.data["initial_upload_time"] = time.perf_counter()

        # Create message_tasks list to store tasks for all drone connections
        message_tasks: list[asyncio.Task[None]] = []

        # Loop through other_drones_ids and create to task to send message data to them if they're in drones_to_send_data to list
        if send_to_app:
            message_task = asyncio.create_task(
                self.send_data_async(
                    server_ip=self.flight_settings.app_IP,
                    server_port=self.flight_settings.app_port,
                    message=message,
                )
            )
            message_tasks.append(message_task)
        # Allow for messages to be sent to self in some special cases
        elif send_to_self:
            message_task = asyncio.create_task(
                self.send_data_async(
                    server_ip=str(self.flight_settings.get_drone_by_id(self.drone_id)["IP"]),
                    server_port=int(self.flight_settings.get_drone_by_id(self.drone_id)["port"]),
                    message=message,
                )
            )
            message_tasks.append(message_task)
        else:
            for i in range(len(self.other_drones_ids)):
                if self.other_drones_ids[i] in drones_to_send_data:
                    message_task = asyncio.create_task(
                        self.send_data_async(
                            server_ip=self.other_drones_ips[i],
                            server_port=self.other_drones_ports[i],
                            message=message,
                        )
                    )
                    message_tasks.append(message_task)

        # Run all message_tasks concurrently
        if message_tasks:
            _ = await asyncio.gather(*message_tasks, return_exceptions=True)

    # Takes Message and sends it to passed in server
    async def send_data_async(self, server_ip: str, server_port: int, message: Message) -> None:
        try:
            client_message_dump: str = JsonMessageUtilities.message_to_json(message=message)
            # Get the connection passed in ip and port
            conn = await self._get_or_create_connection(server_ip, server_port)

            async with (
                conn.lock
            ):  # conn.lock is used to reserve the socket so two threads/tasks don't send data at the same time
                conn.writer.write((client_message_dump + "\n").encode())
                await conn.writer.drain()

        except (
            asyncio.TimeoutError,
            asyncio.IncompleteReadError,
            ConnectionResetError,
            BrokenPipeError,
            ConnectionRefusedError,
        ):
            await self._drop_connection(server_ip, server_port)
            # Messages that need to be resent if they fail to send
            messages_that_need_resend: set[MessageType] = {
                MessageType.ARM,
                MessageType.ARM_ACK,
                MessageType.ARM_NACK,
                MessageType.DISARM,
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
                    await self.client_in_data.put(
                        Message.create(
                            id=MessageType.PING_NACK,
                            drones_to_send_data=(self.drone_id,),
                            sender_id=(
                                server_port - 5000
                            ),  # NACK is coming from drone it failed to contact
                            data={},
                        )
                    )
                case _ if message.id in messages_that_need_resend:
                    print(
                        f"Failed to send message. drones_to_send_data = {message.drones_to_send_data}"
                    )
                    await self.client_in_data.put(message)
            if self.range_test_enabled:
                print(
                    f"Timeout error sending data from drone #{self.flight_settings.current_drone_ID} to #{server_port - 5000}"
                )

        except Exception as e:
            await self._drop_connection(server_ip, server_port)
            raise Exception(f"Error connecting to {server_ip}:{server_port}: {str(e)}")

    # Used to get or create a TCP connection to a specific ip and port
    async def _get_or_create_connection(
        self, server_ip: str, server_port: int
    ) -> PersistentConnection:
        key = (server_ip, server_port)
        conn = self.connection_pool.get(key)

        # If connection already exists, return it
        if conn is not None and not conn.writer.is_closing():
            conn.last_used = time.monotonic()
            return conn

        # Else, establish a new connection
        reader, writer = await asyncio.wait_for(
            asyncio.open_connection(server_ip, server_port), timeout=1.0
        )
        conn = PersistentConnection(
            reader=reader,
            writer=writer,
            lock=asyncio.Lock(),
            last_used=time.monotonic(),
        )
        self.connection_pool[key] = conn
        return conn  # Return new connections

    # Used to kill a TCP connection to a specific ip and port
    async def _drop_connection(self, server_ip: str, server_port: int) -> None:
        key = (server_ip, server_port)
        conn = self.connection_pool.pop(key, None)
        if conn is None:
            return
        conn.writer.close()
        await conn.writer.wait_closed()

    # Drops connections that are closing or have been idle > 30s
    async def _cleanup_idle_connections(self) -> None:
        now = time.monotonic()

        # Only perform cleanup every 5 seconds (saves performance)
        if now - self.last_cleanup_time < 5.0:
            return
        self.last_cleanup_time = now

        keys_to_close: list[tuple[str, int]] = []
        for key, conn in self.connection_pool.items():
            # If connection is closing or is over idle time, flag connection to be closed
            if (
                conn.writer.is_closing()
                or (now - conn.last_used) > self.connection_idle_timeout_sec
            ):
                keys_to_close.append(key)

        # Close all connections flagged above
        for server_ip, server_port in keys_to_close:
            await self._drop_connection(server_ip, server_port)

    # Called when program ends to close all connections
    # TODO INTEGRATE THIS WHEN WE CREATE A SHUTDOWN STATE!!!
    async def close_all_connections(self) -> None:
        keys = list(self.connection_pool.keys())
        for server_ip, server_port in keys:
            await self._drop_connection(server_ip, server_port)

    # Helper method to run the async client
    def run(self):
        asyncio.run(self.start_client_async())
