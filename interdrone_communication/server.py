# Outside Imports
from asyncio import Queue
import asyncio
from asyncio import StreamReader, StreamWriter
import time

# Interdrone Imports
from interdrone_communication.json_message_utilities import JsonMessageUtilities
from interdrone_communication.message_types import Message, MessageType
from state_machine.flight_settings import FlightSettings


class Server:
    # Server Class constructor. Used to pass in JSON Data
    def __init__(
        self,
        flight_settings: FlightSettings,
        server_out_data: Queue[Message],
        client_in_data: Queue[Message],
    ):
        self.flight_settings: FlightSettings = flight_settings
        self.server_out_data: Queue[Message] = server_out_data
        self.client_in_data: Queue[Message] = client_in_data
        self.server_ready: asyncio.Event = asyncio.Event()

        # Check for drone_id from flag in main.py
        self.drone_id: int = flight_settings.current_drone_ID

    # Async server startup
    async def start_server_async(self):
        server = await asyncio.start_server(
            self.handle_client,
            "0.0.0.0",
            int(self.flight_settings.get_drone_by_id(self.drone_id)["port"]),
        )
        self.server_ready.set()

        try:
            async with server:
                await server.serve_forever()
        except KeyboardInterrupt:
            print("Server shutting down.")
        finally:
            server.close()
            await server.wait_closed()

    # Handle individual client connections
    async def handle_client(self, reader: StreamReader, writer: StreamWriter):
        try:
            while True:
                try:
                    byte_message = await reader.readuntil(b"\n")
                except asyncio.IncompleteReadError:
                    break
                except EOFError:
                    break
                message: Message = JsonMessageUtilities.message_from_json(byte_message.decode())
                # If message was read in, begin processing
                if not message:
                    continue
                # If the received message requires a response inside of server, it's overwritten and sent via client_in_data at the end of handle_client()
                response_message: Message | None = None
                print(f"Message received: {message.id}")
                # Message Handling for all messages sent to the server. Some messages are processed here if they are simple while others are sent back to interdrone to be processed.
                match message.id:
                    case MessageType.APP_CONFIG:
                        self.flight_settings.app_IP = str(message.data["ip"])
                        self.flight_settings.app_port = int(message.data["port"])
                    case MessageType.APP_DEBUG:
                        writer.write((str(message.data["embedded_debug_message"]) + "\n").encode())
                        await writer.drain()
                    case MessageType.HEARTBEAT:
                        await self.server_out_data.put(item=message)
                    case MessageType.SPEED_TEST_REQUEST:
                        final_upload_time = time.perf_counter()
                        response_message = Message.create(
                            id=MessageType.SPEED_TEST_RESPONSE,
                            drones_to_send_data=(message.sender_id,),
                            sender_id=self.flight_settings.current_drone_ID,
                            data={
                                "initial_upload_time": message.data.get("initial_upload_time", 0.0),
                                "final_upload_time": final_upload_time,
                                "initial_download_time": 0.0,
                                "target_id": self.flight_settings.current_drone_ID,
                                "upload_rtt_ms": 0.0,
                                "upload_throughput_kbps": 0.0,
                                "download_rtt_ms": 0.0,
                                "download_throughput_kbps": 0.0,
                                "payload": message.data["payload"],
                            },
                        )  # From here update processing response and then go onto making sure response_message is sent to server
                        response_message.data["initial_download_time"] = time.perf_counter()
                        # Send response message to server
                    # Receive response data, calculate values, and return to server_out_data
                    case MessageType.SPEED_TEST_RESPONSE:
                        # Client receives response - set final download time
                        receive_time = time.perf_counter()

                        if "initial_upload_time" not in message.data:
                            print("SPEED_TEST_RESPONSE missing initial_upload_time, skipping")
                            continue

                        # Calculate Server Processing Time (Delta on Server Clock)
                        server_processing_time: float = float(
                            message.data["initial_download_time"]
                        ) - float(message.data["final_upload_time"])

                        # Calculate Total Round Trip Time (Delta on Client Clock)
                        total_rtt = receive_time - message.data["initial_upload_time"]

                        # Calculate Network RTT (Total - Processing)
                        network_rtt = total_rtt - server_processing_time

                        # Since the server echoes the payload, upload and download sizes are roughly equal,
                        # so we estimate upload and download time as half of the network RTT.
                        estimated_one_way_time = network_rtt / 2

                        upload_time = estimated_one_way_time
                        download_time = estimated_one_way_time

                        upload_size_bytes = len(
                            (JsonMessageUtilities.message_to_json(message=message)).encode("utf-8")
                        )  # TODO change this actual uploaded message (difference is negligible)
                        upload_throughput_kbps = (
                            (upload_size_bytes * 8 / 1000) / upload_time
                            if upload_time > 0
                            else float("inf")
                        )

                        download_size_bytes = len(
                            (JsonMessageUtilities.message_to_json(message=message)).encode("utf-8")
                        )
                        download_throughput_kbps = (
                            (download_size_bytes * 8 / 1000) / download_time
                            if download_time > 0
                            else float("inf")
                        )

                        message.data["upload_rtt_ms"] = round(upload_time * 1000, 2)
                        message.data["upload_throughput_kbps"] = round(upload_throughput_kbps, 2)
                        message.data["download_rtt_ms"] = round(download_time * 1000, 2)
                        message.data["download_throughput_kbps"] = round(download_throughput_kbps, 2)
                        await self.server_out_data.put(item=message)
                    case MessageType.PING:
                        # Respond to ping with PING_ACK
                        response_message = Message.create(
                            id=MessageType.PING_ACK,
                            drones_to_send_data=(message.sender_id,),
                            sender_id=(self.flight_settings.current_drone_ID),
                            data={},
                        )
                    # If message doesn't need specific handling here, just pass out to server_out_data
                    case _:
                        await self.server_out_data.put(item=message)

                # If response_message was overwritten, send the response
                if response_message is not None:
                    await self.client_in_data.put(response_message)

        except asyncio.TimeoutError:
            print("Client timeout - no data received")
        except asyncio.IncompleteReadError:
            print("Client disconnected before sending complete message")
        except Exception as e:
            print(f"Error handling client: {e}")
        finally:
            writer.close()
            try:
                await writer.wait_closed()
            except ConnectionResetError:
                pass

    # Run server
    def run(self):
        asyncio.run(self.start_server_async())
