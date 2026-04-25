# Outside Imports
from asyncio.queues import Queue
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
        serverOutData: Queue[Message],
        clientInData: Queue[Message],
    ):
        self.flight_settings: FlightSettings = flight_settings
        self.serverOutData: Queue[Message] = serverOutData
        self.clientInData: Queue[Message] = clientInData

        # Check for droneId from flag in main.py
        self.droneId: int = flight_settings.current_drone_ID

    # Async server startup
    async def start_server_async(self):
        server = await asyncio.start_server(
            self.handle_client,
            "0.0.0.0",
            int(self.flight_settings.get_drone_by_id(self.droneId)["port"]),
        )

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
                    byteMessage = await reader.readuntil(b"\n")
                except asyncio.IncompleteReadError:
                    break
                except EOFError:
                    break
                message: Message = JsonMessageUtilities.message_from_json(
                    byteMessage.decode()
                )
                # If message was read in, begin processing
                if not message:
                    continue

                # If the received message requires a response inside of server, it's overwritten and sent via clientInData at the end of handle_client()
                responseMessage: Message | None = None
                print(f"Message received: {message.id}")
                # Message Handling for all messages sent to the server. Some messages are processed here if they are simple while others are sent back to interdrone to be processed.
                match message.id:
                    case MessageType.APP_CONFIG:
                        self.flight_settings.app_IP = str(message.data["IP"])
                        self.flight_settings.app_port = int(message.data["Port"])
                    case MessageType.APP_DEBUG:
                        writer.write(
                            (str(message.data["embeddedDebugMessage"]) + "\n").encode()
                        )
                        await writer.drain()
                    case MessageType.REQUEST_DRONE_LOCATIONS:
                        # Send back response message with two drones locations
                        # NOTE in the future we will need to have state that fetches drone location to fill in the data here
                        # This is temporary for app testing
                        responseMessage = Message.create(
                            id=MessageType.SEND_DRONE_LOCATIONS,
                            dronesToSendData=(message.senderId,),
                            senderId=self.flight_settings.current_drone_ID,
                            # TODO FIGURE OUT PATHFINDINGS COORD SYSTEM (please write docs)
                            data={
                                "drone1Data": {
                                    "latLong": [37.9586040775280, -91.771233861919],
                                    "xYCoords": [100, 10],
                                },
                                "drone2Data": {
                                    "latLong": [37.9586654649470, -91.772145189968],
                                    "xYCoords": [10, 250],
                                },
                            },
                        )
                        print("Sending drone location response to app")
                    case MessageType.HEARTBEAT:
                        await self.serverOutData.put(item=message)
                    case MessageType.SPEED_TEST_REQUEST:
                        finalUploadTime = time.perf_counter()
                        responseMessage = Message.create(
                            id=MessageType.SPEED_TEST_RESPONSE,
                            dronesToSendData=(message.senderId,),
                            senderId=self.flight_settings.current_drone_ID,
                            data={
                                "initialUploadTime": message.data.get(
                                    "initialUploadTime", 0.0
                                ),
                                "finalUploadTime": finalUploadTime,
                                "initialDownloadTime": 0.0,
                                "targetId": self.flight_settings.current_drone_ID,
                                "uploadRttMs": 0.0,
                                "uploadThroughputKbps": 0.0,
                                "downloadRttMs": 0.0,
                                "downloadThroughputKbps": 0.0,
                                "payload": message.data["payload"],
                            },
                        )  # From here update processing response and then go onto making sure responseMessage is sent to server
                        responseMessage.data["initialDownloadTime"] = (
                            time.perf_counter()
                        )
                        # Send response message to server
                    # Receive response data, calculate values, and return to serverOutData
                    case MessageType.SPEED_TEST_RESPONSE:
                        # Client receives response - set final download time
                        receiveTime = time.perf_counter()

                        if "initialUploadTime" not in message.data:
                            print(
                                "SPEED_TEST_RESPONSE missing initialUploadTime, skipping"
                            )
                            continue

                        # Calculate Server Processing Time (Delta on Server Clock)
                        serverProcessingTime: float = float(
                            message.data["initialDownloadTime"]
                        ) - float(message.data["finalUploadTime"])

                        # Calculate Total Round Trip Time (Delta on Client Clock)
                        totalRtt = receiveTime - message.data["initialUploadTime"]

                        # Calculate Network RTT (Total - Processing)
                        networkRtt = totalRtt - serverProcessingTime

                        # Since the server echoes the payload, upload and download sizes are roughly equal,
                        # so we estimate upload and download time as half of the network RTT.
                        estimatedOneWayTime = networkRtt / 2

                        uploadTime = estimatedOneWayTime
                        downloadTime = estimatedOneWayTime

                        uploadSizeBytes = len(
                            (
                                JsonMessageUtilities.message_to_json(message=message)
                            ).encode("utf-8")
                        )  # TODO change this actual uploaded message (difference is negligible)
                        uploadThroughputKbps = (
                            (uploadSizeBytes * 8 / 1000) / uploadTime
                            if uploadTime > 0
                            else float("inf")
                        )

                        downloadSizeBytes = len(
                            (
                                JsonMessageUtilities.message_to_json(message=message)
                            ).encode("utf-8")
                        )
                        downloadThroughputKbps = (
                            (downloadSizeBytes * 8 / 1000) / downloadTime
                            if downloadTime > 0
                            else float("inf")
                        )

                        message.data["uploadRttMs"] = round(uploadTime * 1000, 2)
                        message.data["uploadThroughputKbps"] = round(
                            uploadThroughputKbps, 2
                        )
                        message.data["downloadRttMs"] = round(downloadTime * 1000, 2)
                        message.data["downloadThroughputKbps"] = round(
                            downloadThroughputKbps, 2
                        )
                        await self.serverOutData.put(item=message)
                    case MessageType.PING:
                        # Respond to ping with PING_ACK
                        responseMessage = Message.create(
                            id=MessageType.PING_ACK,
                            dronesToSendData=(message.senderId,),
                            senderId=(self.flight_settings.current_drone_ID),
                            data={},
                        )
                    # If message doesn't need specific handling here, just pass out to serverOutData
                    case _:
                        await self.serverOutData.put(item=message)

                # If responseMessage was overwritten, send the response
                if responseMessage is not None:
                    await self.clientInData.put(responseMessage)

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
