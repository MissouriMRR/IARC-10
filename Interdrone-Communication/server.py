from asyncio.queues import Queue
from json_config_reader import json_config_reader
import asyncio
from asyncio import StreamReader, StreamWriter
import time


from _t_message_types import Message, MessageType
from json_message_utilities import JsonMessageUtilities


class Server:
    # Server Class constructor. Used to pass in JSON Data
    def __init__(
        self,
        jsonConfigData: json_config_reader,
        serverOutData: Queue[Message],
    ):
        self.jsonConfigData: json_config_reader = jsonConfigData
        self.serverOutData: Queue[Message] = serverOutData
        self.serverDefaultResponseMessage: Message = Message.create(
            id=MessageType.SERVER_DEFAULT_RESPONSE,
            dronesToSendData=(),
            data={
                "payload": "Server received your message!",
            },
        )

        # Check for droneId from flag in main.py
        self.droneId: int = jsonConfigData.get_self_id()

    # Async server startup
    async def start_server_async(self):
        server = await asyncio.start_server(
            self.handle_client,
            self.jsonConfigData.get_drone_ip(self.droneId),
            self.jsonConfigData.get_drone_port(self.droneId),
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
            # Read all data from client until end of data char (\n)
            byteMessage = await reader.readuntil(b"\n")
            # Convert Message to json
            message: Message = JsonMessageUtilities.message_from_json(
                byteMessage.decode()
            )

            # If message was read in, begin processing
            if message:
                # IN ORDER TO HAVE THE CLIENT PROCESS A SERVER RESPONSE, YOU MUST OVERWRITE THE responseMessage!!! SEE MessageType.SPEED_TEST_REQUEST FOR AN EXAMPLE
                responseMessage: Message = self.serverDefaultResponseMessage
                # Check for messageId and perform required operations
                match message.id:
                    # App test message
                    case MessageType.APP_TEST:
                        print(message)
                        await self.serverOutData.put(item=message)
                    # App config message
                    case MessageType.APP_CONFIG:
                        # Set json config app fields based on received data here
                        self.jsonConfigData.set_app_ip(newIP=str(message.data["IP"]))
                        self.jsonConfigData.set_app_port(
                            newPort=int(message.data["Port"])
                        )
                    case MessageType.HEARTBEAT:
                        await self.serverOutData.put(item=message)
                    case MessageType.SPEED_TEST_REQUEST:
                        # Set final upload time when server receives
                        # Note: We use perf_counter for high precision. While this value is local to the server, the client will use (initialDownloadTime - finalUploadTime) to calculate server processing time.
                        message.data["finalUploadTime"] = time.perf_counter()

                        # Set initial download time when server sends response
                        message.data["initialDownloadTime"] = time.perf_counter()

                        # Echo back the message with timing data
                        responseMessage = message
                    case _:
                        pass
                # Convert responseMessage to string and send over
                writer.write(
                    (
                        JsonMessageUtilities.message_to_json(responseMessage) + "\n"
                    ).encode()
                )
                await writer.drain()
        except asyncio.TimeoutError:
            print("Client timeout - no data received")
        except asyncio.IncompleteReadError:
            print("Client disconnected before sending complete message")
        except Exception as e:
            print(f"Error handling client: {e}")
        finally:
            # Close the connection
            writer.close()
            await writer.wait_closed()

    # Run server
    def run(self):
        asyncio.run(self.start_server_async())
