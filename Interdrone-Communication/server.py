from asyncio.queues import Queue
from json_config_reader import JsonConfigReader
import asyncio
from asyncio import StreamReader, StreamWriter
import time


from message_types import Message, MessageType
from json_message_utilities import JsonMessageUtilities


class Server:
    # Server Class constructor. Used to pass in JSON Data
    def __init__(
        self,
        jsonConfigData: JsonConfigReader,
        serverOutData: Queue[Message],
    ):
        self.jsonConfigData: JsonConfigReader = jsonConfigData
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

                # IN ORDER TO HAVE THE CLIENT PROCESS A SERVER RESPONSE, YOU MUST OVERWRITE THE responseMessage!!! SEE MessageType.SPEED_TEST_REQUEST FOR AN EXAMPLE
                responseMessage: Message = self.serverDefaultResponseMessage

                # messageSent is set to true in special cases to send a different message early. If it's true, message won't be sent at bottom.
                messageSent = False

                match message.id:
                    case MessageType.APP_TEST:
                        await self.serverOutData.put(item=message)
                    case MessageType.APP_CONFIG:
                        self.jsonConfigData.set_app_ip(newIP=str(message.data["IP"]))
                        self.jsonConfigData.set_app_port(
                            newPort=int(message.data["Port"])
                        )
                    case MessageType.APP_DEBUG:
                        print(message.data["embeddedDebugMessage"])
                        writer.write(
                            (str(message.data["embeddedDebugMessage"]) + "\n").encode()
                        )
                        await writer.drain()
                        messageSent = True

                    case MessageType.HEARTBEAT:
                        await self.serverOutData.put(item=message)
                    case MessageType.SPEED_TEST_REQUEST:
                        message.data["finalUploadTime"] = time.perf_counter()
                        message.data["initialDownloadTime"] = time.perf_counter()
                        responseMessage = message
                    case _:
                        pass
                # Convert responseMessage to string and send over if message hasn't already been sent
                if not messageSent:
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
            writer.close()
            await writer.wait_closed()

    # Run server
    def run(self):
        asyncio.run(self.start_server_async())
