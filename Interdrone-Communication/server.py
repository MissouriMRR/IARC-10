from asyncio.queues import Queue
from json_config_reader import json_config_reader
import asyncio
from asyncio import StreamReader, StreamWriter
import json
import time


from typed_dicts_classes import MessageData


class Server:
    # Server Class constructor. Used to pass in JSON Data
    def __init__(
        self,
        jsonConfigData: json_config_reader,
        serverOutData: Queue[MessageData],
    ):
        self.jsonConfigData: json_config_reader = jsonConfigData
        self.serverOutData: Queue[MessageData] = serverOutData
        self.serverDefaultResponseMessage: MessageData = {
            "messageId": 505,
            "dronesToSendData": [],
            "data": {
                "payload": "Server received your message!",
            },
        }

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
            messageData = await reader.readuntil(b"\n")
            if messageData:
                strMessage: str = messageData.decode()
                jsonMessage: MessageData = json.loads(strMessage)

                # IN ORDER TO HAVE THE CLIENT PROCESS A SERVER RESPONSE, YOU MUST OVERWRITE THE responseMessage!!! SEE MESSAGE 513 FOR AN EXAMPLE
                responseMessage: MessageData = self.serverDefaultResponseMessage
                # Check for messageId and perform required operations
                match jsonMessage["messageId"]:
                    # App test message
                    case 400:
                        print(jsonMessage)
                        await self.serverOutData.put(item=jsonMessage)
                    # App config message
                    case 401:
                        # Set json config app fields based on received data here
                        self.jsonConfigData.set_app_ip(jsonMessage["data"]["IP"])
                        self.jsonConfigData.set_app_port(jsonMessage["data"]["Port"])
                    case 504:
                        await self.serverOutData.put(item=jsonMessage)
                    case 513:
                        # Set final upload time when server receives
                        # Note: We use perf_counter for high precision. While this value is local to the server, the client will use (initialDownloadTime - finalUploadTime) to calculate server processing time.
                        jsonMessage["data"]["finalUploadTime"] = time.perf_counter()

                        # Set initial download time when server sends response
                        jsonMessage["data"]["initialDownloadTime"] = time.perf_counter()

                        # Echo back the message with timing data
                        responseMessage = jsonMessage
                    case _:
                        pass
                # Convert responseMessage to string and send over
                writer.write((json.dumps(responseMessage) + "\n").encode())
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
