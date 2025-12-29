from asyncio.queues import Queue
from json_config_reader import json_config_reader
import asyncio
from asyncio import StreamReader, StreamWriter
import json
import time
from pathlib import Path


from typed_dicts_classes import MessageData


class Server:
    # Server Class constructor. Used to pass in JSON Data
    def __init__(
        self,
        jsonConfigData: json_config_reader,
        serverOutData: Queue[str],
    ):
        self.jsonConfigData: json_config_reader = jsonConfigData
        self.serverOutData: Queue[str] = serverOutData

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
                message: str = messageData.decode()
                jsonMessage: MessageData = json.loads(message)
                responseMessage = "Server received your message!"
                # Check for messageId and perform required operations
                match jsonMessage["messageId"]:
                    # App test message
                    case 400:
                        print(jsonMessage)
                        await self.serverOutData.put(item="OMG the app contacted us!")
                    # App config message
                    case 401:
                        # Set json config app fields based on received data here
                        self.jsonConfigData.set_app_ip(jsonMessage["data"]["IP"])
                        self.jsonConfigData.set_app_port(jsonMessage["data"]["Port"])
                    # Startup JSON message
                    case 501:
                        # TODO need to finish having this overwrite the json file, catching the change in main, and simplifying logic in main to flow better
                        try:
                            # TODO clean this up with MessageData change to have this output MessageData rather than string
                            config_path = Path(__file__).with_name("config.json")
                            temp_path = config_path.with_suffix(".json.tmp")

                            # Extract payload and parse it as JSON
                            payload_str = jsonMessage["data"]["payload"]
                            config_data = json.loads(payload_str)

                            temp_path.write_text(
                                json.dumps(config_data, indent=4), encoding="utf-8"
                            )
                            temp_path.replace(config_path)
                            jsonMessage["data"]["successfulOverwrite"] = True
                            await self.serverOutData.put(item="JSON Overwritten!")
                            responseMessage = json.dumps(jsonMessage)
                        except Exception as e:
                            print(f"Failed to overwrite JSON file: {e}")
                    case 513:
                        # Set final upload time when server receives
                        # Note: We use perf_counter for high precision. While this value is local to the server, the client will use (initialDownloadTime - finalUploadTime) to calculate server processing time.
                        jsonMessage["data"]["finalUploadTime"] = time.perf_counter()

                        # Set initial download time when server sends response
                        jsonMessage["data"]["initialDownloadTime"] = time.perf_counter()

                        # Echo back the message with timing data
                        responseMessage = json.dumps(jsonMessage)
                    case _:
                        pass
                writer.write((responseMessage + "\n").encode())
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
