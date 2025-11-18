from asyncio.queues import Queue
import asyncio
import sys
from asyncio import StreamReader, StreamWriter
import json
import time

from message_types import MessageData


class Server:
    # Server Class constructor. Used to pass in JSON Data
    def __init__(self, jsonData, serverOutData: Queue[str]):
        self.jsonData = jsonData
        self.serverOutData: Queue[str] = serverOutData

        # Check for sys arg for drone selfId
        self.droneId: str
        try:
            self.droneId = sys.argv[1]
        except Exception:
            self.droneId = jsonData["localInfo"]["selfId"]

    # Async server startup
    async def start_server_async(self):
        server = await asyncio.start_server(
            self.handle_client,
            str(self.jsonData["drones"][self.droneId]["ip"]),
            int(self.jsonData["drones"][self.droneId]["port"]),
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
                message = messageData.decode()
                jsonMessage: MessageData = json.loads(message)
                responseMessage = "Server received your message!"
                # Check for messageId and perform required operations
                match int(jsonMessage["messageId"]):
                    # App test message
                    case 400:
                        print(jsonMessage)
                        await self.serverOutData.put(item="OMG the app contacted us!")
                    case 513:
                        # Set final upload time when server receives
                        jsonMessage["data"]["finalUploadTime"] = time.perf_counter()

                        # Set initial download time when server sends response
                        jsonMessage["data"]["initialDownloadTime"] = time.perf_counter()

                        # Echo back the message with timing data
                        responseMessage = json.dumps(jsonMessage)

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
