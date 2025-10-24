from asyncio.queues import Queue
import json
import asyncio
import sys
from asyncio import StreamReader, StreamWriter


class Server:
    # Server Class constructor. Used to pass in JSON Data
    def __init__(self, jsonData, serverOutData: Queue[str]):
        self.jsonData = jsonData
        self.serverOutData: Queue[str] = serverOutData
        self.speedTest: bool = jsonData["localInfo"]["speedTest"]
        self.speedTestKbDataSize: int = jsonData["localInfo"]["speedTestKbDataSize"]

        # Check for sys arg for drone selfId
        try:
            self.droneId: str = sys.argv[1]
        except Exception:
            self.droneId: str = jsonData["localInfo"]["selfId"]

    # Handle individual client connections
    async def handle_client(self, reader: StreamReader, writer: StreamWriter):
        try:
            # Read data from client
            data = await reader.read(4096)

            if data:
                message = data.decode()
                # Send response back to client
                response = "Server received your message!"
                writer.write(response.encode())
                await writer.drain()

                # Store received data in serverOutData to be accessed from main.py
                clientAddress = writer.get_extra_info("peername")
                # Check if this is a speed test message

                # TODO abstract to a handle data function
                try:
                    msgObj = json.loads(message)
                    if isinstance(msgObj, dict) and msgObj.get("speedTest") == True:
                        # For speed tests, echo back the original message
                        writer.write(data)
                        await writer.drain()

                        # Log speed test info
                        await self.serverOutData.put(
                            f"Speed test from drone {msgObj.get('senderId')} ({clientAddress[0]}), size: {len(message) / 1024:.1f}KB"
                        )
                        return
                except json.JSONDecodeError:
                    # Not JSON data, treat as regular message
                    pass
                await self.serverOutData.put(
                    item=(f"clientAddress {clientAddress}, {message} message")
                )

        except Exception as e:
            print(f"Error handling client: {e}")
        finally:
            # Close the connection
            writer.close()
            await writer.wait_closed()

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

    # Run server
    def run(self):
        asyncio.run(self.start_server_async())
