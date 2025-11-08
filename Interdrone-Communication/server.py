from asyncio.queues import Queue
import asyncio
import sys
from asyncio import StreamReader, StreamWriter
import json


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
            clientAddress: str = str(writer.get_extra_info(name="peername"))

            if messageData:
                message = messageData.decode()
                # print(message)
                # TODO implement server response stuff here
                jsonMessage = json.loads(message)
                match int(jsonMessage["messageId"]):
                    case 400:
                        await self.serverOutData.put(item="OMG the app contacted us!")

                # Send default response back to client
                responseMessage = "Server received your message!"
                writer.write((responseMessage + "\n").encode())
                await writer.drain()

                # Store received data in serverOutData to be accessed from main.py
                await self.serverOutData.put(
                    item=(f"clientAddress {clientAddress}, {message} message")
                )
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


"""
def processData():
    match messageId:
        case 402:
            drone.hover()

"""
