import asyncio
import sys


class Server:
    # Server Class constructor. Used to pass in JSON Data
    def __init__(self, jsonData):
        self.jsonData = jsonData
        # TODO REMOVE FOR BETTER TESTING METHOD
        self.droneId: str = sys.argv[1]
        # self.droneId: str = jsonData["localInfo"]["selfId"]

    # Handle individual client connections
    async def handle_client(self, reader, writer):
        try:
            # Read data from client
            data = await reader.read(1024)

            if data:
                message = data.decode()
                # print(f"Received from {client_address}: {message}")

                # Send response back to client
                response = "Server received your message!"
                writer.write(response.encode())
                await writer.drain()

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
