from asyncio.queues import Queue


import asyncio
import sys


class Client:
    # Client Class constructor. Used to pass in JSON Data
    def __init__(self, jsonData, clientOutData: Queue[str]):
        self.jsonData = jsonData
        self.clientOutData: Queue[str] = clientOutData

        # Check for sys arg for drone selfId
        try:
            self.droneId: str = sys.argv[1]
        except Exception:
            self.droneId: str = jsonData["localInfo"]["selfId"]

        # Instantiate otherDrones lists
        self.otherDronesIps: list[str] = []
        self.otherDronesPorts: list[int] = []

        # Parse JSON to get IPs and Ports of drones to connect to
        # Loop through drones 1-4
        for i in range(1, 5):
            # If drone is self (drone running this script) don't add them otherDrones list
            if i != int(self.droneId):
                # Add other drones IP and Ports to their respective lists
                self.otherDronesIps.append(
                    self.jsonData["drones"][str(i)]["ip"]
                )  # This will be simplified after JSON parser is implemented
                self.otherDronesPorts.append(
                    int(self.jsonData["drones"][str(i)]["port"])
                )
        print(self.otherDronesIps)
        print(self.otherDronesPorts)

    # Start client and try to send messages to servers using asyncio
    async def start_client(self):
        while True:
            # Create coroutines for all drone connections
            coroutines = []
            for i in range(len(self.otherDronesIps)):
                coroutine = self.send_data_async(
                    serverIP=self.otherDronesIps[i], serverPort=self.otherDronesPorts[i]
                )
                coroutines.append(coroutine)

            # Run all coroutines concurrently
            results = await asyncio.gather(*coroutines, return_exceptions=True)

            # Process results
            for i, result in enumerate(results):
                if isinstance(result, Exception):
                    pass
                    # print(f"Connection to {self.otherDronesIps[i]} failed: {result}")
                else:
                    pass
                    # print(
                    #     f"Successfully communicated with {self.otherDronesIps[i]}: {result}"
                    # )

            # Wait 1 second before next iteration
            await asyncio.sleep(1)

    # Async coroutine to send data to one server
    # TODO add a data param
    async def send_data_async(self, serverIP: str, serverPort: int):
        try:
            # Open async connection with timeout
            reader, writer = await asyncio.wait_for(
                asyncio.open_connection(serverIP, serverPort), timeout=1.0
            )

            # Send data
            writer.write(b"Hello, server!")
            await writer.drain()

            # Receive response
            data = await asyncio.wait_for(reader.read(1024), timeout=1.0)

            # Add response data to clientOutData queue
            await self.clientOutData.put(item=str(data))

            # Close connection
            writer.close()
            await writer.wait_closed()

            return data.decode()

        except asyncio.TimeoutError:
            raise Exception(f"Timeout connecting to {serverIP}:{serverPort}")
        except ConnectionRefusedError:
            raise Exception(f"Connection refused by {serverIP}:{serverPort}")
        except Exception as e:
            raise Exception(f"Error connecting to {serverIP}:{serverPort}: {str(e)}")

    # Helper method to run the async client
    def run(self):
        asyncio.run(self.start_client())
