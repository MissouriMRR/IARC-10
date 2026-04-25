# Outside Imports
from asyncio.events import AbstractEventLoop
from asyncio.queues import Queue as AsyncQueue
import queue
import asyncio

# Interdrone Imports
from interdrone_communication.message_types import Message
from interdrone_communication.networking_interface import NetworkingInterface
from interdrone_communication.server import Server
from interdrone_communication.client import Client
from state_machine.flight_settings import FlightSettings


# The NetworkingThread class contains methods to start the networking on a seperate thread and generate the NetworkingInterface
class NetworkingThread:
    # Run the async networking stack on its own thread
    def run_networking_thread(
        self,
        resourcesReady: queue.Queue[NetworkingInterface],
        flight_settings: FlightSettings,
        range_test_toggle: bool = False,
    ) -> None:
        loop: AbstractEventLoop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        clientIn: AsyncQueue[Message] = asyncio.Queue()
        serverOut: AsyncQueue[Message] = asyncio.Queue()

        interface = NetworkingInterface(
            loop=loop,
            clientIn=clientIn,
            serverOut=serverOut,
        )

        loop.run_until_complete(
            self.start_networking(
                clientInData=clientIn,
                serverOutData=serverOut,
                flight_settings=flight_settings,
                range_test_toggle=range_test_toggle,
                resourcesReady=resourcesReady,
                interface=interface,
            )
        )
        # Async networking entry point - runs server and client

    async def start_networking(
        self,
        clientInData: AsyncQueue[Message],
        serverOutData: AsyncQueue[Message],
        flight_settings: FlightSettings,
        resourcesReady: queue.Queue[NetworkingInterface],
        interface: NetworkingInterface,
        range_test_toggle: bool = False,
    ) -> None:
        # Instantiate Server and Client
        serverInstance = Server(
            flight_settings=flight_settings,
            serverOutData=serverOutData,
            clientInData=clientInData,
        )
        clientInstance = Client(
            flight_settings=flight_settings,
            clientInData=clientInData,
            range_test_toggle=range_test_toggle,
        )

        # Start server and wait until it is bound before signalling the main thread
        serverTask = asyncio.create_task(serverInstance.start_server_async())
        await serverInstance.server_ready.wait()
        resourcesReady.put(interface)

        clientTask = asyncio.create_task(clientInstance.start_client_async())
        print("Server and Client started")

        try:
            # Keep the networking loop alive
            while True:
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            print("Networking shutting down...")
            serverTask.cancel()
            clientTask.cancel()
