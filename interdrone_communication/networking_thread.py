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
        resources_ready: queue.Queue[NetworkingInterface],
        flight_settings: FlightSettings,
        range_test_toggle: bool = False,
    ) -> None:
        loop: AbstractEventLoop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        client_in: AsyncQueue[Message] = asyncio.Queue()
        server_out: AsyncQueue[Message] = asyncio.Queue()

        interface = NetworkingInterface(
            loop=loop,
            client_in=client_in,
            server_out=server_out,
        )

        loop.run_until_complete(
            self.start_networking(
                client_in_data=client_in,
                server_out_data=server_out,
                flight_settings=flight_settings,
                range_test_toggle=range_test_toggle,
                resources_ready=resources_ready,
                interface=interface,
            )
        )
        # Async networking entry point - runs server and client

    async def start_networking(
        self,
        client_in_data: AsyncQueue[Message],
        server_out_data: AsyncQueue[Message],
        flight_settings: FlightSettings,
        resources_ready: queue.Queue[NetworkingInterface],
        interface: NetworkingInterface,
        range_test_toggle: bool = False,
    ) -> None:
        # Instantiate Server and Client
        server_instance = Server(
            flight_settings=flight_settings,
            server_out_data=server_out_data,
            client_in_data=client_in_data,
        )
        client_instance = Client(
            flight_settings=flight_settings,
            client_in_data=client_in_data,
            range_test_toggle=range_test_toggle,
        )

        # Start server and wait until it is bound before signalling the main thread
        server_task = asyncio.create_task(server_instance.start_server_async())
        await server_instance.server_ready.wait()
        resources_ready.put(interface)

        client_task = asyncio.create_task(client_instance.start_client_async())
        print("Server and Client started")

        try:
            # Keep the networking loop alive
            while True:
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            print("Networking shutting down...")
            server_task.cancel()
            client_task.cancel()
