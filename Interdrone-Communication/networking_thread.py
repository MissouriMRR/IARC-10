from asyncio.events import AbstractEventLoop
from asyncio.queues import Queue as AsyncQueue
import queue
import asyncio

from typed_dicts_classes import MessageData
from json_config_reader import json_config_reader
from networking_interface import NetworkingInterface
import server
import client


# The NetworkingThread class contains methods to start the networking on a seperate thread and generate the NetworkingInterface
class NetworkingThread:
    # Run the async networking stack on its own thread
    def run_networking_thread(
        self,
        resourcesReady: queue.Queue[NetworkingInterface],
        jsonConfigData: json_config_reader,
    ) -> None:
        loop: AbstractEventLoop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        clientIn: AsyncQueue[MessageData] = asyncio.Queue()
        clientOut: AsyncQueue[MessageData] = asyncio.Queue()
        serverOut: AsyncQueue[MessageData] = asyncio.Queue()

        # Provide interface to main thread
        resourcesReady.put(
            NetworkingInterface(
                loop=loop,
                clientIn=clientIn,
                clientOut=clientOut,
                serverOut=serverOut,
            )
        )

        loop.run_until_complete(
            self.start_networking(
                clientInData=clientIn,
                clientOutData=clientOut,
                serverOutData=serverOut,
                jsonConfigData=jsonConfigData,
            )
        )
        # Async networking entry point - runs server and client

    async def start_networking(
        self,
        clientInData: AsyncQueue[MessageData],
        clientOutData: AsyncQueue[MessageData],
        serverOutData: AsyncQueue[MessageData],
        jsonConfigData: json_config_reader,
    ) -> None:
        # Instantiate Server and Client
        serverInstance = server.Server(
            jsonConfigData=jsonConfigData, serverOutData=serverOutData
        )
        clientInstance = client.Client(
            jsonConfigData=jsonConfigData,
            clientInData=clientInData,
            clientOutData=clientOutData,
        )

        # Run both concurrently
        serverTask = asyncio.create_task(serverInstance.start_server_async())
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
