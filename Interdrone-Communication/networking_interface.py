from asyncio.events import AbstractEventLoop
from asyncio.queues import Queue
from asyncio.queues import Queue as AsyncQueue
import asyncio
import concurrent.futures
from typing import Any

from typed_dicts_classes import MessageData


# Thread safe interface to communicate with the async networking layer
class NetworkingInterface:
    def __init__(
        self,
        loop: asyncio.AbstractEventLoop,
        clientIn: AsyncQueue[MessageData],
        clientOut: AsyncQueue[str],
        serverOut: AsyncQueue[str],
    ) -> None:
        self.loop: AbstractEventLoop = loop
        self.clientIn: Queue[MessageData] = clientIn
        self.clientOut: Queue[str] = clientOut
        self.serverOut: Queue[str] = serverOut
        self._clientOutFuture: concurrent.futures.Future[str] | None = None
        self._serverOutFuture: concurrent.futures.Future[str] | None = None

    # Send a message to the client
    def queue_client_message(
        self, message: MessageData, timeout: float | None = None
    ) -> None:
        future = asyncio.run_coroutine_threadsafe(self.clientIn.put(message), self.loop)
        future.result(timeout=timeout)

    # Check if client input queue is empty
    def is_client_in_empty(self) -> bool:
        future = asyncio.run_coroutine_threadsafe(
            self.check_queue_empty(self.clientIn), self.loop
        )
        return future.result(
            timeout=1.0
        )  # May be better to use a enum value here for timeout

    # Used in is_client_in_empty
    async def check_queue_empty(self, q: AsyncQueue[Any]) -> bool:
        return q.empty()

    # Try to get a response from client
    def try_get_client_response(self, timeout: float = 0.0) -> str | None:
        if self._clientOutFuture is None:
            self._clientOutFuture = asyncio.run_coroutine_threadsafe(
                self.clientOut.get(), self.loop
            )
        try:
            result = self._clientOutFuture.result(timeout=timeout)
            self._clientOutFuture = None
            return result
        except concurrent.futures.TimeoutError:
            return None

    # Try to get a message from server
    def try_get_server_message(self, timeout: float = 0.0) -> str | None:
        if self._serverOutFuture is None:
            self._serverOutFuture = asyncio.run_coroutine_threadsafe(
                self.serverOut.get(), self.loop
            )
        try:
            result = self._serverOutFuture.result(timeout=timeout)
            self._serverOutFuture = None
            return result
        except concurrent.futures.TimeoutError:
            return None
