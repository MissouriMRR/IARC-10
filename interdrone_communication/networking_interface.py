from asyncio.events import AbstractEventLoop
from asyncio.queues import Queue
from asyncio.queues import Queue as AsyncQueue
import asyncio
import concurrent.futures

from message_types import Message


# Thread safe interface to communicate with the async networking layer
class NetworkingInterface:
    def __init__(
        self,
        loop: asyncio.AbstractEventLoop,
        clientIn: AsyncQueue[Message],
        clientOut: AsyncQueue[Message],
        serverOut: AsyncQueue[Message],
    ) -> None:
        self.loop: AbstractEventLoop = loop
        self.clientIn: Queue[Message] = clientIn
        self.clientOut: Queue[Message] = clientOut
        self.serverOut: Queue[Message] = serverOut
        self._clientOutFuture: concurrent.futures.Future[Message] | None = None
        self._serverOutFuture: concurrent.futures.Future[Message] | None = None

    # Send a message to the client
    def queue_client_message(self, message: Message, timeout: float | None = None) -> None:
        future = asyncio.run_coroutine_threadsafe(self.clientIn.put(message), self.loop)
        future.result(timeout=timeout)

    # Check if client input queue is empty
    def is_client_in_empty(self) -> bool:
        return self.clientIn.empty()

    # Try to get a response from client
    def try_get_client_response(self, timeout: float = 0.0) -> Message | None:
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
    def try_get_server_message(self, timeout: float = 0.0) -> Message | None:
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

    # Used to empty all queues (used for testing purposes)
    def empty_queues(self, timeout: float | None = 1.0) -> tuple[int, int, int]:
        future = asyncio.run_coroutine_threadsafe(self._empty_queues_async(), self.loop)
        return future.result(timeout=timeout)

    async def _empty_queues_async(self) -> tuple[int, int, int]:
        # Drop any in-flight pending get futures
        if self._clientOutFuture is not None and not self._clientOutFuture.done():
            self._clientOutFuture.cancel()
        self._clientOutFuture = None

        if self._serverOutFuture is not None and not self._serverOutFuture.done():
            self._serverOutFuture.cancel()
        self._serverOutFuture = None

        def drain(q: AsyncQueue[Message]) -> int:
            n = 0
            while True:
                try:
                    q.get_nowait()
                    n += 1
                except asyncio.QueueEmpty:
                    break
            return n

        return (
            drain(self.clientIn),
            drain(self.clientOut),
            drain(self.serverOut),
        )
