# Outside Imports
from asyncio.events import AbstractEventLoop
from asyncio.queues import Queue
from asyncio.queues import Queue as AsyncQueue
import asyncio
import concurrent.futures

# Interdrone Imports
from interdrone_communication.message_types import Message


# Thread safe interface to communicate with the async networking layer
class NetworkingInterface:
    def __init__(
        self,
        loop: asyncio.AbstractEventLoop,
        client_in: AsyncQueue[Message],
        server_out: AsyncQueue[Message],
    ) -> None:
        self.loop: AbstractEventLoop = loop
        self.client_in: Queue[Message] = client_in
        self.server_out: Queue[Message] = server_out
        self._server_out_future: concurrent.futures.Future[Message] | None = None

    # Send a message to the client
    def queue_client_message(self, message: Message, timeout: float | None = None) -> None:
        future = asyncio.run_coroutine_threadsafe(self.client_in.put(message), self.loop)
        future.result(timeout=timeout)

    # Check if client input queue is empty
    def is_client_in_empty(self) -> bool:
        return self.client_in.empty()

    # Try to get a message from server
    def try_get_server_message(self, timeout: float = 0.0) -> Message | None:
        if self._server_out_future is None:
            self._server_out_future = asyncio.run_coroutine_threadsafe(
                self.server_out.get(), self.loop
            )
        try:
            result = self._server_out_future.result(timeout=timeout)
            self._server_out_future = None
            return result
        except concurrent.futures.TimeoutError:
            return None

    # Used to empty all queues (used for testing purposes)
    def empty_queues(self, timeout: float | None = 1.0) -> tuple[int, int]:
        future = asyncio.run_coroutine_threadsafe(self._empty_queues_async(), self.loop)
        return future.result(timeout=timeout)

    async def _empty_queues_async(self) -> tuple[int, int]:
        # Drop any in-flight pending get futures
        if self._server_out_future is not None and not self._server_out_future.done():
            self._server_out_future.cancel()
        self._server_out_future = None

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
            drain(self.client_in),
            drain(self.server_out),
        )
