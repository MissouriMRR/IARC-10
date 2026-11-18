"""
App communications test: checks that commands from the app reach drone 1, get
distributed to the rest of the swarm, and that the acks/replies make it back.

Run it on every drone in the mission, the same way as networking_main.py:

    python app_coms_test.py -i 1 [--fake-app]
    python app_coms_test.py -i 2
    ...

(from tests/, or as tests/app_coms_test.py from the repo root). --config defaults
to the repo root's mission_config.json either way.

No flight controller is used. Commands that would move the drone (LAND,
EMERGENCY_LAND, DISARM) are only received and relayed, never executed.

Every drone prints the app-related messages it receives and sends. Drone 1 also
prints a per-peer result line a few seconds after each app command arrives, and
the full summary table on exit (Ctrl+C, or when the --fake-app sequence ends).

Without --fake-app, drone 1 waits for the real app. With --fake-app, drone 1
plays the app itself: it sends APP_CONFIG pointing at a local listener, then a
scripted sequence of commands to its own server port.
"""

import argparse
import asyncio
import os
import sys
import time
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Any

# Run as a plain script from anywhere (tests/ or the repo root), so the repo root
# has to be put on the path for the project imports below.
_REPO_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_DIR not in sys.path:
    sys.path.insert(0, _REPO_DIR)

from interdrone_communication.json_message_utilities import JsonMessageUtilities  # noqa: E402
from interdrone_communication.message_types import Message, MessageType  # noqa: E402
from state_machine.drone import Drone  # noqa: E402
from state_machine.flight_settings import FlightSettings  # noqa: E402
from state_machine.interdrone import CMD_MSG, Interdrone  # noqa: E402

APP_ID = 0
FAKE_APP_PORT = 5050
# Seconds drone 1 waits after an app command before checking the result.
RESULT_WINDOW_SEC = 5.0
# Ground truth averages 10 position samples 0.5 s apart before it acks.
GROUND_TRUTH_WINDOW_SEC = 10.0

# Too frequent to be worth printing.
QUIET_MESSAGES = {MessageType.PING, MessageType.PING_ACK, MessageType.HEARTBEAT}

# App command -> the ack each peer sends back to drone 1.
ACKED_COMMANDS = {
    MessageType.ARM: MessageType.ARM_ACK,
    MessageType.START_TAKEOFF: MessageType.START_TAKEOFF_ACK,
    MessageType.START_DEMO: MessageType.START_DEMO_ACK,
    MessageType.START_MISSION: MessageType.START_MISSION_ACK,
}
# App commands with no ack: drone 1 is only checked for relaying them to each peer.
RELAYED_COMMANDS = {
    MessageType.DEMO_DONE,
    MessageType.LAND,
    MessageType.EMERGENCY_LAND,
    MessageType.DISARM,
}


class _NoVehicle:
    """Just enough of a dronekit Vehicle for the interdrone handlers to run against."""

    armed = False
    parameters: dict[str, float] = {}
    location = SimpleNamespace(global_relative_frame=SimpleNamespace(lat=0.0, lon=0.0))


@dataclass
class Event:
    """One message seen by this drone's networking, in either direction."""

    time: float
    outgoing: bool
    message: Message


@dataclass
class CommandResult:
    """What came of one app command, keyed by "drone N" or "app"."""

    name: str
    outcomes: dict[str, str] = field(default_factory=dict)

    def __str__(self) -> str:
        return f"{self.name}: " + ", ".join(f"{k}: {v}" for k, v in self.outcomes.items())


def describe(message: Message) -> str:
    """One-line, human-readable form of a message for the live log."""
    text = f"{message.id.name} from {message.sender_id} to {list(message.drones_to_send_data)}"
    return f"{text} {message.data}" if message.data else text


class AppCommandChecker:
    """
    Logs every message this drone's networking reads or sends and, on drone 1,
    checks what came of each command from the app once its result window closes.
    """

    def __init__(self, interdrone: Interdrone) -> None:
        self.interdrone = interdrone
        self.self_id: int = interdrone.flight_settings.current_drone_ID
        self.peers: list[int] = list(interdrone.flight_settings.other_drones_in_mission)
        self.events: list[Event] = []
        self.results: list[CommandResult] = []
        self.checks_in_progress: set[asyncio.Task] = set()
        self._wrap_networking()

    def _wrap_networking(self) -> None:
        """Route the networking interface's reads and sends through _on_message."""
        networking = self.interdrone.networking
        original_read = networking.try_get_server_message
        original_send = networking.queue_client_message

        def read(timeout: float = 0.0) -> Message | None:
            message = original_read(timeout=timeout)
            if message is not None:
                self._on_message(message, outgoing=False)
            return message

        def send(message: Message, timeout: float | None = None) -> None:
            self._on_message(message, outgoing=True)
            original_send(message, timeout)

        networking.try_get_server_message = read
        networking.queue_client_message = send

    def _on_message(self, message: Message, outgoing: bool) -> None:
        event = Event(time.time(), outgoing, message)
        self.events.append(event)
        if message.id not in QUIET_MESSAGES:
            print(f"[drone {self.self_id}] {'SEND' if outgoing else 'RECV'} {describe(message)}")
        if self.self_id == 1 and not outgoing and message.sender_id == APP_ID:
            task = asyncio.get_running_loop().create_task(self._check_command(event))
            self.checks_in_progress.add(task)
            task.add_done_callback(self.checks_in_progress.discard)

    def _messages_since(
        self, start: float, outgoing: bool, message_id: MessageType
    ) -> list[Message]:
        return [
            e.message
            for e in self.events
            if e.time >= start and e.outgoing == outgoing and e.message.id == message_id
        ]

    def _relayed_to(self, start: float, message_id: MessageType) -> set[int]:
        """Drones this drone has sent message_id to since start."""
        return {
            drone_id
            for m in self._messages_since(start, True, message_id)
            for drone_id in m.drones_to_send_data
        }

    def _sent_to_app(self, start: float, message_id: MessageType) -> list[Message]:
        """Every message_id this drone has sent to the app since start."""
        return [
            m
            for m in self._messages_since(start, True, message_id)
            if m.drones_to_send_data == (APP_ID,)
        ]

    async def _check_command(self, event: Event) -> None:
        command = event.message
        result = CommandResult(command.id.name)
        start = event.time

        if command.id == MessageType.SEND_GROUND_TRUTH_COORDS:
            result.name += f" (target {list(command.drones_to_send_data)})"
            await asyncio.sleep(GROUND_TRUTH_WINDOW_SEC)
        else:
            await asyncio.sleep(RESULT_WINDOW_SEC)

        if command.id in ACKED_COMMANDS:
            self._check_acked(command, start, result)
        elif command.id in RELAYED_COMMANDS:
            self._check_relayed(command, start, result)
        elif command.id == MessageType.REQUEST_SWARM_STATUS:
            self._check_swarm_status(start, result)
        elif command.id == MessageType.SEND_GROUND_TRUTH_COORDS:
            self._check_ground_truth(command, start, result)
        elif command.id == MessageType.COMMAND_OFFSET_DISTRIBUTION:
            self._check_offset_distribution(start, result)
        else:
            result.outcomes["-"] = "not checked by this test"

        self.results.append(result)
        print(f"[drone 1] RESULT {result}")

    def _add_own_cmd_msg(self, result: CommandResult) -> None:
        result.outcomes[f"drone {self.self_id}"] = f"cmd_msg={self.interdrone.cmd_msg.name}"

    def _check_acked(self, command: Message, start: float, result: CommandResult) -> None:
        ack_id = ACKED_COMMANDS[command.id]
        relayed = self._relayed_to(start, command.id)
        acked = {m.sender_id for m in self._messages_since(start, False, ack_id)}
        for peer in self.peers:
            if peer in acked:
                outcome = "OK"
            elif peer in relayed:
                outcome = f"relayed, no {ack_id.name}"
            else:
                outcome = "NOT RELAYED"
            result.outcomes[f"drone {peer}"] = outcome
        self._add_own_cmd_msg(result)

    def _check_relayed(self, command: Message, start: float, result: CommandResult) -> None:
        relayed = self._relayed_to(start, command.id)
        for peer in self.peers:
            result.outcomes[f"drone {peer}"] = "relayed" if peer in relayed else "NOT RELAYED"
        self._add_own_cmd_msg(result)

    def _check_swarm_status(self, start: float, result: CommandResult) -> None:
        replies = self._sent_to_app(start, MessageType.SEND_SWARM_STATUS)
        if not replies:
            result.outcomes["app"] = "NO SEND_SWARM_STATUS"
            return
        for status in replies[0].data.get("drone_status", []):
            result.outcomes[f"drone {status['drone_id']}"] = (
                f"cmd_msg={CMD_MSG(status['drone_cmd_msg']).name}"
                if status["drone_available"]
                else "UNAVAILABLE"
            )

    def _check_ground_truth(self, command: Message, start: float, result: CommandResult) -> None:
        acks = self._sent_to_app(start, MessageType.SEND_GROUND_TRUTH_COORDS_ACK)
        acked = {m.data.get("calculating_drone_id") for m in acks}
        for target in command.drones_to_send_data:
            result.outcomes[f"drone {target}"] = "OK" if target in acked else "NO ACK TO APP"

    def _check_offset_distribution(self, start: float, result: CommandResult) -> None:
        replies = self._sent_to_app(start, MessageType.COMMAND_OFFSET_DISTRIBUTION_ACK)
        if not replies:
            result.outcomes["app"] = "NO COMMAND_OFFSET_DISTRIBUTION_ACK"
            return
        for entry in replies[0].data.get("drone_ack_list", []):
            result.outcomes[f"drone {entry['drone_id']}"] = "OK" if entry["acked"] else "NOT ACKED"

    async def print_summary(self) -> None:
        """Wait for commands still being checked, then print every result (drone 1 only)."""
        if self.checks_in_progress:
            await asyncio.wait(set(self.checks_in_progress))
        if self.self_id != 1:
            return
        print("\n===== APP COMS SUMMARY =====")
        if not self.results:
            print("No app commands received.")
        for result in self.results:
            print(result)
        print("============================\n")


async def fake_app(flight_settings: FlightSettings) -> None:
    """Plays the app against drone 1: listens for replies and sends a scripted sequence."""

    async def handle_reply(reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        try:
            while line := await reader.readline():
                reply = JsonMessageUtilities.message_from_json(line.decode())
                print(f"[fake app] RECV {describe(reply)}")
        finally:
            writer.close()

    listener = await asyncio.start_server(handle_reply, "0.0.0.0", FAKE_APP_PORT)
    drone_1_port = int(flight_settings.get_drone_by_id(1)["port"])
    _, writer = await asyncio.open_connection("127.0.0.1", drone_1_port)

    async def send(
        message_id: MessageType,
        data: dict[str, Any] | None = None,
        to: tuple[int, ...] = (1,),
        wait: float = RESULT_WINDOW_SEC + 1,
    ) -> None:
        """Send one app message to drone 1, then give it `wait` seconds to play out."""
        message = Message.create(
            id=message_id, drones_to_send_data=to, sender_id=APP_ID, data=data or {}
        )
        print(f"[fake app] SEND {describe(message)}")
        writer.write((JsonMessageUtilities.message_to_json(message) + "\n").encode())
        await writer.drain()
        await asyncio.sleep(wait)

    await send(MessageType.APP_CONFIG, {"ip": "127.0.0.1", "port": FAKE_APP_PORT}, wait=0.5)
    # DEMO and MISSION are only accepted while cmd_msg is ARM, so re-arm before each.
    await send(MessageType.ARM)
    await send(MessageType.START_TAKEOFF)
    await send(MessageType.ARM)
    await send(MessageType.START_DEMO)
    await send(MessageType.DEMO_DONE)
    await send(MessageType.ARM)
    await send(MessageType.START_MISSION)
    await send(MessageType.REQUEST_SWARM_STATUS)
    for drone_id in flight_settings.drones_in_mission:
        await send(
            MessageType.SEND_GROUND_TRUTH_COORDS,
            {"lat": 1.0, "lon": 1.0},
            to=(drone_id,),
            wait=GROUND_TRUTH_WINDOW_SEC + 1,
        )
    await send(MessageType.COMMAND_OFFSET_DISTRIBUTION)
    # No acks exist for these, so follow each with a swarm status to see every cmd_msg.
    for command in (MessageType.LAND, MessageType.EMERGENCY_LAND, MessageType.DISARM):
        await send(command, wait=1.0)
        await send(MessageType.REQUEST_SWARM_STATUS)

    writer.close()
    listener.close()


async def main() -> None:
    parser = FlightSettings.add_flight_arguments(argparse.ArgumentParser())
    # The stock default is relative to the working directory; pin it to the repo root.
    parser.set_defaults(config=os.path.join(_REPO_DIR, "mission_config.json"))
    parser.add_argument(
        "--fake-app",
        action="store_true",
        help="Drone 1 only: send the app's commands itself instead of waiting for the real app",
    )
    args = parser.parse_args()
    flight_settings = FlightSettings.from_mission_config(args=args)
    self_id = flight_settings.current_drone_ID

    drone = Drone()
    drone._vehicle = _NoVehicle()  # pylint: disable=protected-access
    interdrone = Interdrone(flight_settings=flight_settings, drone=drone)
    # Log instead of running the Land / EmergencyLand states.
    interdrone.start_land = lambda: print(f"[drone {self_id}] would start LAND")
    interdrone.start_emergency_land = lambda: print(f"[drone {self_id}] would start EMERGENCY_LAND")
    checker = AppCommandChecker(interdrone)
    interdrone_task = asyncio.create_task(interdrone.start_interdrone())

    print(f"Drone {self_id}: pinging {flight_settings.other_drones_in_mission}")
    while not await interdrone.ping_drones():
        await asyncio.sleep(0.1)
    print(f"Drone {self_id}: all drones answered ping")

    try:
        if args.fake_app and self_id == 1:
            await fake_app(flight_settings)
        else:
            if self_id == 1:
                print("Waiting for commands from the app. Ctrl+C to stop and print the summary.")
            while True:
                await asyncio.sleep(1)
    finally:
        await checker.print_summary()
        interdrone_task.cancel()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
