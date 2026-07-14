"""Declares the LidarMap state class."""

from typing import TYPE_CHECKING, Awaitable, Callable, ClassVar

from state_machine.drone import Drone
from state_machine.flight_settings import FlightSettings
from state_machine.states.state import State

if TYPE_CHECKING:
    from state_machine.interdrone import Interdrone


class LidarMap(State):
    """
    The LidarMap state of the state machine.

    This state circles an object detected by the LIDAR proximity monitor,
    collecting range returns to map the object's edges, then resumes the
    state that was interrupted.

    Attributes
    ----------
    resume_state : State
        The state to return to once the scan completes. Traversal states
        are re-entrant against drone.waypoints, so returning a fresh
        instance continues the interrupted path.
    run_callable : ClassVar[Callable[["LidarMap"], Awaitable[State]]]
        The callable object to call when this state is run. This object is
        shared between all instances of this class.

    Methods
    -------
    run() -> Awaitable[State]:
        Execute the logic associated with this state and return the next state
        to transition to.
    """

    run_callable: ClassVar[Callable[["LidarMap"], Awaitable[State]]]

    def __init__(
        self,
        drone: Drone,
        flight_settings: FlightSettings,
        interdrone: "Interdrone",
        resume_state: State,
    ) -> None:
        super().__init__(drone, flight_settings, interdrone)
        self.resume_state = resume_state

    def run(self) -> Awaitable[State]:
        return self.run_callable()
