"""Declares the EmergencyLand state class."""

from typing import Awaitable, Callable, ClassVar

from state_machine.states.state import State


class EmergencyLand(State):
    """
    The EmergencyLand state of the state machine.

    Entered by cancelling whatever state was running, so it can be reached from
    any point in a flight. Unlike Land, it does not transit anywhere first: the
    drone descends where it stands.

    Attributes
    ----------
    run_callable : ClassVar[Callable[["EmergencyLand"], Awaitable[None]]]
        The callable object to call when this state is run. This object is
        shared between all instances of this class.

    Methods
    -------
    run() -> Awaitable[None]:
        Execute the logic associated with this state. This state is terminal,
        so it has no successor.
    """

    run_callable: ClassVar[Callable[["EmergencyLand"], Awaitable[None]]]

    def run(self) -> Awaitable[None]:
        return self.run_callable()
