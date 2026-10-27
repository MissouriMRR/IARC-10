"""Declares the ExpandNodes state class."""

from typing import Awaitable, Callable, ClassVar

from state_machine.states.state import State


class ExpandNodes(State):
    """
    The ExpandNodes state of the state machine.

    This state represents the phase after a valid path has been found
    (the scan is fully complete) but before the mission ends -- it grows
    every known mine's danger radius by a final safety margin and
    re-validates the already-flown/planned route against it.

    Attributes
    ----------
    run_callable : ClassVar[Callable[["ExpandNodes"], Awaitable[State]]]
        The callable object to call when this state is run. This object is
        shared between all instances of this class.

    Methods
    -------
    run() -> Awaitable[State]:
        Execute the logic associated with this state and return the next state
        to transition to.
    """

    run_callable: ClassVar[Callable[["ExpandNodes"], Awaitable[State]]]

    def run(self) -> Awaitable[State]:
        return self.run_callable()
