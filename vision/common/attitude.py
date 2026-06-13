from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Attitude:
    """
    A class representing the attitude of a vehicle or camera in terms of roll, pitch, and yaw.
    When applying a rotation, roll is applied first, followed by pitch, then yaw.
    An attitude with 0 roll and pitch means the vehicle is level with the horizon
    or that the camera is pointed straight down.
    See more here: https://dronekit.netlify.app/automodule.html#dronekit.Attitude.

    Attributes
    ----------
    roll_deg : float
        The roll angle in degrees.
        When viewed from behind, a positive roll corresponds to a right-wing-down rotation.
    pitch_deg : float
        The pitch angle in degrees.
        When viewed from the left, a positive pitch corresponds to a nose-up rotation.
    yaw_deg : float
        The yaw angle in degrees. A yaw of 0 is north, and 90 is east.
        When viewed from above, a positive yaw corresponds to a clockwise rotation.
    """
    roll_deg: float
    pitch_deg: float
    yaw_deg: float

    @property
    def roll_rad(self) -> float:
        """Get the roll in radians."""
        return math.radians(self.roll_deg)

    @property
    def pitch_rad(self) -> float:
        """Get the pitch in radians."""
        return math.radians(self.pitch_deg)

    @property
    def yaw_rad(self) -> float:
        """Get the yaw in radians."""
        return math.radians(self.yaw_deg)
