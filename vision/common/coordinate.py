"""Implements a Coordinate class representing geographic coordinates.s"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Coordinate:
    """
    A class representing a geographic coordinate.

    Attributes
    ----------
    lat : float
        The latitude of the coordinate.
    lon : float
        The longitude of the coordinate.
    alt : float
        The altitude of the coordinate in DroneKit's global relative frame.
        This value is relative to the home location of the drone, which is where
        the drone first gets a GPS fix.
        See more here: https://dronekit.netlify.app/automodule.html#dronekit.Locations.global_relative_frame.
    """

    lat: float
    lon: float
    alt: float
