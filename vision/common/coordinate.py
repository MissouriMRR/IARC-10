"""Implements a Coordinate class representing geographic coordinates.s"""

from dataclasses import dataclass

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from vision.common.attitude import Attitude
    from vision.common.camera_info import CameraInfo


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

    @staticmethod
    def from_image_coord(
        x: float,
        y: float,
        image_width: float,
        image_height: float,
        camera_coord: "Coordinate",
        attitude: "Attitude",
        camera_info: "CameraInfo",
    ) -> "Coordinate":
        """
        Convert image coordinates to a geographic coordinate.

        Parameters
        ----------
        x : float
            The x-coordinate in the image.
        y : float
            The y-coordinate in the image.
        image_width : float
            The width of the image.
        image_height : float
            The height of the image.
        coord : Coordinate
            The GPS coordinate of the drone and camera when the image was captured.
        attitude : Attitude
            The combined attitude of the drone and camera when the image was captured.
        camera_info : CameraInfo
            The camera information including field of view.

        Returns
        -------
        Coordinate
            The geographic coordinate corresponding to the image coordinates.
        """
        raise NotImplementedError("Coordinate conversion not yet implemented.")
