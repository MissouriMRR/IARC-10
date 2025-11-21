"""
Contains the extract_gps() function for extracting data out of
the provided boundary data JSON file for the IARC competition.
"""

import argparse
from dataclasses import dataclass
import json
from typing import Any

import utm


@dataclass(frozen=True)
class BoundaryPoint:
    """
    Data class storing the data for a single boundary point.

    Attributes
    ----------
    latitude : float
        The latitude of the boundary point.
    longitude : float
        The longitude of the boundary point.
    """

    latitude: float
    longitude: float


@dataclass(frozen=True)
class BoundaryPointUtm:
    """
    Data class storing the data for a single boundary point using UTM coordinates.

    Attributes
    ----------
    easting : float
        The easting of the boundary point.
    northing : float
        The northing of the boundary point.
    zone_number : int
        The zone number of the boundary point.
    zone_letter : str
        The zone letter of the boundary point.
    """

    easting: float
    northing: float
    zone_number: int
    zone_letter: str


@dataclass(frozen=True)
class AltitudeLimit:
    """
    Data class storing the altitude limits for the flight.

    Attributes
    ----------
    min_altitude : float
        The minimum altitude for the flight, in meters.
    max_altitude : float
        The maximum altitude for the flight, in meters.
    """

    min_altitude: float
    max_altitude: float


@dataclass(frozen=True)
class GPSData:
    """
    Data class storing the GPS boundary data extracted from the JSON file.

    Attributes
    ----------
    boundary_points : list[BoundaryPoint]
        The list of boundary points in latitude and longitude.
    boundary_points_utm : list[BoundaryPointUtm]
        The list of boundary points in UTM coordinates.
    altitude_limits : AltitudeLimit
        The altitude limits for the flight.
    """
    boundary_points: list[BoundaryPoint]
    boundary_points_utm: list[BoundaryPointUtm]
    altitude_limits: AltitudeLimit


def extract_gps(path: str) -> GPSData:
    """
    Returns the waypoints, boundary points, and altitude limits from a waypoint data file.

    Parameters
    ----------
    path : str
        File path to the waypoint data JSON file.

    Returns
    -------
    GPSData
        The data in the mission data file.

    Raises
    ------
    KeyError
        If the structure of the JSON is incorrect.
    ValueError
        If there are invalid values in the JSON.
    """

    # Load the JSON file as a Python dict to be able to easily access the data
    with open(path, encoding="UTF-8") as data_file:
        json_data: dict[str, Any] = json.load(data_file)

    boundary_points: list[BoundaryPoint] = []
    boundary_points_utm: list[BoundaryPointUtm] = []

    # Get forced UTM zone number and zone letter
    forced_zone_number: int
    forced_zone_letter: str
    _, _, forced_zone_number, forced_zone_letter = utm.from_latlon(
        json_data["flyzones"]["boundaryPoints"][0]["latitude"],
        json_data["flyzones"]["boundaryPoints"][0]["longitude"],
    )

    boundary_point: dict[str, float]
    full_boundary_point_utm: BoundaryPointUtm
    for boundary_point in json_data["flyzones"]["boundaryPoints"]:
        latitude = boundary_point["latitude"]
        longitude = boundary_point["longitude"]

        boundary_points.append(BoundaryPoint(latitude, longitude))
        full_boundary_point_utm = BoundaryPointUtm(
            *utm.from_latlon(latitude, longitude, forced_zone_number, forced_zone_letter)
        )
        boundary_points_utm.append(full_boundary_point_utm)

    waypoint_data: GPSData = GPSData(
        boundary_points=boundary_points,
        boundary_points_utm=boundary_points_utm,
        altitude_limits=AltitudeLimit(
            json_data["flyzones"]["minAltitude"],
            json_data["flyzones"]["maxAltitude"],
        ),
    )
    return waypoint_data


# If run on its own, use the default data location
if __name__ == "__main__":
    # Read file to be used as the data file using the -file argument
    parser: argparse.ArgumentParser = argparse.ArgumentParser()
    parser.add_argument("-file")
    args: argparse.Namespace = parser.parse_args()

    extract_gps(vars(args)["file"])
