from collections.abc import Iterable

import cv2
import cv2.typing

from vision.common import Attitude, BoundingBox, CameraInfo, Coordinate


def detect_mines(
    image: cv2.typing.MatLike,
    coord: Coordinate,
    attitude: Attitude,
    camera_info: CameraInfo,
) -> Iterable[tuple[Coordinate, float]]:
    """
    Detect mines in the given image and return coordinates.

    Parameters
    ----------
    image : cv2.typing.MatLike
        The input image in which to detect mines.
    coord : Coordinate
        The GPS coordinate of the drone and camera when the image was captured.
    attitude : Attitude
        The combined attitude of the drone and camera when the image was captured.
    camera_info : CameraInfo
        The camera information including field of view.

    Returns
    -------
    Iterable[tuple[Coordinate, float]]
        The coordinates and confidence scores of the detected mines.
    """
    raise NotImplementedError("Mine detection not yet implemented.")


def detect_mines_in_image(image: cv2.typing.MatLike) -> Iterable[BoundingBox]:
    """
    Detect mines in the given image and return bounding boxes.

    Parameters
    ----------
    image : cv2.typing.MatLike
        The input image in which to detect mines.

    Returns
    -------
    Iterable[BoundingBox]
        The bounding boxes of the detected mines.
    """
    raise NotImplementedError("Mine detection not yet implemented.")
