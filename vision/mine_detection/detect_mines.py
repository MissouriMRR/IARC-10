from collections.abc import Iterable

import cv2
import cv2.typing

from vision.common import Attitude, BoundingBox, Coordinate


def detect_mines(
    image: cv2.typing.MatLike, coord: Coordinate, attitude: Attitude
) -> Iterable[BoundingBox]:
    """
    Detect mines in the given image.

    Parameters
    ----------
    image : cv2.typing.MatLike
        The input image in which to detect mines.
    coord : Coordinate
        The GPS coordinate of the drone and camera when the image was captured.
    attitude : Attitude
        The combined attitude of the drone and camera when the image was captured.

    Returns
    -------
    Iterable[BoundingBox]
        An iterable of BoundingBox instances representing detected mines.
    """
    raise NotImplementedError("Mine detection not yet implemented.")
