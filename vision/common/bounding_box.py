"""Implements a BoundingBox class for image processing tasks."""

from dataclasses import dataclass


@dataclass(frozen=True)
class BoundingBox:
    """
    A class representing a bounding box in an image.

    Attributes
    ----------
    x_min : float
        The minimum x-coordinate of the bounding box.
    y_min : float
        The minimum y-coordinate of the bounding box.
    width : float
        The width of the bounding box.
    height : float
        The height of the bounding box.
    confidence : float, default 1.0
        The confidence score of the bounding box detection.
    """
    x_min: float
    y_min: float
    width: float
    height: float
    confidence: float = 1.0

    @property
    def x_max(self) -> float:
        """Get the maximum x-coordinate of the bounding box."""
        return self.x_min + self.width

    @property
    def y_max(self) -> float:
        """Get the maximum y-coordinate of the bounding box."""
        return self.y_min + self.height
