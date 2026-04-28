"""
Visualization module for drone waypoints and paths with local grid conversion.
"""

import time
from typing import Optional
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from matplotlib.colors import TABLEAU_COLORS
import math
from waypoint import Waypoint


class CoordinateConverter:
    """Convert lat/lon coordinates to a local Cartesian grid."""

    EARTH_RADIUS_M = 6371000  # Earth radius in meters

    def __init__(self, reference_lat: float, reference_lon: float):
        """
        Initialize converter with a reference point (origin of local grid).

        Args:
            reference_lat: Latitude of the origin point
            reference_lon: Longitude of the origin point
        """
        self.ref_lat = reference_lat
        self.ref_lon = reference_lon

    def lat_lon_to_local(self, lat: float, lon: float) -> tuple[float, float]:
        """
        Convert lat/lon to local x/y coordinates in meters.

        Uses simplified equirectangular projection for small areas.

        Args:
            lat: Latitude
            lon: Longitude

        Returns:
            Tuple of (x, y) in meters from the reference point
        """
        # Convert to radians
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        ref_lat_rad = math.radians(self.ref_lat)
        ref_lon_rad = math.radians(self.ref_lon)

        # Calculate distances in meters using equirectangular approximation
        x = (lon_rad - ref_lon_rad) * self.EARTH_RADIUS_M * math.cos(ref_lat_rad)
        y = (lat_rad - ref_lat_rad) * self.EARTH_RADIUS_M

        return x, y

    def get_bounds(
        self, waypoints: list[Waypoint]
    ) -> tuple[float, float, float, float]:
        """
        Get the bounding box of all waypoints in local coordinates.

        Args:
            waypoints: List of Waypoint objects

        Returns:
            Tuple of (min_x, max_x, min_y, max_y)
        """
        if not waypoints:
            return 0, 0, 0, 0

        coords = [
            self.lat_lon_to_local(wp._get_latitude(), wp._get_longitude())
            for wp in waypoints
        ]
        xs = [c[0] for c in coords]
        ys = [c[1] for c in coords]

        return min(xs), max(xs), min(ys), max(ys)


class LiveWaypointVisualizer:
    """Interactive waypoint visualizer that supports live updates."""

    def __init__(
        self,
        title: str = "Drone Waypoint Paths",
        show_labels: bool = True,
        show_arrows: bool = True,
        figsize: tuple[int, int] = (12, 8),
    ):
        """
        Initialize the live visualizer.

        Args:
            title: Title for the plot
            show_labels: Whether to show waypoint labels
            show_arrows: Whether to show direction arrows on the paths
            figsize: Figure size as (width, height)
        """
        self.title = title
        self.show_labels = show_labels
        self.show_arrows = show_arrows
        self.figsize = figsize

        self.fig = None
        self.ax = None
        self.converter = None
        self.waypoints = None

        # Enable interactive mode for live updates
        plt.ion()

    def update(self, waypoints: list[Waypoint]) -> None:
        """
        Update the visualization with new waypoints.

        Args:
            waypoints: List of Waypoint objects to visualize
        """
        if not waypoints:
            print("No waypoints to visualize")
            return

        # Initialize converter using first waypoint as reference
        if self.converter is None:
            first_wp = waypoints[0]
            self.converter = CoordinateConverter(
                first_wp._get_latitude(), first_wp._get_longitude()
            )

        self.waypoints = waypoints
        self._draw()

    def _draw(self) -> None:
        """Draw or redraw the visualization."""
        # Clear previous plot
        if self.ax is not None:
            self.ax.clear()
        else:
            self.fig, self.ax = plt.subplots(figsize=self.figsize)

        if not self.waypoints:
            return

        # Group waypoints by drone_id
        drone_waypoints = {}
        for wp in self.waypoints:
            drone_id = wp._get_drone_id()
            if drone_id not in drone_waypoints:
                drone_waypoints[drone_id] = []
            drone_waypoints[drone_id].append(wp)

        # Sort waypoints for each drone
        for drone_id in drone_waypoints:
            drone_waypoints[drone_id].sort(key=lambda wp: wp._get_waypoint_id())

        # Get colors for different drones
        colors = list(TABLEAU_COLORS.values())

        # Plot paths for each drone
        for drone_idx, (drone_id, wps) in enumerate(drone_waypoints.items()):
            color = colors[drone_idx % len(colors)]

            # Convert to local coordinates
            local_coords = [
                self.converter.lat_lon_to_local(wp._get_latitude(), wp._get_longitude())
                for wp in wps
            ]
            xs = [c[0] for c in local_coords]
            ys = [c[1] for c in local_coords]

            # Plot waypoints as scatter points
            self.ax.scatter(
                xs,
                ys,
                s=200,
                color=color,
                alpha=0.8,
                label=f"Drone {drone_id}",
                zorder=3,
            )

            # Plot lines between consecutive waypoints
            self.ax.plot(
                xs, ys, color=color, linestyle="-", linewidth=2, alpha=0.6, zorder=2
            )

            # Add arrows if requested
            if self.show_arrows and len(wps) > 1:
                for i in range(len(wps) - 1):
                    x1, y1 = xs[i], ys[i]
                    x2, y2 = xs[i + 1], ys[i + 1]

                    # Calculate arrow position (middle of the line)
                    mid_x = (x1 + x2) / 2
                    mid_y = (y1 + y2) / 2
                    dx = (x2 - x1) * 0.3
                    dy = (y2 - y1) * 0.3

                    arrow = FancyArrowPatch(
                        (mid_x - dx / 2, mid_y - dy / 2),
                        (mid_x + dx / 2, mid_y + dy / 2),
                        arrowstyle="->",
                        mutation_scale=20,
                        color=color,
                        zorder=4,
                    )
                    self.ax.add_patch(arrow)

            # Add labels if requested
            if self.show_labels:
                for i, wp in enumerate(wps):
                    x, y = xs[i], ys[i]
                    label_text = wp.name if wp.name else f"ID: {wp._get_waypoint_id()}"
                    self.ax.annotate(
                        label_text,
                        (x, y),
                        xytext=(5, 5),
                        textcoords="offset points",
                        fontsize=8,
                        bbox=dict(boxstyle="round,pad=0.3", facecolor=color, alpha=0.3),
                    )

        # Formatting
        self.ax.set_xlabel("X (meters)", fontsize=12)
        self.ax.set_ylabel("Y (meters)", fontsize=12)
        self.ax.set_title(self.title, fontsize=14, fontweight="bold")
        self.ax.legend(loc="best", fontsize=10)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect("equal", adjustable="box")

        plt.tight_layout()

    def show(self, pause_duration: float = 0.1) -> None:
        """
        Display or update the visualization (non-blocking).

        Args:
            pause_duration: How long to pause after drawing (in seconds)
        """
        if self.fig is not None:
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.pause(pause_duration)

    def save(self, save_path: str) -> None:
        """
        Save the visualization to a file.

        Args:
            save_path: Path to save the figure
        """
        if self.fig is not None:
            self.fig.savefig(save_path, dpi=150, bbox_inches="tight")
            print(f"Visualization saved to {save_path}")


def visualize_waypoints(
    waypoints: list[Waypoint],
    title: str = "Drone Waypoint Paths",
    show_labels: bool = True,
    show_arrows: bool = True,
    figsize: tuple[int, int] = (12, 8),
    save_path: Optional[str] = None,
) -> None:
    """
    Visualize drone waypoint paths on a local grid.

    Args:
        waypoints: List of Waypoint objects to visualize
        title: Title for the plot
        show_labels: Whether to show waypoint labels
        show_arrows: Whether to show direction arrows on the paths
        figsize: Figure size as (width, height)
        save_path: Optional path to save the figure. If None, displays the plot.
    """
    visualizer = LiveWaypointVisualizer(title, show_labels, show_arrows, figsize)
    visualizer.update(waypoints)

    if save_path:
        visualizer.save(save_path)
    else:
        visualizer.show()


def visualize_waypoints_from_dict(
    waypoint_dicts: list[dict],
    drone_ids: list[int],
    title: str = "Drone Waypoint Paths",
    show_labels: bool = True,
    show_arrows: bool = True,
    figsize: tuple[int, int] = (12, 8),
    save_path: Optional[str] = None,
) -> None:
    """
    Convenience function to visualize waypoints from dictionaries.

    Args:
        waypoint_dicts: List of waypoint dictionaries with 'latitude' and 'longitude' keys
        drone_ids: List of drone IDs corresponding to each waypoint
        title: Title for the plot
        show_labels: Whether to show waypoint labels
        show_arrows: Whether to show direction arrows on the paths
        figsize: Figure size as (width, height)
        save_path: Optional path to save the figure. If None, displays the plot.
    """
    waypoints = []
    for idx, (wp_dict, drone_id) in enumerate(zip(waypoint_dicts, drone_ids)):
        wp = Waypoint(
            drone_id=drone_id,
            lat=wp_dict["latitude"],
            long=wp_dict["longitude"],
            waypoint_id=idx,
            name=wp_dict.get("name", ""),
        )
        waypoints.append(wp)

    visualize_waypoints(waypoints, title, show_labels, show_arrows, figsize, save_path)


if __name__ == "__main__":
    import json

    # Example usage with test data
    with open("./data/waypoints_test.json", "r") as f:
        data = json.load(f)

    waypoints = []
    wp_id = 0

    # Add Set 1 waypoints (drone 0)
    for wp_dict in data["Set 1"]:
        waypoints.append(
            Waypoint(
                drone_id=0,
                lat=wp_dict["latitude"],
                long=wp_dict["longitude"],
                waypoint_id=wp_id,
                name=wp_dict["name"],
            )
        )
        wp_id += 1

    # Add Set 2 waypoints (drone 1)
    for wp_dict in data["Set 2"]:
        waypoints.append(
            Waypoint(
                drone_id=1,
                lat=wp_dict["latitude"],
                long=wp_dict["longitude"],
                waypoint_id=wp_id,
                name=wp_dict["name"],
            )
        )
        wp_id += 1

    # Example: Live update demonstration with interactive mode
    print("Creating live visualization with interactive updates...")
    visualizer = LiveWaypointVisualizer(title="Live Waypoint Visualization")

    # First update with initial waypoints
    print("Initial visualization...")
    visualizer.update(waypoints)
    visualizer.show(pause_duration=2)

    # Modify waypoints and update (in same window)
    print("Updating with modified waypoints (drone 0 moved slightly)...")
    for wp in waypoints:
        if wp._get_drone_id() == 0:
            wp._set_latitude(wp._get_latitude() + 0.0001)

    visualizer.update(waypoints)
    visualizer.show(pause_duration=2)

    # Another update
    print("Updating again (drone 1 moved slightly)...")
    for wp in waypoints:
        if wp._get_drone_id() == 1:
            wp._set_longitude(wp._get_longitude() - 0.0001)

    visualizer.update(waypoints)
    visualizer.show(pause_duration=2)

    print("Done! Close the window to exit.")
    plt.show()
