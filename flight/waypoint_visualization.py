"""
Visualization module for drone waypoints and paths with local grid conversion.
"""

from typing import Optional
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from matplotlib.colors import TABLEAU_COLORS
import math
from waypoint import Line, Waypoint


class CoordinateConverter:
    """Convert lat/lon coordinates to a local Cartesian grid."""

    EARTH_RADIUS_M = 6371000  # Earth radius in meters
    INCHES_PER_METER = 39.3701

    def __init__(self, reference_lat: float, reference_lon: float):
        """
        Initialize converter with a reference point (origin of local grid).

        Args:
            reference_lat: Latitude of the origin point
            reference_lon: Longitude of the origin point
        """
        self.ref_lat = reference_lat
        self.ref_lon = reference_lon

    def local_to_lat_lon(self, x: float, y: float) -> tuple[float, float]:
        """
        Convert local x/y coordinates in inches back to lat/lon.

        Args:
            x: X coordinate in inches from reference point
            y: Y coordinate in inches from reference point

        Returns:
            Tuple of (latitude, longitude)
        """
        x_m = x / self.INCHES_PER_METER
        y_m = y / self.INCHES_PER_METER
        ref_lat_rad = math.radians(self.ref_lat)
        ref_lon_rad = math.radians(self.ref_lon)
        lat_rad = y_m / self.EARTH_RADIUS_M + ref_lat_rad
        lon_rad = x_m / (self.EARTH_RADIUS_M * math.cos(ref_lat_rad)) + ref_lon_rad
        return math.degrees(lat_rad), math.degrees(lon_rad)

    def lat_lon_to_local(self, lat: float, lon: float) -> tuple[float, float]:
        """
        Convert lat/lon to local x/y coordinates in meters.

        Uses simplified equirectangular projection for small areas.

        Args:
            lat: Latitude
            lon: Longitude

        Returns:
            Tuple of (x, y) in inches from the reference point
        """
        # Convert to radians
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        ref_lat_rad = math.radians(self.ref_lat)
        ref_lon_rad = math.radians(self.ref_lon)

        # Calculate distances using equirectangular approximation, converted to inches
        x = (
            (lon_rad - ref_lon_rad)
            * self.EARTH_RADIUS_M
            * math.cos(ref_lat_rad)
            * self.INCHES_PER_METER
        )
        y = (lat_rad - ref_lat_rad) * self.EARTH_RADIUS_M * self.INCHES_PER_METER

        return x, y

    def get_bounds(self, waypoints: list[Waypoint]) -> tuple[float, float, float, float]:
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
            self.lat_lon_to_local(wp._get_latitude(), wp._get_longitude()) for wp in waypoints
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
        self._drag_wp: Optional[Waypoint] = None

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
            self.fig.canvas.mpl_connect("button_press_event", self._on_press)
            self.fig.canvas.mpl_connect("motion_notify_event", self._on_motion)
            self.fig.canvas.mpl_connect("button_release_event", self._on_release)

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
            self.ax.plot(xs, ys, color=color, linestyle="-", linewidth=2, alpha=0.6, zorder=2)

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
        self.ax.set_xlabel("X (inches)", fontsize=12)
        self.ax.set_ylabel("Y (inches)", fontsize=12)
        self.ax.set_title(self.title, fontsize=14, fontweight="bold")
        self.ax.legend(loc="best", fontsize=10)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect("equal", adjustable="box")

        plt.tight_layout()

    def _print_collision_debug(self) -> None:
        if not self.waypoints:
            return
        print("\nNew Print Collision Debug:")
        drone_waypoints: dict[int, list[Waypoint]] = {}
        for wp in self.waypoints:
            drone_id = wp._get_drone_id()
            if drone_id not in drone_waypoints:
                drone_waypoints[drone_id] = []
            drone_waypoints[drone_id].append(wp)

        for drone_id in drone_waypoints:
            drone_waypoints[drone_id].sort(key=lambda wp: wp._get_waypoint_id())

        drone_ids = list(drone_waypoints.keys())
        if len(drone_ids) < 2:
            return
        collision_points = Waypoint.check_for_collision(drone_waypoints[1], drone_waypoints[2])
        for waypoint_group in collision_points:
            print(
                f"Collision at: {waypoint_group[0][0].name} -> {waypoint_group[0][1].name} and {waypoint_group[1][0].name} -> {waypoint_group[1][1].name} "
            )
        # for i in range(len(drone_ids)):
        #     # print(f"i: {i}")
        #     for j in range(i + 1, len(drone_ids)):
        #         # print(f"j: {j}")
        #         # issue is seg 2 isn't existing
        #         wps1 = drone_waypoints[drone_ids[i]]
        #         wps2 = drone_waypoints[drone_ids[j]]

        #         lines1 = [Line(wps1[k], wps1[k + 1]) for k in range(len(wps1) - 1)]
        #         lines2 = [Line(wps2[k], wps2[k + 1]) for k in range(len(wps2) - 1)]
        #         # print(f"For i = {i} and j = {j}, Lines are: \nlines1: {lines1}\nlines2: {lines2} ")
        #         for pi, l1 in enumerate(lines1):
        #             for pj, l2 in enumerate(lines2):
        #                 s1_x = l1.dx
        #                 s1_y = l1.dy
        #                 s2_x = l2.dx
        #                 s2_y = l2.dy
        #                 denom = s1_x * s2_y - s2_x * s1_y
        #                 s02_x = l1.start._get_longitude() - l2.start._get_longitude()
        #                 s02_y = l1.start._get_latitude() - l2.start._get_latitude()
        #                 s_numer = s1_x * s02_y - s1_y * s02_x
        #                 t_numer = s2_x * s02_y - s2_y * s02_x
        #                 print(
        #                     f"Drone {drone_ids[i]} seg {pi} vs Drone {drone_ids[j]} seg {pj}: "
        #                     f"denom={denom:.9f}, s_numer={s_numer:.9f}, t_numer={t_numer:.9f}"
        #                 )

    def _on_press(self, event) -> None:
        if event.inaxes != self.ax or event.button != 1 or not self.waypoints:
            return

        min_dist_sq = float("inf")
        closest = None

        for wp in self.waypoints:
            x, y = self.converter.lat_lon_to_local(wp._get_latitude(), wp._get_longitude())
            disp = self.ax.transData.transform((x, y))
            dist_sq = (disp[0] - event.x) ** 2 + (disp[1] - event.y) ** 2
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                closest = wp

        if min_dist_sq < 15**2:
            self._drag_wp = closest

    def _on_motion(self, event) -> None:
        if self._drag_wp is None or event.inaxes != self.ax:
            return
        lat, lon = self.converter.local_to_lat_lon(event.xdata, event.ydata)
        self._drag_wp._set_latitude(lat)
        self._drag_wp._set_longitude(lon)
        self._print_collision_debug()
        self._draw()
        self.fig.canvas.draw_idle()

    def _on_release(self, event) -> None:
        self._drag_wp = None

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
                drone_id=1,
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
                drone_id=2,
                lat=wp_dict["latitude"],
                long=wp_dict["longitude"],
                waypoint_id=wp_id,
                name=wp_dict["name"],
            )
        )
        wp_id += 1

    print("Drag waypoints to reposition them. Close the window to exit.")
    visualizer = LiveWaypointVisualizer(title="Interactive Waypoint Visualization")
    visualizer.update(waypoints)
    plt.ioff()
    plt.show()
