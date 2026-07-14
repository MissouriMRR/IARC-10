"""Implements the behavior of the LidarMap state."""

import asyncio
import logging
import time

from flight.circlePath import circle_waypoints
from flight.lidar import (
    DWELL_S,
    RangeSample,
    ScannedObject,
    condition_yaw,
    filter_scan_samples,
    rotate_to_nearest,
)
from flight.pathfinding.utils.geo import bearing_deg_between
from flight.pathfinding.utils.goto import move_to
from state_machine.state_tracker import (
    update_drone,
    update_flight_settings,
    update_state,
)
from state_machine.states.lidar_map import LidarMap
from state_machine.states.state import State

# Seconds allowed for the drone to swing its nose toward the object before
# sampling starts at each circle vertex
YAW_SETTLE_S = 0.5


async def run(self: LidarMap) -> State:
    """
    Implements the run method for the LidarMap state.

    Circles the object queued by the LIDAR proximity monitor, collecting
    range returns at each circle vertex with the nose pointed at the object
    center. Returns from other obstacles are filtered out, the surviving
    hit points are stored as the object's vertices in field-frame feet, and
    the interrupted state is resumed.

    Returns
    -------
    State
        The state that was interrupted by this scan (resume_state).
    """
    lidar = self.drone.lidar
    assert lidar is not None, "LidarMap state entered without an active LidarController"
    try:
        update_state("LidarMap")
        update_drone(self.drone)
        update_flight_settings(self.flight_settings)
        logging.info("LidarMap state running")

        vehicle = self.drone.vehicle
        lidar.scan_in_progress = True
        center = lidar.pending_center_estimate
        assert center is not None, "LidarMap state entered without a pending scan"
        config = lidar.config
        scan_alt = vehicle.location.global_relative_frame.alt

        # Circle of sampling stops around the object, entered at the vertex
        # nearest the drone so we never fly across the object to start
        circle = circle_waypoints(
            center[0],
            center[1],
            radius_m=config.standoff_radius_m,
            drone_id=self.drone.id,
            num_points=config.circle_num_points,
            closed=True,
        )
        position = vehicle.location.global_relative_frame
        circle = rotate_to_nearest(circle, position.lat, position.lon)

        # Deconflict the circle with the other drones' paths
        await self.interdrone.send_new_waypoints(
            tuple(self.flight_settings.other_drones_in_mission), circle
        )
        for state in self.interdrone.drone_states:
            self.drone.checkForCollision(state.list_of_waypoints)

        # Hold commanded yaw through simple_goto legs; otherwise the
        # autopilot swings the nose back toward the direction of travel
        # and the forward-facing sensors point away from the object
        old_yaw_behavior = None
        try:
            old_yaw_behavior = vehicle.parameters["WP_YAW_BEHAVIOR"]
            vehicle.parameters["WP_YAW_BEHAVIOR"] = 0
        except KeyError:
            logging.warning("WP_YAW_BEHAVIOR parameter unavailable; yaw may not hold")

        samples: list[RangeSample] = []
        try:
            for waypoint in circle:
                await move_to(vehicle, waypoint.lat, waypoint.long, scan_alt, tolerance=1)
                heading = bearing_deg_between(waypoint.lat, waypoint.long, center[0], center[1])
                condition_yaw(vehicle, heading)
                await asyncio.sleep(YAW_SETTLE_S)
                samples.extend(await lidar.collect(DWELL_S))
        finally:
            if old_yaw_behavior is not None:
                vehicle.parameters["WP_YAW_BEHAVIOR"] = old_yaw_behavior

        vertices_ft, vertices_latlon = filter_scan_samples(
            samples,
            center,
            config.standoff_radius_m,
            config.max_object_radius_ft,
            lidar.transformer,
        )

        if vertices_ft:
            # Refine the center to the centroid of the accepted vertices
            final_center = (
                sum(vertex[0] for vertex in vertices_latlon) / len(vertices_latlon),
                sum(vertex[1] for vertex in vertices_latlon) / len(vertices_latlon),
            )
        else:
            # Register the object anyway so the dedupe check prevents an
            # endless re-trigger loop on an object we cannot resolve
            logging.warning(
                "LidarMap: no samples survived filtering; registering object "
                "at estimated center with no vertices"
            )
            final_center = center

        center_ft = lidar.transformer.latlon_to_local(*final_center)
        lidar.register_scan(
            ScannedObject(
                center_latlon=final_center,
                center_field_ft=(center_ft[0], center_ft[1]),
                vertices_field_ft=vertices_ft,
                vertices_latlon=vertices_latlon,
                scanned_at=time.time(),
            )
        )

        logging.info("LidarMap complete; resuming %s state", self.resume_state.name)
        return self.resume_state

    except asyncio.CancelledError as ex:
        logging.error("LidarMap state canceled")
        raise ex
    finally:
        if lidar is not None:
            lidar.scan_in_progress = False


# Setting the run_callable attribute of the LidarMap class to the run function
LidarMap.run_callable = run
