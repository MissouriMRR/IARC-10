
# IARC-10

## WHEN EDITTING THE Mission_Config.json PLEASE READ THIS:

"run_title": **[String title of the flight]** *This is logging info from my understanding*

"run_description": **[String description of the flight]** *This is logging info from my understanding*

"real_mode_config": {

---"mission_data_path": **[String path to the mission data json file]** *not sure the exact function of this file but its important*

}

"sim_mode_config": {

---"mission_data_path": **[String path to the mission data json file]** *same as the above*

}

"airsim_mode_config": {

---"mission_data_path": **[String path to the mission data json file]** *same as the above*

}

"simple_takeoff": **bool** *What type of take off should be used*

"app_opperable": **bool** *Should the drones be expecting to communicate with the app*

"current_drone_ID": **int** *The ID of drone that this instance of the files is installed on*

"app_IP": **int IP of the App** *Should only be used if the drone is going to expect to connect to the app*

"number_of_total_drones": **int** *The number of drones active in the swarm*

"other_drone_info": [
---{"id": **int** *The "other" drone's ID int*, "IP": **int IP of the target drone** *What's the IP of the Drone associated with this ID*}
---*Copy the above format for each drone and add to this list of objects*
]

"mission_field_corners": [

---{"lat": **float** *take a wild guess*, "lon": **float** *take a wild guess*}, **1**

---{"lat": **float** *take a wild guess*, "lon": **float** *take a wild guess*}, **2**

---{"lat": **float** *take a wild guess*, "lon": **float** *take a wild guess*}, **3**

---{"lat": **float** *take a wild guess*, "lon": **float** *take a wild guess*} **4**

---**IMPORTANT** *The order the points go in needs to match the following:*

---**1---end---2**

---**|xxxxxxxxx|**

---**|xxxxxxxxx|**

---**3--start--4**

---*ALSO CHECK THE COORDS IN GOOGLE EARTH BEFORE IMPLEMENTATION*

]

"start_coord": {
---"lat": **float** *take a wild guess*,
---"lon": **float** *take a wild guess*
---*See above for the map of where this point should be in relation to other objects*
},

"max_flight_height": **float** *maximum altitude meters*

"lidar_config": {

---"enabled": **bool** *whether the LIDAR obstacle detection/mapping mode runs at all*

---"proximity_threshold_m": **float** *range in meters below which an object queues a mapping scan*

---"standoff_radius_m": **float** *radius in meters of the circle flown around a detected object*

---"circle_num_points": **int** *number of waypoints (sampling stops) on the scan circle*

---"dedupe_radius_ft": **float** *objects within this many feet of an already scanned object are treated as the same object and skipped*

---"max_object_radius_ft": **float** *scan returns farther than this from the object center are treated as a different obstacle and discarded*

}

## LIDAR mapping mode

The drone carries two forward-facing TF Luna 1-D rangefinders (front-left and front-right)
wired to the flight controller, so range data arrives over MAVLink as `DISTANCE_SENSOR`
messages. A background task (`flight/lidar.py`, started by the `FlightManager` when
`lidar_config.enabled` is true) watches the range stream; when an unscanned object comes
within `proximity_threshold_m` a scan is queued. The active traversal state finishes its
current waypoint leg, checks the approach is clear, then diverts into the `LidarMap` state:
it circles the object at `standoff_radius_m` with the nose pointed at it, collects range
returns, filters out returns from other obstacles, and stores the object's vertices in
field-frame feet in `drone.lidar.scanned_objects`. The interrupted state then resumes.

Unit tests: `uv run python -m flight.tests.test_lidar_math`
Pipeline visualization: `uv run python -m flight.pathfinding.test_lidar_vertices`

### Simulating the rangefinders in AirSim

Add two forward-facing Distance sensors to the drone template in your AirSim settings
(the `update_airsim_settings.ps1` script clones the template's `Sensors` block to every
drone automatically):

```json
"Sensors": {
  "DistanceLeft": {
    "SensorType": 5, "Enabled": true,
    "MinDistance": 0.2, "MaxDistance": 8,
    "X": 0.3, "Y": -0.15, "Z": 0, "Yaw": 0, "Pitch": 0, "Roll": 0,
    "DrawDebugPoints": true
  },
  "DistanceRight": {
    "SensorType": 5, "Enabled": true,
    "MinDistance": 0.2, "MaxDistance": 8,
    "X": 0.3, "Y": 0.15, "Z": 0, "Yaw": 0, "Pitch": 0, "Roll": 0,
    "DrawDebugPoints": true
  }
}
```

ArduPilot SITL parameters: `RNGFND1_TYPE=100` (SITL/AirSim), `RNGFND1_MIN_CM=20`,
`RNGFND1_MAX_CM=800`, `RNGFND1_ORIENT=0` (forward), and the same for `RNGFND2_*`.
On the real drone the TF Lunas use `RNGFNDx_TYPE=20` (serial) or `25` (I2C).

**Verify before relying on it:** whether ArduPilot's AirSim backend forwards
forward-facing Distance sensors as `DISTANCE_SENSOR` messages with orientation 0 has not
been confirmed. Start SITL + AirSim and watch for messages with
`vehicle.add_message_listener('DISTANCE_SENSOR', print)`. If nothing arrives, an
AirSim-client backend implementing the same two-method protocol as
`MavlinkRangefinderBackend` (`start()` / `latest_all()`, using
`airsim.MultirotorClient().getDistanceSensorData()`) can be swapped in for sim runs —
the MAVLink backend stays as-is for the real drone. There is also a built-in fallback:
if no `DISTANCE_SENSOR` message ever arrives the backend polls `vehicle.rangefinder`,
which only exposes the first rangefinder.
