
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
---**|//#########|**
---**|//#########|**
---**3--start--4**
---*ALSO CHECK THE COORDS IN GOOGLE EARTH BEFORE IMPLEMENTATION*

]

"start_coord": {
---"lat": **float** *take a wild guess*,
---"lon": **float** *take a wild guess*
---*See above for the map of where this point should be in relation to other objects*
},

"max_flight_height": **float** *maximum altitude meters*