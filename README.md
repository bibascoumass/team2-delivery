# TODO
- integrate camera feed and fuse with lidar in perception_hub 

# Dependencies
```
sudo apt install -y libzbar0
pip3 install pyzbar opencv-python-headless numpy pynput pyyaml
```

Clone: https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

Replace: turtlebot3_house.world with team2_delivery's turtlebot3_house.world file.

# Config

## config/table_locations.yaml
Table locations, delivery_settings and safety_thresholds
- Table locations were placeholders and are ignored when hotspots are turned on

## config/amcl.yaml
Full AMCL parameter set — likelihood_field laser model, diff drive odometry model, particles, update thresholds tuned for a slow-moving robot.

## maps/restaurant_map.pgm + maps/restaurant_map.yaml
Pre-built occupancy grid from a simulation mapping run. Required by both sim.launch and team2_delivery.launch for map_server.

## config/hotspots.yaml

## config/move_base_mapping.yaml
- Added a StaticLayer to the global costmap so AMCL-based localization works during re-mapping runs (previously static_map: false).

## config/move_base_delivery.yaml
- recovery_behavior_enabled = disbled
- track_unknown_space = false


# Run
```
cd ~/catkin_ws ; catkin_make ; source devel/setup.bash
```

## Simulation
NOTE: Robot is expected to be spawned in middle room with trashcan

1. Mapping
```
roslaunch team2_delivery mapping.launch open_rviz:=true
```
2. Save map
```
mkdir -p ~/catkin_ws/src/team2_delivery/maps
rosrun map_server map_saver -f $(rospack find team2_delivery)/maps/restaurant_map
```
- This writes maps/restaurant_map.pgm and maps/restaurant_map.yaml

3. Generate hotspots
```
rosrun team2_delivery generate_hotspots.py \
  _map_yaml:=$(rospack find team2_delivery)/maps/restaurant_map.yaml \
  _output_yaml:=$(rospack find team2_delivery)/config/hotspots.yaml
```
4. Delivery
NOTE: 'Human' cylinder is expected to spawn next to lower left trash can and loop between the two trash cans
```
roslaunch team2_delivery sim.launch
```

send delivery order:
```
rostopic pub /target_qr std_msgs/String "data: 'Table_2'" --once
```

check state:
```
rostopic echo /delivery_state
```
---

## Hardware Run

1. map
```
roslaunch team2_delivery mapping_hw.launch
```
TODO: base_link and base_scan are commented out in mapping_hw.launch and team2_delivery.launch . Need to measure the physical LIDAR mounting offset

2. deliver
```
roslaunch team2_delivery team2_delivery.launch
```
- use_sim_time = false 

lidar args:
```
roslaunch team2_delivery team2_delivery.launch \
  map_file:=/path/to/your_map.yaml \
  lidar_port:=/dev/ttyUSB0 \
  lidar_baud:=115200
```

send order:
```
rostopic pub /target_qr std_msgs/String "data: 'Table_2'" --once
```
---

# TOPICS
| Topic | Type | Description |
|---|---|---|
| /target_qr | String | delivery order IDs |
| /delivery_state | String | current state |
| /qr_data | QRInfo.msg | QR scan results |
| /safety_status | SafetyStatus.msg | CLEAR / WARNING / CRITICAL obstacle level |
| /scan |  |  |
| /cmd_vel | | |
| /odom | | |
| /map |OccupancyGrid |  |

# Nodes

## hardware_bridge.py
- Reads ODOM x y yaw vx wz lines back from Arduino, publishes /odom and broadcasts odom → base_link TF at 20 Hz
- hardware_bridge now receives initial_x/y/yaw so dead-reckoning sim odometry starts at the correct spawn position
- Opens a port at startup default params: (/dev/ttyACM0, 115200 baud)
- Sends velocity commands to Arduino as CMD {linear_x:.3f} {angular_z:.3f}\n
- Sends a serial stop command (CMD 0.000 0.000) on timeout
- sim_mode=true path: dead-reckoning odometry integration (mid-point Euler) so /odom + TF still work in simulation without Arduino

## task_coordinator.py
- AVOID_DYNAMIC triggers for dynamic obstacles. It parses the sensor_source field of SafetyStatus for a dynamic: prefix and only yields for dynamic objects. Static obstacle warnings do NOT interrupt delivery.

# Utils

## delivery_debug_logger.py / explore_debug_logger.py : 
- Subscribe to all major topics and log data to CSV 
