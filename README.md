# TODO
- implement hardware_bridge.py
- integrate camera feed and fuse with lidar in perception_hub
- implement localization 

# Dependencies
```
sudo apt install -y libzbar0
pip3 install pyzbar opencv-python-headless numpy pynput pyyaml
```

Clone: https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

Replace: turtlebot3_house.world with team2_delivery's turtlebot3_house.world file.

# Config
Table locations, delivery_settings and safety_thresholds: config/table_locations.yaml
- Table locations were placeholders and are ignored when hotspots are turned on


# Run
```
cd ~/catkin_ws ; catkin_make ; source devel/setup.bash
```

## Simulation
NOTE: Robot is expected to be spawned in middle room with trashcan

1. Mapping
```
roslaunch team2_delivery mapping.launch
```
2. Save map
```
mkdir -p ~/catkin_ws/src/team2_delivery/maps
rosrun map_server map_saver -f $(rospack find team2_delivery)/maps/restaurant_map
```
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
