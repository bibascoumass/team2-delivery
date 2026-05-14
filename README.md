# Overview
Autonomous Exploration and Vision-Based Destination Pursuit using the Triton robot.

NOTE: Due to issues working with the ROS Noetic Docker container on the Triton robot, ROS Melodic was used for running on the physical robot itself, but simulations were run on ROS Noetic. 

# Dependencies
```
sudo apt install -y libzbar0
pip3 install pyzbar opencv-python-headless numpy pynput pyyaml
```

## Simulation Dependencies
Clone: https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

Replace: turtlebot3_house.world with team2_delivery's turtlebot3_house.world file.

The python directive header for the following files under scripts/ must be updated to use Python 3 (#!/usr/bin/python3):
- cmd_mux.py
- delivery_state_machine.py
- exploration_navigator.py
- qr_detector.py
- qr_goal_tracker.py

Install required ROS packages:
```
sudo apt install -y \
  ros-noetic-gazebo-ros \
  ros-noetic-gazebo-plugins \
  ros-noetic-gmapping \
  ros-noetic-map-server \
  ros-noetic-move-base \
  ros-noetic-move-base-msgs \
  ros-noetic-explore-lite \
  ros-noetic-cv-bridge \
  ros-noetic-tf \
  ros-noetic-tf2-ros \
  ros-noetic-tf2-geometry-msgs \
  ros-noetic-rviz \
  ros-noetic-xacro \
  ros-noetic-robot-state-publisher \
  ros-noetic-joint-state-publisher \
  libzbar0
```

## Hardware Dependencies
The following packages must be cloned and compiled in order for the hardware launch files to work:
- https://github.com/Slamtec/rplidar_ros.git
- https://github.com/hrnr/m-explore.git

Install required ROS packages:
```
sudo apt install -y \
  ros-melodic-gmapping \
  ros-melodic-map-server \
  ros-melodic-move-base \
  ros-melodic-move-base-msgs \
  ros-melodic-cv-bridge \
  ros-melodic-tf \
  ros-melodic-tf2-ros \
  ros-melodic-tf2-geometry-msgs \
  ros-melodic-rviz \
  ros-melodic-xacro \
  ros-melodic-robot-state-publisher \
  ros-melodic-joint-state-publisher \
  libzbar0
```
If errors are encountered related to `pyzbar`, install:
```bash
sudo apt install -y libzbar0 python-pip python-setuptools
python -m pip install --user pyzbar==0.1.9
```

# Run
```
cd ~/catkin_ws ; catkin_make ; source devel/setup.bash
```

## Simulation
### Mapping
```
roslaunch team2_delivery sim_mapping.launch open_rviz:=true
```

Save map
```
rosrun map_server map_saver -f $(rospack find team2_delivery)/maps/<MAP_FILE_NAME>
```

Generate hotspots
```
rosrun team2_delivery generate_hotspots.py \
  _map_yaml:=$(rospack find team2_delivery)/maps/<MAP_FILE_NAME>.yaml \
  _output_yaml:=$(rospack find team2_delivery)/config/<HOTSPOT_FILE_NAME>.yaml
```

### Delivery
```
roslaunch team2_delivery sim_delivery.launch open_rviz:=true \
  map_file:=$(rospack find team2_delivery)/maps/<MAP_FILE_NAME> \
  hotspots_yaml:=$(rospack find team2_delivery)/config/<HOTSPOT_FILE_NAME>.yaml
```
- Note that the defaults for `map_file` and `hotspots_yaml` set in the launch file point to saved map and hotspot files taken from the custom turtlebot3_house.world environment. 

Wait for:
```text
HOTSPOT_EXPLORER: connected to move_base
```

Send delivery order:
```
rostopic pub /target_qr std_msgs/String "data: 'Table_2'" --once
```
- data value is case insensitive

Monitor state transitions:
```
rostopic echo /delivery_state
```
---

## Hardware Run

### Mapping
```
roslaunch team2_delivery mapping_hw.launch open_rviz:=true
```

Save map
```
rosrun map_server map_saver -f $(rospack find team2_delivery)/maps/<MAP_FILE_NAME>
```

Generate hotspots
```
rosrun team2_delivery generate_hotspots.py \
  _map_yaml:=$(rospack find team2_delivery)/maps/<MAP_FILE_NAME> \
  _output_yaml:=$(rospack find team2_delivery)/config/<HOTSPOT_FILE_NAME>.yaml
```

Delivery
```
roslaunch team2_delivery team2_delivery.launch \
  map_file:=$(rospack find team2_delivery)/maps/<MAP_FILE_NAME> \
  hotspots_yaml:=$(rospack find team2_delivery)/config/<HOTSPOT_FILE_NAME>.yaml
```

Wait for:
```text
HOTSPOT_EXPLORER: connected to move_base
```

Send delivery order:
```
rostopic pub /target_qr std_msgs/String "data: 'Table_2'" --once
```
- data value is case insensitive

# Expected Flow

```text
WAIT_FOR_ORDER -> EXPLORE -> QR_TRACK -> ARRIVED
```

- `delivery_state_machine.py` receives `/target_qr` and switches to `EXPLORE`.
- `exploration_navigator.py` visits hotspots from `hotspots.yaml`.
- At each hotspot, the robot rotates and scans for the QR.
- `qr_detector.py` publishes detected QR information.
- If the detected QR matches the target, `qr_goal_tracker.py` takes over and drives toward it.
