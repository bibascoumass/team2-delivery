# Hardware Run Order

This is the command order for running saved-map hotspot search on the Triton robot.

## 1. Copy Required Files From Local WSL

Run from local WSL:

```bash
scp ~/catkin_ws/my_map_explore.* ~/catkin_ws/hotspots.yaml \
  triton@hcr-triton-9:~/catkin_ws/

scp ~/catkin_ws/src/team2_delivery/launch/mapping_hw.launch \
  triton@hcr-triton-9:~/catkin_ws/src/team2_delivery/launch/

scp ~/catkin_ws/src/team2_delivery/launch/hardware_delivery.launch \
  triton@hcr-triton-9:~/catkin_ws/src/team2_delivery/launch/

scp ~/catkin_ws/src/team2_delivery/scripts/{exploration_navigator.py,delivery_state_machine.py,cmd_mux.py,qr_detector.py,qr_goal_tracker.py,hotspot_visualizer.py} \
  triton@hcr-triton-9:~/catkin_ws/src/team2_delivery/scripts/
```

## 2. Start Hardware Delivery/Search Stack

On the robot:

```bash
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

roslaunch team2_delivery hardware_delivery.launch
```

This starts the real robot sensors/base bridge, publishes the saved map, starts AMCL, starts `move_base`, starts the hotspot explorer, and starts the QR search/tracking nodes.

Optional arguments:

```bash
roslaunch team2_delivery hardware_delivery.launch \
  map_file:=/home/triton/catkin_ws/my_map_explore.yaml \
  hotspots_yaml:=/home/triton/catkin_ws/hotspots.yaml \
  open_rviz:=false
```

If the camera topics are different:

```bash
roslaunch team2_delivery hardware_delivery.launch \
  image_topic:=/your/rgb/image_topic \
  depth_topic:=/your/depth/image_topic
```

Wait for:

```text
HOTSPOT_EXPLORER: connected to move_base
```

## 3. Send Target QR

Publish the QR text to search for:

```bash
rostopic pub /target_qr std_msgs/String "data: 'table_1'" -1
```

Replace `table_1` with the actual QR code text.

## Expected Flow

```text
WAIT_FOR_ORDER -> EXPLORE -> QR_TRACK -> ARRIVED
```

- `delivery_state_machine.py` receives `/target_qr` and switches to `EXPLORE`.
- `exploration_navigator.py` visits hotspots from `hotspots.yaml`.
- At each hotspot, the robot rotates and scans for the QR.
- `qr_detector.py` publishes detected QR information.
- If the detected QR matches the target, `qr_goal_tracker.py` takes over and drives toward it.

## Notes

- The robot uses ROS Melodic, so run nodes with `python`/`rosrun`, not `python3`.
- `table_1` is the QR text to search for, not a predefined map coordinate.
- If `qr_detector.py` complains about `pyzbar`, install:

```bash
sudo apt install -y libzbar0 python-pip python-setuptools
python -m pip install --user pyzbar==0.1.9
```

## Manual Fallback

If `hardware_delivery.launch` is too much at once, start navigation first:

```bash
roslaunch team2_delivery mapping_hw.launch
```

Then start these nodes manually:

```bash
rosrun team2_delivery delivery_state_machine.py
rosrun team2_delivery cmd_mux.py
rosrun team2_delivery exploration_navigator.py _hotspots_yaml:=/home/triton/catkin_ws/hotspots.yaml
rosrun team2_delivery qr_detector.py
rosrun team2_delivery qr_goal_tracker.py
```
