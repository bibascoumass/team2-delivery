Launch Simulation:
- Clone & compile https://gitlab.com/HCRLab/stingray-robotics/cs603_particle_filter 
- compile team2_delivery
- source ~/catkin_ws/devel/setup.bash
- roslaunch team2_delivery sim.launch


Run Tests:
source ~/catkin_ws/devel/setup.bash
rosrun team2_delivery test_stubs.py    

Notes:
- Robot is expected to be spawned in middle room with trashcan