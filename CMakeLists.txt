cmake_minimum_required(VERSION 3.0.2)
project(team2_delivery)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  move_base_msgs
  actionlib
  actionlib_msgs
  cv_bridge
  message_generation
  tf
)

add_message_files(
  FILES
  QRInfo.msg
  SafetyStatus.msg
  Observation.msg
)

add_action_files(
  FILES
  ScanTable.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    move_base_msgs
    actionlib
    actionlib_msgs
    cv_bridge
    message_runtime
    tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
