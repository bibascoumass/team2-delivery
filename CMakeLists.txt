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
  tf
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
    tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(team2_model_push SHARED plugins/model_push.cc)
target_link_libraries(team2_model_push ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
