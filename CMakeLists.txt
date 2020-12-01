cmake_minimum_required(VERSION 3.0.2)
project(ros-costmap-plugin)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  roscpp
  rospy
  # Grid Map Features
  grid_map_ros
  # Interactive Marker Features
  interactive_markers
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ros-costmap-plugin
#  CATKIN_DEPENDS costmap_2d dynamic_reconfigure grid_map_ros interactive_markers roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Create single_robot_front_obstacle_layer
add_library(single_obstacle_layer src/single_robot_front_obstacle_layer.cpp)
add_library(all_obstacle_layer src/all_robot_front_obstacle_layer.cpp)
