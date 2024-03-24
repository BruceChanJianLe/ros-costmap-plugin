cmake_minimum_required(VERSION 3.0.2)
project(ros-costmap-plugin)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

# Generate dynamic reconfigure parameters in the 'cfg' folder
# Must be called before catkin_package()
generate_dynamic_reconfigure_options(
  cfg/InteractiveObstacleLayer.cfg
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

# Create all_robot_front_obstacle_layer
add_library(all_obstacle_layer src/all_robot_front_obstacle_layer.cpp)

# Create interactive_obstacle_layer
add_library(interactive_obstacle_layer src/interactive_obstacle_layer.cpp)
add_dependencies(interactive_obstacle_layer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(interactive_obstacle_layer ${catkin_LIBRARIES})

# Install costmap_plugins.xml
install(FILES costmap_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install library
install(TARGETS single_obstacle_layer all_obstacle_layer interactive_obstacle_layer
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install include directories
install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
