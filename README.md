# ROS Costmap Plugin

This repository demonstrates the means to write a personal costmap plugin layer.

Here there will be three layers where two layers are just implementation from the roswiki while the third one is a interactive costmap layer.  

## Creating a Costmap Plugin

**Step 1:**  
Create a package with all the needed dependencies.  
```bash
# Minimal
catkin_create_pkg ros-costmap-plugin roscpp rospy costmap_2d dynamic_reconfigure

# With Grid Map and Interactive Markers
catkin_create_pkg ros-costmap-plugin roscpp rospy costmap_2d dynamic_reconfigure grid_map_ros interactive_markers
```

## Reference

- Costmap_2D v.s. Occupancy_Grid [link](https://answers.ros.org/question/60026/difference-between-costmap2d-and-occupancygrid-not-clear/)
- Costmap_2D v.s Grid Map [link](https://github.com/stonier/cost_map)
- Detailed Explanation on Costmap_2D [link](https://www.programmersought.com/article/7101361061/)
- 3 Important Costmap Plugin Function Explained [link](https://github.com/ros-planning/navigation2/pull/1541/files/0b6c7483bbd54d697c1f78c855e68bfd0e94e55e)
