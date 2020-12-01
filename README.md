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

**Step 2:**  
Update export tag in package.xml.  
```xml
<export>
    <costmap_2d plugin="${prefix}/costmap_plugins.xml" />
</export>
```

**Step 3:**  
Add header file, define the `namespace`, and three important `functions`.  
```cpp
#ifndef single_robot_front_obstacle_layer_H_
#define single_robot_front_obstacle_layer_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

#include <memory>

namespace ros_costmap_plugin_namespace
{
    class single_robot_front_obstacle_layer: public costmap_2d::Layer
    {
        public:
            // Constructor & destructor
            single_robot_front_obstacle_layer();
            ~single_robot_front_obstacle_layer();

            // Three important functions
            virtual void onInitialize();
            virtual void updateBounds(
                double robot_x,
                double robot_y,
                double robot_yaw,
                double * min_x,
                double * min_y,
                double * max_x,
                double * max_y
            );
            virtual void updateCosts(
                costmap_2d::Costmap2D & master_grid,
                int min_i, 
                int min_j,
                int max_i,
                int max_j
            );

        private:
            double mark_x_, mark_y_;
            std::shared_ptr<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>> dym_srv_;
 
    };
} // namespace ros_costmap_plugin_namespace

#endif
```

Virtual Method | Method Description | Requires Override?
--- | --- | ---
onInitialize() | (Called only once) Method is called at the end of plugin initialization. There is usually declarations of ROS parameters with its default values. | No
updateBounds() | (Call periodically) Method is called to ask the plugin: which area of costmap layer it needs to update. There are 3 input parameters of method: robot position and orientation and 4 output parameters: pointers to window bounds. These bounds are used for performance reasons: to update the area inside window where is new info available, avoiding updates of whole costmap on every iteration. |  Yes
updateCosts() | (Call periodically) Method is called each time when costmap re-calculation is required. It updates the costmap layer only within its bounds window. There are 4 input parameters of method: calculation window bounds and 1 output parameter: reference to a resulting costmap `master_grid`. To work with local costmap layer and merge it with other costmap layers, please use `costmap_` pointer instead (this is pointer to local costmap layer grid) and then call one of updates methods: `updateWithAddition()`, `updateWithMax()`, `updateWithOverwrite()` or `updateWithTrueOverwrite()`. | Yes
matchSize() | Method is called each time when map size was changed. | No
onFootprintChanged() | Method is called each time when footprint was changed. | No
reset() | It may have any code to be executed during costmap reset. | Yes

## Reference

- Costmap_2D v.s. Occupancy_Grid [link](https://answers.ros.org/question/60026/difference-between-costmap2d-and-occupancygrid-not-clear/)
- Costmap_2D v.s Grid Map [link](https://github.com/stonier/cost_map)
- Detailed Explanation on Costmap_2D [link](https://www.programmersought.com/article/7101361061/)
- 3 Important Costmap Plugin Function Explained [link](https://github.com/ros-planning/navigation2/pull/1541/files/0b6c7483bbd54d697c1f78c855e68bfd0e94e55e)
