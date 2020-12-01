#include "ros-costmap-plugin/single_robot_front_obstacle_layer.hpp"
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ros_costmap_plugin_namespace::single_robot_front_obstacle_layer, costmap_2d::Layer);

namespace ros_costmap_plugin_namespace
{
    single_robot_front_obstacle_layer::single_robot_front_obstacle_layer()
    {
        ;
    }


    single_robot_front_obstacle_layer::~single_robot_front_obstacle_layer()
    {
        ;
    }


    void single_robot_front_obstacle_layer::onInitialize()
    {
        // Instantiate nodehandle with specific name
        ros::NodeHandle nh("~/" + name_);
        current_ = true;

        // Setup dynamic reconfigure server
        dym_srv_ = std::make_shared<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>> (nh);
        dym_srv_->setCallback(
            [this](costmap_2d::GenericPluginConfig & config, uint32_t level)
            {
                enabled_ = config.enabled;
            }
        );
    }


    void single_robot_front_obstacle_layer::updateBounds(
        double robot_x,
        double robot_y,
        double robot_yaw,
        double * min_x,
        double * min_y,
        double * max_x,
        double * max_y
    )
    {
        // Disable functionality if not enabled
        if(!enabled_)
            return;

        // Obtain current front position of robot
        mark_x_ = robot_x + std::cos(robot_yaw);
        mark_y_ = robot_y + std::sin(robot_yaw);

        // Limit the update bounds to save computational waste
        *min_x = std::min(*min_x, mark_x_);
        *min_y = std::min(*min_y, mark_y_);
        *max_x = std::max(*max_x, mark_x_);
        *max_y = std::max(*max_y, mark_y_);
    }


    void single_robot_front_obstacle_layer::updateCosts(
        costmap_2d::Costmap2D & master_grid,
        int min_i, 
        int min_j,
        int max_i,
        int max_j
    )
    {
        // Disable functionality if not enabled
        if(!enabled_)
            return;
        
        // Set master costmap_2d value
        unsigned int mx, my;
        if(master_grid.worldToMap(mark_x_, mark_y_, mx, my))
            master_grid.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
    }
} // namespace ros_costmap_plugin_namespace
