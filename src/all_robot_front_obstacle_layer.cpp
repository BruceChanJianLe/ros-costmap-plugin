#include "ros-costmap-plugin/all_robot_front_obstacle_layer.hpp"
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ros_costmap_plugin_namespace::all_robot_front_obstacle_layer, costmap_2d::Layer);

namespace ros_costmap_plugin_namespace
{
    all_robot_front_obstacle_layer::all_robot_front_obstacle_layer()
    {
        ;
    }


    all_robot_front_obstacle_layer::~all_robot_front_obstacle_layer()
    {
        ;
    }


    void all_robot_front_obstacle_layer::matchSize()
    {
        // Obtain master costmap
        Costmap2D* master = layered_costmap_->getCostmap();
        // Resize our costmap according to master costmap
        resizeMap(
            master->getSizeInCellsX(),
            master->getSizeInCellsY(),
            master->getResolution(),
            master->getOriginX(),
            master->getOriginY()
        );
    }


    void all_robot_front_obstacle_layer::onInitialize()
    {
        // Instantiate nodehandle with specific name
        ros::NodeHandle nh("~/" + name_);
        current_ = true;
        default_value_ = costmap_2d::NO_INFORMATION;
        matchSize();

        // Setup dynamic reconfigure server
        dym_srv_ = std::make_shared<dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>> (nh);
        dym_srv_->setCallback(
            [this](costmap_2d::GenericPluginConfig & config, uint32_t level)
            {
                enabled_ = config.enabled;
            }
        );
    }


    void all_robot_front_obstacle_layer::updateBounds(
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

        // Obtain current front position of robot and set its value in costmap_
        double mark_x = robot_x + std::cos(robot_yaw), mark_y = robot_y + std::sin(robot_yaw);
        unsigned int mx, my;
        if(worldToMap(mark_x, mark_y, mx, my))
            setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);

        // Limit the update bounds to save computational waste
        *min_x = std::min(*min_x, mark_x);
        *min_y = std::min(*min_y, mark_y);
        *max_x = std::max(*max_x, mark_x);
        *max_y = std::max(*max_y, mark_y);
    }


    void all_robot_front_obstacle_layer::updateCosts(
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
        
        // Set master costmap_2d value to be equal to local costmap_
        for(int j = min_j; j < max_j; j++)
        {
            for(int i = min_i; i < max_i; i++)
            {
                int idx = getIndex(i, j);
                if(costmap_[idx] == costmap_2d::NO_INFORMATION)
                    continue;
                master_grid.setCost(i, j, costmap_[idx]);
            }
        }
    }
} // namespace ros_costmap_plugin_namespace