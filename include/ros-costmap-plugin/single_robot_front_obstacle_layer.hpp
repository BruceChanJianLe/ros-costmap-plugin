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