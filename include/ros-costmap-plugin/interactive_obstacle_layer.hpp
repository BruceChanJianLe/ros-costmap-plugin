#ifndef interactive_obstacle_layer_H_
#define interactive_obstacle_layer_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

// Dynamic Reconfigure Feature
#include <dynamic_reconfigure/server.h>
#include <ros-costmap-plugin/InteractiveObstacleLayerConfig.h>

// Interactive Marker Feature
#include <interactive_markers/interactive_marker_server.h>

#include <memory>
#include <map>
#include <random>


namespace ros_costmap_plugin_namespace
{
    struct point_
    {
        double x, y;
    };
    
    class interactive_layer : public costmap_2d::Layer, public costmap_2d::Costmap2D
    {
        public:
            // Constructor & destructor
            interactive_layer();
            ~interactive_layer();

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
            std::shared_ptr<dynamic_reconfigure::Server<interactive_obstacle_layer::InteractiveObstacleLayerConfig>> dym_srv_;
            std::shared_ptr<interactive_markers::InteractiveMarkerServer> int_srv_;
            visualization_msgs::InteractiveMarker int_marker_msg_;
            visualization_msgs::Marker marker_msg_;
            visualization_msgs::InteractiveMarkerControl viz_int_marker_msg_;
            visualization_msgs::InteractiveMarkerControl con_int_marker_msg_;

            // If marker position updated
            bool update_;
            std::map<std::string, point_> points_;

            // Add interactive marker
            void add_int_marker();
            void prepare_int_marker(const std::string &, const std::string &);
            void insert_int_marker(const std::string &);
            double request_random_double(double, double);
    };
} // namespace ros_costmap_plugin_namespace


#endif