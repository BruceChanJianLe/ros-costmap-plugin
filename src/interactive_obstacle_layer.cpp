#include "ros-costmap-plugin/interactive_obstacle_layer.hpp"
#include <pluginlib/class_list_macros.hpp>


PLUGINLIB_EXPORT_CLASS(ros_costmap_plugin_namespace::interactive_layer, costmap_2d::Layer);

namespace ros_costmap_plugin_namespace
{
    interactive_layer::interactive_layer()
    {
        ;
    }


    interactive_layer::~interactive_layer()
    {
        ;
    }


    void interactive_layer::onInitialize()
    {
        // Instantiate nodehandle with specific name
        ros::NodeHandle nh("~/" + name_);
        current_ = true;
        update_ = true;

        // Setup dynamic reconfigure server
        dym_srv_ = std::make_shared<dynamic_reconfigure::Server<interactive_obstacle_layer::InteractiveObstacleLayerConfig>> (nh);
        dym_srv_->setCallback(
            [this](interactive_obstacle_layer::InteractiveObstacleLayerConfig & config, uint32_t level)
            {
                enabled_ = config.enabled;
            }
        );

        // Setup interactive marker server
        int_srv_ = std::make_shared<interactive_markers::InteractiveMarkerServer> ("interactive_server", "", false);

        // Add a single marker
        add_int_marker();
    }


    void interactive_layer::updateBounds(
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
        
        // Skip if interactive marker pose is not updated
        if(!update_)
            return;

        // Loop through points_
        for(auto itr = points_.begin(); itr != points_.end(); itr++)
        {
            // Limit the update bounds to save computational waste
            *min_x = std::min(*min_x, (*itr).second.x);
            *min_y = std::min(*min_y, (*itr).second.y);
            *max_x = std::max(*max_x, (*itr).second.x);
            *max_y = std::max(*max_y, (*itr).second.y);
        }
    }


    void interactive_layer::updateCosts(
        costmap_2d::Costmap2D & master_grid,
        int min_i, 
        int min_j,
        int max_i,
        int max_j
    )
    {
        // Process callbacks
        ros::spinOnce();

        // Disable functionality if not enabled
        if(!enabled_)
            return;

        unsigned int mx, my;
        // Loop through points_
        for(auto itr = points_.begin(); itr != points_.end(); itr++)
        {
            // Set master costmap_2d value
            if(master_grid.worldToMap((*itr).second.x, (*itr).second.y, mx, my))
                master_grid.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
        }
    }


    void interactive_layer::add_int_marker()
    {
        // Prepare an interactive marker
        prepare_int_marker("obstacle_1", "Obstacle 1");
    }


    void interactive_layer::prepare_int_marker(const std::string & marker_name, const std::string & description)
    {
        // Set frame_id for interactive marker
        int_marker_msg_.header.frame_id = layered_costmap_->getGlobalFrameID();

        // Set interactive marker scale
        int_marker_msg_.scale = 0.75;

        // Set interactive marker position
        int_marker_msg_.pose.position.x = request_random_double(-5.0, 5.0);
        int_marker_msg_.pose.position.y = request_random_double(-5.0, 5.0);

        // Prepare visual marker for interactive marker

            // Marker visual
            marker_msg_.type = visualization_msgs::Marker::CUBE;

            // marker scale (obtain from interactive marker)
            marker_msg_.scale.x = int_marker_msg_.scale * 0.30;
            marker_msg_.scale.y = int_marker_msg_.scale * 0.30;
            marker_msg_.scale.z = int_marker_msg_.scale * 0.30;

            // Marker color
            marker_msg_.color.r = 0.5;
            marker_msg_.color.g = 0.5;
            marker_msg_.color.b = 0.5;
            marker_msg_.color.a = 1.0;

            // Prepare visualization for interactive marker
            viz_int_marker_msg_.always_visible = true;
            viz_int_marker_msg_.markers.emplace_back(marker_msg_);

        // Planar Marker
        int_marker_msg_.name = marker_name;
        int_marker_msg_.description = description;

        // Display control (For planar movement)

            // Planar movement control
            con_int_marker_msg_.name = "planar_control";
            con_int_marker_msg_.orientation.x = 0;
            con_int_marker_msg_.orientation.y = 1;
            con_int_marker_msg_.orientation.z = 0;
            con_int_marker_msg_.orientation.w = 1;
            con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

                // Attach control to interactive marker
                int_marker_msg_.controls.emplace_back(con_int_marker_msg_);

            // Prepare visualization marker to attach to control marker
            con_int_marker_msg_.markers.emplace_back(marker_msg_);
            con_int_marker_msg_.always_visible = true;

                // Attach control marker with visualize marker in it to interactive marker
                int_marker_msg_.controls.emplace_back(con_int_marker_msg_);

        // Display control (For menu)

            // Menu control
            con_int_marker_msg_.name = "menu_control";
            con_int_marker_msg_.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
            con_int_marker_msg_.always_visible = true;

            // Attach control to interactive marker
            int_marker_msg_.controls.emplace_back(con_int_marker_msg_);

        // Insert interactive marker into interactive marker server
        insert_int_marker(int_marker_msg_.name);

        // Add marker to points map
        point_ temp_point; temp_point.x = int_marker_msg_.pose.position.x; int_marker_msg_.pose.position.y;
        points_[marker_name] = temp_point;
    }


    void interactive_layer::insert_int_marker(const std::string & marker_name)
    {
        // Insert marker
        int_srv_->insert(int_marker_msg_);

        // Prepare lambda function with swallow argument (marker name)
        auto int_server_callback =
            [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback, std::string name) -> void
            {
                // Pose update event
                if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE)
                {

                    // Update the marker position to point position
                    point_ temp_point; temp_point.x = feedback->pose.position.x; temp_point.y = feedback->pose.position.y;
                    this->points_[name] = temp_point;

                    // Set update_ to true
                    this->update_ = true;
                }
            };

        // Set marker callback
        int_srv_->setCallback(
            int_marker_msg_.name,
            [this, int_server_callback, marker_name](const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback)
            {
                // Use callback function
                int_server_callback(feedback, marker_name);
            }
        );

        // Commit and apply changes
        int_srv_->applyChanges();
    }

    // Method that return a random double between the region you specify
    double interactive_layer::request_random_double(double min, double max)
    {
        // Instantiate random device to generate random number
        std::random_device rd;
        std::mt19937 eng(rd());
        std::uniform_real_distribution<> distr_double(min, max);

        return distr_double(eng);
    }
} // namespace ros_costmap_plugin_namespace
