#ifndef POOL_EXPLORATION_ROS_HPP
#define POOL_EXPLORATION_ROS_HPP

#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h> // skal ha med?
#include <tf2_eigen/tf2_eigen.h> // skal ha med?

#include <chrono> // trenger denne?
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <vortex_msgs/msg/line_segment2_d_array.hpp>
#include <vortex_msgs/msg/waypoint.hpp>
#include <vortex_msgs/srv/send_waypoints.hpp>

// trenger disse to?
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pool_exploration/pool_exploration.hpp>


// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <rclcpp/publisher.hpp>

// #include <vortex_msgs/srv/detail/send_waypoints__struct.hpp>

// #include <rmw/types.h>

namespace vortex::pool_exploration{

class PoolExplorationNode : public rclcpp::Node {
    public:
        explicit PoolExplorationNode(const rclcpp::NodeOptions& options);
        ~PoolExplorationNode() = default;

    private:
        // setup
        void setup_publishers_and_subscribers();
        void setup_planner();

        // callbacks
        void line_callback(
            const vortex_msgs::msg::LineSegment2DArray::ConstSharedPtr& msg);

        void pose_callback(
            const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg);

        // helpers (se på disse nærmere)
        static std::vector<LineSegment> transform_segments_2d( //static?
            const vortex_msgs::msg::LineSegment2DArray& msg,
            const Eigen::Matrix4f& T_target_src); 

        void send_docking_waypoint(const Eigen::Vector2f& docking_estimate);
        bool waypoint_sent_{false};
        
        //kaller på sendwaypoint service og se
        void estimate_and_send_docking_waypoint(const vortex_msgs::msg::LineSegment2DArray& msg);

        // frames
        std::string odom_frame_;
        std::string map_frame_;
        std::string base_frame_;
        std::chrono::milliseconds pub_dt_{200};

        // TF publishing
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

        // TF listening
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_; //Hvorfor bruke buffer
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // subscriber
        // Har gjor tlitt annen filtering uten peker enn line_filtering eksempel
        message_filters::Subscriber<vortex_msgs::msg::LineSegment2DArray> line_sub_;
        std::shared_ptr<tf2_ros::MessageFilter<vortex_msgs::msg::LineSegment2DArray>> line_filter_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

        rclcpp::TimerBase::SharedPtr timer_;

        // service client
        rclcpp::Client<vortex_msgs::srv::SendWaypoints>::SharedPtr waypoint_client_;

        // Position and heading
        DroneState drone_state_;

        std::unique_ptr<PoolExplorationPlanner> planner_;

    // GRID LOGIKK TIL SENERE
    # if 0
    private:

        void timer_callback();
        // Lager en transformasjon TF map->odom
        geometry_msgs::msg::TransformStamped compute_map_odom_transform();
        
        void publish_grid();

        // Logikk til senere bruk:
        void draw_segments_in_map_frame(const vortex_msgs::msg::LineSegment2DArray& msg);

        // publisher
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr 
            map_pub_;

        // map
        PoolExplorationPlanner map_;

    #endif
    };

    }  // namespace vortex::pool_exploration

    #endif  // POOL_EXPLORATION_ROS_HPP

    // TO DO 
    // service client
    // pose_subscriber
    // se på map publisheren
    // sjekke map vs odom transformene og hvordan det er i filter
