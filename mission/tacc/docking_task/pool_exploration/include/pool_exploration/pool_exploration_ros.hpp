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

        // Lager en transformasjon TF map->odom
        geometry_msgs::msg::TransformStamped compute_map_odom_transform();

        // callbacks
        void line_callback(
            const vortex_msgs::msg::LineSegment2DArray::ConstSharedPtr& msg);

        void timer_callback();

        // helpers (se på disse nærmere)
        static std::vector<LineSegment> toMapSegments2D( //static?
            const vortex_msgs::msg::LineSegment2DArray& msg,
            const Eigen::Matrix4f& T_map_src); 

        void publish_grid();

        void sendDockingWaypoint(const Eigen::Vector2f& docking_estimate);
        bool waypoint_sent_{false};

        // parametere - endre disse senere?
        double size_x_{10.0};
        double size_y_{10.0};
        double resolution_{0.1};

        double min_dist_{0.0f};
        double max_dist_{50.0f};
        double angle_threshold_{0.3f};
        double min_angle_{0.7f};
        double max_angle_{2.4f};
        double right_wall_offset_{0.5f};
        double far_wall_offset_{0.5f};

        // frames
        std::string odom_frame_;
        std::string map_frame_;
        std::string sonar_frame_;
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
      //rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped> 
      //    pose_sub_;

        // publisher
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr 
            map_pub_;

        rclcpp::TimerBase::SharedPtr timer_;

        // service client
        rclcpp::Client<vortex_msgs::srv::SendWaypoints>::SharedPtr waypoint_client_;

        // map
        PoolExplorationMap map_;

    };

    }  // namespace vortex::pool_exploration

    #endif  // POOL_EXPLORATION_ROS_HPP

    // TO DO 
    // riktig transform
    // service client
    // pose_subscriber
    // se på map publisheren
