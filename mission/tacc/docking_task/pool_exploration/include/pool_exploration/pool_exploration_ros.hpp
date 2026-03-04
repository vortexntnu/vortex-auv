#ifndef POOL_EXPLORATION_ROS_HPP
#define POOL_EXPLORATION_ROS_HPP

#include <rclcpp/publisher.hpp>
#include <tf2_ros/buffer.hpp>
#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <pool_exploration/pool_exploration.hpp>
#include <rclcpp/timer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vortex_msgs/msg/line_segment2_d_array.hpp>
#include <vortex_msgs/srv/detail/send_waypoints__struct.hpp>
#include <vortex_msgs/srv/send_waypoints.hpp>
#include <vortex_msgs/msg/waypoint.hpp>

#include <rmw/types.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace vortex::pool_exploration{

class PoolExplorationNode : public rclcpp::Node {
public:
    explicit PoolExplorationNode(const rclcpp::NodeOptions& options);
    ~PoolExplorationNode() = default;

private:

    // setup
    void setup_publishers_and_subscribers();

    // TF map->odom
    geometry_msgs::msg::TransformStamped compute_map_odom_transform();

    // callbacks
    void line_callback(const vortex_msgs::msg::LineSegment2DArray::ConstSharedPtr& msg);
    void timer_callback();

    // helpers
    static std::vector<LineSegment> toMapSegments2D( //static?
        const vortex_msgs::msg::LineSegment2DArray& msg,
        const Eigen::Matrix4f& T_map_src); 

    void publish_grid();

    void sendDockingWaypoint(const Eigen::Vector2f& docking_estimate);
    bool waypoint_sent_{false};


    // frames
    std::string odom_frame_;
    std::string map_frame_;
    std::chrono::milliseconds pub_dt_{200};

    // TF publishing
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

    // TF listening
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_; //Hvorfor bruke buffer
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // subscriber (HVEM AV DISSE?? Bruke filter??)
    //rclcpp::Subscription<vortex_msgs::msg::LineSegment2DArray>::SharedPtr line_sub_;
    message_filters::Subscriber<vortex_msgs::msg::LineSegment2DArray> line_sub_;
    //er dette riktig subscriber
    //rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped> pose_sub_;
    std::shared_ptr<tf2_ros::MessageFilter<vortex_msgs::msg::LineSegment2DArray>> line_filter_;

    // publisher
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // service client
    rclcpp::Client<vortex_msgs::srv::SendWaypoints>::SharedPtr waypoint_client_;

    // map
    PoolExplorationMap map_;

};

}  // namespace vortex::pool_exploration

#endif  // POOL_EXPLORATION_ROS_HPP