#ifndef POOL_EXPLORATION_ROS_HPP
#define POOL_EXPLORATION_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <pool_exploration/pool_exploration.hpp>


namespace vortex::pool_exploration{

class PoolExplorationNode : public rclcpp::Node {
public:
    PoolExplorationNode(const rclcpp::NodeOptions& options);

    ~PoolExplorationNode() {};

    geometry_msgs::msg::TransformStamped compute_map_odom_transform();

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    
    void publish_grid();

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    PoolExplorationMap map_;

    std::string odom_frame_;
    std::string map_frame_;

    Eigen::Matrix4f map_to_odom_tf_;

};

}  // namespace vortex::pool_exploration

#endif  // POOL_EXPLORATION_ROS_HPP