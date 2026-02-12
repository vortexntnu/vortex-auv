#ifndef POOL_EXPLORATION_ROS_HPP
#define POOL_EXPLORATION_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <pool_exploration/pool_exploration.hpp>


namespace vortex::pool_exploration{

class PoolExplorationNode : public rclcpp::Node {
public:
    PoolExplorationNode(const rclcpp::NodeOptions& options);

    ~PoolExplorationNode() {};

private:
    void publish_grid();

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    PoolExplorationMap map_;

};

}  // namespace vortex::pool_exploration

#endif  // POOL_EXPLORATION_ROS_HPP