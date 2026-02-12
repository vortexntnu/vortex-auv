#include <pool_exploration/pool_exploration_ros.hpp>

namespace vortex::pool_exploration{

PoolExplorationNode::PoolExplorationNode(const rclcpp::NodeOptions& options)
    : Node("pool_exploration_node", options) {

    double size_x = this->declare_parameter<double>("size_x", 10.0);
    double size_y = this->declare_parameter<double>("size_y", 10.0);
    double resolution = this->declare_parameter<double>("resolution", 0.1);
    std::string frame_id = this->declare_parameter<std::string>("frame_id", "map");
    
    map_ = PoolExplorationMap(size_x, size_y, resolution, frame_id);

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

    publish_grid();
}


void PoolExplorationNode::publish_grid()
{
    auto grid_msg = map_.grid();
    grid_msg_.header.stamp = now();
    map_pub_->publish(grid_msg_);
}

}  // namespace vortex::pool_exploration
