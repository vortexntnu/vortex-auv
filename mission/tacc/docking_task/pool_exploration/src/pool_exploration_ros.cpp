#include <pool_exploration/pool_exploration_ros.hpp>


MappingNode::MappingNode(const rclcpp::NodeOptions& options)
    : Node("mapping_node", options) {

    declare_parameter("size_x", 10.0);
    declare_parameter("size_y", 10.0);
    declare_parameter("resolution", 0.1);
    declare_parameter("frame_id", std::string("map"));

    get_parameter("size_x", size_x_);
    get_parameter("size_y", size_y_);
    get_parameter("resolution", resolution_);
    get_parameter("frame_id", frame_id_);

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

    initialize_grid();
    publish_grid();
}

void MappingNode::initialize_grid() {
    int width  = static_cast<int>(size_x_ / resolution_);
    int height = static_cast<int>(size_y_ / resolution_);

    //Setup header and info
    grid_msg_.header.frame_id = frame_id_;
    grid_msg_.info.resolution = resolution_;
    grid_msg_.info.width      = width;
    grid_msg_.info.height     = height;

    //center of map, no rotation
    grid_msg_.info.origin.position.x = -size_x_ / 2.0;
    grid_msg_.info.origin.position.y = -size_y_ / 2.0;
    grid_msg_.info.origin.orientation.w = 1.0; 

    //Set all cells unknown
    grid_msg_.data.resize(width * height, -1);

}

void MappingNode::publish_grid()
{
    grid_msg_.header.stamp = now();
    map_pub_->publish(grid_msg_);
}

