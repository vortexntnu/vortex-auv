#ifndef POOL_EXPLORATION_ROS_HPP
#define POOL_EXPLORATION_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class MappingNode : public rclcpp::Node {
public:
    MappingNode(const rclcpp::NodeOptions& options);

    ~MappingNode() {};

private:
    //Occupancy grid
    void initialize_grid();
    void publish_grid();

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    nav_msgs::msg::OccupancyGrid grid_msg_;

    //Parameters for Occupancy grid
    double size_x_;
    double size_y_;
    double resolution_;
    std::string frame_id_;

};

#endif  // POOL_EXPLORATION_ROS_HPP