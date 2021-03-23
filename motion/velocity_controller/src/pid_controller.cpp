#include "ros/ros.h"
#include "nav_msgs/Odometry.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "velocity_controller");
    ros::NodeHandle ros_node;

    std::string odometry_topic;
    if (!ros_node.getParam("/velocity_controller/odometry_topic", odometry_topic))
        odometry_topic = "/odometry/filtered";

    std::string thrust_topic;
    if (!ros_node.getParam("/velocity_controller/thrust_topic", thrust_topic))
        thrust_topic = "/thrust/desired";

    std::string desired_velocity_topic;
    if (!ros_node.getParam("velocity_controller/desired_velocity_topic", desired_velocity_topic))
        desired_velocity_topic = "/controller/desired_velocity";

    
    
}