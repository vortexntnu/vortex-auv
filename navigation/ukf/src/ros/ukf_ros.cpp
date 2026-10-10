#include "ukf/ros/ukf_ros.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>
#include <vortex/utils/ros/ros_conversions.hpp>

UKFNode::UKFNode(const rclcpp::NodeOptions& options)
    : Node("ukf_node", options) {
    set_parameters();
    set_subscribers_and_publishers();
}

void UKFNode::set_parameters() {
    // TODO: noise, prior, unscented transform and environment parameters,
    // then build the models and the filter
}

void UKFNode::set_subscribers_and_publishers() {
    // TODO: same topics as the eskf, publish under ukf/ to run side by side
}

void UKFNode::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
    // TODO: dt from stamps, imu into base_link, predict
}

void UKFNode::dvl_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg) {
    // TODO: dvl into base_link minus lever arm velocity, update, nis
}

void UKFNode::pressure_callback(
    const sensor_msgs::msg::FluidPressure::ConstSharedPtr msg) {
    // TODO: pressure to depth, update, nis
}

void UKFNode::magnetometer_callback(
    const sensor_msgs::msg::MagneticField::ConstSharedPtr msg) {
    // TODO: field into base_link, update, nis
}

void UKFNode::publish_odom() {
    // TODO: odom from estimate_, covariances like eskf/output.hpp
}

RCLCPP_COMPONENTS_REGISTER_NODE(UKFNode)
