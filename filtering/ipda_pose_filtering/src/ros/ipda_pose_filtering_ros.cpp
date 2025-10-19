#include "ipda_pose_filtering/ros/ipda_pose_filtering_ros.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <vortex/utils/qos_profiles.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace vortex::filtering {

IPDAPoseFilteringNode::IPDAPoseFilteringNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("ipda_pose_filtering_node", options) {
    setup_publishers_and_subscribers();
}

void IPDAPoseFilteringNode::setup_publishers_and_subscribers() {
    const auto qos_sensor_data_pub{
        vortex::utils::qos_profiles::sensor_data_profile(1)};
    std::string pub_topic_name =
        this->declare_parameter<std::string>("pose_array_pub_topic");

    pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        pub_topic_name, qos_sensor_data_pub);

    target_frame_ = this->declare_parameter<std::string>("target_frame");

    std::chrono::duration<int> buffer_timeout(1);

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());

    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    std::string pose_sub_topic =
        this->declare_parameter<std::string>("pose_sub_topic");
    std::string pose_array_sub_topic =
        this->declare_parameter<std::string>("pose_array_sub_topic");

    const auto qos_sensor_data_sub{
        vortex::utils::qos_profiles::sensor_data_profile(10)};

    pose_sub_.subscribe(this, pose_sub_topic,
                        qos_sensor_data_sub.get_rmw_qos_profile());
    pose_array_sub_.subscribe(this, pose_array_sub_topic,
                              qos_sensor_data_sub.get_rmw_qos_profile());

    tf2_filter_pose_ = std::make_shared<
        tf2_ros::MessageFilter<geometry_msgs::msg::PoseStamped>>(
        pose_sub_, *tf2_buffer_, target_frame_, 100,
        this->get_node_logging_interface(), this->get_node_clock_interface());

    tf2_filter_pose_array_ =
        std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::PoseArray>>(
            pose_array_sub_, *tf2_buffer_, target_frame_, 100,
            this->get_node_logging_interface(),
            this->get_node_clock_interface());

    tf2_filter_pose_->registerCallback(
        std::bind(&IPDAPoseFilteringNode::pose_callback, this, _1));

    tf2_filter_pose_array_->registerCallback(
        std::bind(&IPDAPoseFilteringNode::pose_array_callback, this, _1));
}

void IPDAPoseFilteringNode::pose_callback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {}

void IPDAPoseFilteringNode::pose_array_callback(
    const geometry_msgs::msg::PoseArray::ConstSharedPtr msg) {}

RCLCPP_COMPONENTS_REGISTER_NODE(IPDAPoseFilteringNode);

}  // namespace vortex::filtering
