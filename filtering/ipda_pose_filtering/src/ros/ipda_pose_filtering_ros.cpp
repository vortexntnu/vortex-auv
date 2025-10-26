#include "ipda_pose_filtering/ros/ipda_pose_filtering_ros.hpp"
#include <spdlog/spdlog.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <vortex/utils/math.hpp>
#include <vortex/utils/qos_profiles.hpp>
#include "vortex/utils/ros_conversions.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace vortex::filtering {

IPDAPoseFilteringNode::IPDAPoseFilteringNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("ipda_pose_filtering_node", options) {
    setup_publishers_and_subscribers();
    setup_track_manager();
}

void IPDAPoseFilteringNode::setup_publishers_and_subscribers() {
    const auto qos_sensor_data_pub{
        vortex::utils::qos_profiles::sensor_data_profile(1)};
    std::string pub_topic_name =
        this->declare_parameter<std::string>("pose_array_pub_topic");

    pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        pub_topic_name, qos_sensor_data_pub);

    int publish_interval_ms =
        this->declare_parameter<int>("publish_interval_ms");

    pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(publish_interval_ms),
        std::bind(&IPDAPoseFilteringNode::timer_callback, this));

    target_frame_ = this->declare_parameter<std::string>("target_frame");

    std::chrono::duration<int> buffer_timeout(1);

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());

    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    const std::string msg_type =
        this->declare_parameter<std::string>("pose_message_type");
    const std::string pose_sub_topic =
        this->declare_parameter<std::string>("pose_sub_topic");

    const auto qos_sensor_data_sub{
        vortex::utils::qos_profiles::sensor_data_profile(10)};

    if (msg_type == "PoseStamped") {
        create_pose_subscription<geometry_msgs::msg::PoseStamped>(
            pose_sub_topic, qos_sensor_data_sub.get_rmw_qos_profile());
    } else if (msg_type == "PoseArray") {
        create_pose_subscription<geometry_msgs::msg::PoseArray>(
            pose_sub_topic, qos_sensor_data_sub.get_rmw_qos_profile());
    } else if (msg_type == "PoseWithCovarianceStamped") {
        create_pose_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_sub_topic, qos_sensor_data_sub.get_rmw_qos_profile());
    } else {
        spdlog::error(
            "IPDAPoseFilteringNode: Unsupported pose message type: {} \n"
            "Supported types are: Pose, PoseStamped, PoseArray, "
            "PoseWithCovarianceStamped",
            msg_type);
    }
}

void IPDAPoseFilteringNode::setup_track_manager() {
    TrackManagerConfig config;

    config.ipda.ipda.prob_of_survival =
        this->declare_parameter<double>("ipda.prob_of_survival");
    config.ipda.pdaf.prob_of_detection =
        this->declare_parameter<double>("ipda.prob_of_detection");
    config.ipda.pdaf.mahalanobis_threshold =
        this->declare_parameter<double>("ipda.mahalanobis_gate_threshold");
    config.ipda.pdaf.min_gate_threshold =
        this->declare_parameter<double>("ipda.min_gate_threshold");
    config.ipda.pdaf.max_gate_threshold =
        this->declare_parameter<double>("ipda.max_gate_threshold");
    config.ipda.pdaf.clutter_intensity =
        this->declare_parameter<double>("ipda.clutter_intensity");

    config.dyn_mod.std_pos = this->declare_parameter<double>("dyn_mod.std_pos");
    config.dyn_mod.std_orient =
        this->declare_parameter<double>("dyn_mod.std_orient");

    config.sensor_mod.std_pos =
        this->declare_parameter<double>("sensor_mod.std_pos");
    config.sensor_mod.std_orient =
        this->declare_parameter<double>("sensor_mod.std_orient");
    config.sensor_mod.max_angle_gate_threshold =
        this->declare_parameter<double>("sensor_mod.max_angle_gate_threshold");

    config.existence.confirmation_threshold =
        this->declare_parameter<double>("existence.confirmation_threshold");
    config.existence.deletion_threshold =
        this->declare_parameter<double>("existence.deletion_threshold");
    config.existence.initial_existence_probability =
        this->declare_parameter<double>(
            "existence.initial_existence_probability");

    track_manager_ = std::make_unique<IPDAPoseTrackManager>(config);
}

template <typename MsgT>
void IPDAPoseFilteringNode::create_pose_subscription(
    const std::string& topic_name,
    const rmw_qos_profile_t& qos_profile) {
    auto sub = std::make_shared<message_filters::Subscriber<MsgT>>(
        this, topic_name, qos_profile);

    auto filter = std::make_shared<tf2_ros::MessageFilter<MsgT>>(
        *sub, *tf2_buffer_, target_frame_, 100,
        this->get_node_logging_interface(), this->get_node_clock_interface());

    filter->registerCallback([this](const typename MsgT::ConstSharedPtr msg) {
        this->pose_callback<MsgT>(msg);
    });

    subscriber_ = sub;
    tf_filter_ = filter;
}

template <typename MsgT>
void IPDAPoseFilteringNode::pose_callback(
    const typename MsgT::ConstSharedPtr& msg) {
    Eigen::Matrix<double, 6, Eigen::Dynamic> measurements =
        vortex::utils::ros_conversions::ros_to_eigen6d(*msg);
};

void IPDAPoseFilteringNode::timer_callback() {}

template void
IPDAPoseFilteringNode::pose_callback<geometry_msgs::msg::PoseStamped>(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);

template void
IPDAPoseFilteringNode::pose_callback<geometry_msgs::msg::PoseArray>(
    const geometry_msgs::msg::PoseArray::ConstSharedPtr& msg);

template void IPDAPoseFilteringNode::pose_callback<
    geometry_msgs::msg::PoseWithCovarianceStamped>(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg);

RCLCPP_COMPONENTS_REGISTER_NODE(IPDAPoseFilteringNode);

}  // namespace vortex::filtering
