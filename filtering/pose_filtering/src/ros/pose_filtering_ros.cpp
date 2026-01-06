#include "pose_filtering/ros/pose_filtering_ros.hpp"
#include <spdlog/spdlog.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <vortex/utils/math.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>
#include "pose_filtering/ros/pose_filtering_ros_conversions.hpp"
#include "vortex/utils/ros/ros_conversions.hpp"
#include "vortex/utils/ros/ros_transforms.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace vortex::filtering {

PoseFilteringNode::PoseFilteringNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("pose_filtering_node", options) {
    setup_publishers_and_subscribers();
    setup_track_manager();
}

void PoseFilteringNode::setup_publishers_and_subscribers() {
    const auto qos_sensor_data_pub{
        vortex::utils::qos_profiles::sensor_data_profile(1)};
    std::string pub_topic_name =
        this->declare_parameter<std::string>("pose_array_pub_topic");

    std::string landmark_pub_topic =
        this->declare_parameter<std::string>("landmark_pub_topic");

    pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        pub_topic_name, qos_sensor_data_pub);

    landmark_array_pub_ =
        this->create_publisher<vortex_msgs::msg::LandmarkArray>(
            landmark_pub_topic, qos_sensor_data_pub);

    int timer_rate_ms = this->declare_parameter<int>("timer_rate_ms");

    filter_dt_seconds_ = static_cast<double>(timer_rate_ms) / 1000;

    pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_rate_ms),
        std::bind(&PoseFilteringNode::timer_callback, this));

    target_frame_ = this->declare_parameter<std::string>("target_frame");

    std::chrono::duration<int> buffer_timeout(1);

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());

    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    const std::string pose_sub_topic =
        this->declare_parameter<std::string>("pose_sub_topic");

    const auto qos_sensor_data_sub{
        vortex::utils::qos_profiles::sensor_data_profile(10)};

    create_pose_subscription(pose_sub_topic,
                             qos_sensor_data_sub.get_rmw_qos_profile());

    debug_ = this->declare_parameter<bool>("debug.enable");
    if (debug_) {
        setup_debug_publishers();
    }
}

void PoseFilteringNode::create_pose_subscription(
    const std::string& topic_name,
    const rmw_qos_profile_t& qos_profile) {
    auto sub = std::make_shared<message_filters::Subscriber<PoseMsgT>>(
        this, topic_name, qos_profile);

    auto filter = std::make_shared<tf2_ros::MessageFilter<PoseMsgT>>(
        *sub, *tf2_buffer_, target_frame_, 10,
        this->get_node_logging_interface(), this->get_node_clock_interface());

    filter->registerCallback([this](
                                 const typename PoseMsgT::ConstSharedPtr msg) {
        PoseMsgT pose_tf;
        try {
            vortex::utils::ros_transforms::transform_pose(
                *tf2_buffer_, *msg, target_frame_, pose_tf);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(
                this->get_logger(), "TF transform failed from '%s' to '%s': %s",
                msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
            return;
        }

        this->measurements_ =
            vortex::utils::ros_conversions::ros_to_pose_vec(pose_tf);
    });

    subscriber_ = sub;
    tf_filter_ = filter;
}

void PoseFilteringNode::setup_track_manager() {
    TrackManagerConfig config;

    config.ipda.ipda.prob_of_survival =
        this->declare_parameter<double>("position.ipda.prob_of_survival");
    config.ipda.ipda.estimate_clutter =
        this->declare_parameter<bool>("position.ipda.estimate_clutter");
    config.ipda.pdaf.prob_of_detection =
        this->declare_parameter<double>("position.ipda.prob_of_detection");
    config.ipda.pdaf.mahalanobis_threshold = this->declare_parameter<double>(
        "position.ipda.mahalanobis_gate_threshold");
    config.ipda.pdaf.min_gate_threshold =
        this->declare_parameter<double>("position.ipda.min_gate_threshold");
    config.ipda.pdaf.max_gate_threshold =
        this->declare_parameter<double>("position.ipda.max_gate_threshold");
    config.ipda.pdaf.clutter_intensity =
        this->declare_parameter<double>("position.ipda.clutter_intensity");

    config.initial_position_std =
        this->declare_parameter<double>("position.initial_position_std");
    config.initial_orientation_std =
        this->declare_parameter<double>("orientation.initial_orientation_std");

    config.dyn_mod.std_dev =
        this->declare_parameter<double>("position.dyn_mod.std_dev");

    config.sensor_mod.std_dev =
        this->declare_parameter<double>("position.sensor_mod.std_dev");

    config.max_angle_gate_threshold =
        this->declare_parameter<double>("max_angle_gate_threshold");

    config.existence.confirmation_threshold = this->declare_parameter<double>(
        "position.existence.confirmation_threshold");
    config.existence.deletion_threshold = this->declare_parameter<double>(
        "position.existence.deletion_threshold");
    config.existence.initial_existence_probability =
        this->declare_parameter<double>(
            "position.existence.initial_existence_probability");

    config.ori.pdaf.pdaf.prob_of_detection =
        this->declare_parameter<double>("orientation.pdaf.prob_of_detection");
    config.ori.pdaf.pdaf.clutter_intensity =
        this->declare_parameter<double>("orientation.pdaf.clutter_intensity");
    config.ori.pdaf.pdaf.mahalanobis_threshold =
        this->declare_parameter<double>(
            "orientation.pdaf.mahalanobis_gate_threshold");

    config.ori.dyn_mod.std_dev =
        this->declare_parameter<double>("orientation.dyn_mod.std_dev");
    config.ori.sensor_mod.std_dev =
        this->declare_parameter<double>("orientation.sensor_mod.std_dev");
    track_manager_ = std::make_unique<PoseTrackManager>(config);
}

void PoseFilteringNode::timer_callback() {
    if (debug_) {
        publish_meas_debug();
    }
    track_manager_->step(measurements_, filter_dt_seconds_);
    measurements_.clear();
    if (debug_) {
        publish_state_debug();
    }

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = target_frame_;
    pose_array.header.stamp = this->get_clock()->now();
    for (const Track& track : track_manager_->get_tracks()) {
        if (!track.confirmed)
            continue;
        pose_array.poses.push_back(
            vortex::utils::ros_conversions::to_pose_msg(track.to_pose()));
    }

    if (pose_array.poses.empty()) {
        return;
    }
    pose_array_pub_->publish(pose_array);

    vortex_msgs::msg::LandmarkArray landmark_array_msg =
        vortex::filtering::ros_conversions::tracks_to_landmark_array_msg(
            track_manager_->get_tracks(), this->get_clock()->now(),
            target_frame_);
    landmark_array_pub_->publish(landmark_array_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(PoseFilteringNode);

}  // namespace vortex::filtering
