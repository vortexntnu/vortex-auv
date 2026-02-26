#include "pose_filtering/ros/pose_filtering_ros.hpp"
#include <spdlog/spdlog.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <vortex/utils/math.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>
#include "pose_filtering/ros/pose_filtering_ros_conversions.hpp"
#include "vortex/utils/ros/ros_conversions.hpp"
#include "vortex/utils/ros/ros_transforms.hpp"

namespace vortex::filtering {

PoseFilteringNode::PoseFilteringNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("pose_filtering_node", options) {
    setup_publishers_and_subscribers();
    setup_track_manager();
}

void PoseFilteringNode::setup_publishers_and_subscribers() {
    enu_ned_rotation_ = this->declare_parameter<bool>("enu_ned_rotation");
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

    filter_dt_ = std::chrono::milliseconds(
        this->declare_parameter<int>("timer_rate_ms"));

    pub_timer_ = this->create_wall_timer(filter_dt_,
                                         [this]() { this->timer_callback(); });

    target_frame_ = this->declare_parameter<std::string>("target_frame");

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

    filter->registerCallback(
        [this](const typename PoseMsgT::ConstSharedPtr msg) {
            PoseMsgT pose_tf;
            try {
                vortex::utils::ros_transforms::transform_pose(
                    *tf2_buffer_, *msg, target_frame_, pose_tf);
            } catch (const tf2::TransformException& ex) {
                spdlog::warn("TF transform failed from '{}' to '{}': {}",
                             msg->header.frame_id.c_str(),
                             target_frame_.c_str(), ex.what());
                return;
            }

            this->measurements_ =
                vortex::filtering::ros_conversions::ros_to_landmarks(pose_tf);
            if (enu_ned_rotation_) {
                std::ranges::for_each(this->measurements_, [](auto& m) {
                    m.pose.set_ori(vortex::utils::math::enu_ned_rotation(
                        m.pose.ori_quaternion()));
                });
            }
        });

    subscriber_ = sub;
    tf_filter_ = filter;
}

void PoseFilteringNode::setup_track_manager() {
    TrackManagerConfig config;

    config.existence.confirmation_threshold = this->declare_parameter<double>(
        "config.existence.confirmation_threshold");
    config.existence.deletion_threshold =
        this->declare_parameter<double>("config.existence.deletion_threshold");
    config.existence.initial_existence_probability =
        this->declare_parameter<double>(
            "config.existence.initial_existence_probability");

    config.default_class_config.min_pos_error =
        this->declare_parameter<double>("config.gate.min_pos_error");
    config.default_class_config.max_pos_error =
        this->declare_parameter<double>("config.gate.max_pos_error");
    config.default_class_config.min_ori_error =
        this->declare_parameter<double>("config.gate.min_ori_error");
    config.default_class_config.max_ori_error =
        this->declare_parameter<double>("config.gate.max_ori_error");

    config.default_class_config.dyn_std_dev =
        this->declare_parameter<double>("config.dyn_mod_std_dev");
    config.default_class_config.sens_std_dev =
        this->declare_parameter<double>("config.sens_mod_std_dev");

    config.default_class_config.init_pos_std =
        this->declare_parameter<double>("config.init_pos_std_dev");
    config.default_class_config.init_ori_std =
        this->declare_parameter<double>("config.init_ori_std_dev");

    config.default_class_config.mahalanobis_threshold =
        this->declare_parameter<double>("config.mahalanobis_gate_threshold");
    config.default_class_config.prob_of_detection =
        this->declare_parameter<double>("config.prob_of_detection");
    config.default_class_config.clutter_intensity =
        this->declare_parameter<double>("config.clutter_intensity");
    config.default_class_config.prob_of_survival =
        this->declare_parameter<double>("config.prob_of_survival");
    config.default_class_config.estimate_clutter =
        this->declare_parameter<bool>("config.estimate_clutter");

    track_manager_ = std::make_unique<PoseTrackManager>(config);
}

void PoseFilteringNode::timer_callback() {
    if (debug_) {
        publish_meas_debug();
    }
    track_manager_->step(measurements_,
                         std::chrono::duration<double>(filter_dt_).count());
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
