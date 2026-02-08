#include "landmark_server/landmark_server_ros.hpp"
#include <spdlog/spdlog.h>
#include <memory>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>
#include <vortex/utils/ros/ros_transforms.hpp>

namespace vortex::mission {

LandmarkServerNode::LandmarkServerNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("landmark_server_node", options) {
    create_track_manager();
    setup_ros_communicators();

    int timer_rate_ms = this->declare_parameter<int>("timer_rate_ms");
    filter_dt_seconds_ = static_cast<double>(timer_rate_ms) / 1000;
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_rate_ms),
        std::bind(&LandmarkServerNode::timer_callback, this));
}

void LandmarkServerNode::setup_ros_communicators() {
    create_reference_publisher();
    create_pose_subscription();
    create_polling_action_server();
    create_convergence_action_server();
    create_reference_action_client();
}

void LandmarkServerNode::create_reference_publisher() {
    std::string reference_pose_topic =
        this->declare_parameter<std::string>("topics.reference_pose");
    auto qos_sensor_profile =
        vortex::utils::qos_profiles::sensor_data_profile(5);
    reference_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(
            reference_pose_topic, qos_sensor_profile);
}

void LandmarkServerNode::create_pose_subscription() {
    std::string landmark_topic =
        this->declare_parameter<std::string>("topics.landmarks");
    enu_ned_rotation_ = this->declare_parameter<bool>("enu_ned_rotation");
    target_frame_ = this->declare_parameter<std::string>("target_frame");
    auto qos_sensor_profile =
        vortex::utils::qos_profiles::sensor_data_profile(5);
    auto sub = std::make_shared<
        message_filters::Subscriber<vortex_msgs::msg::LandmarkArray>>(
        this, landmark_topic, qos_sensor_profile.get_rmw_qos_profile());

    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface());

    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    auto filter = std::make_shared<
        tf2_ros::MessageFilter<vortex_msgs::msg::LandmarkArray>>(
        *sub, *tf2_buffer_, target_frame_, 10,
        this->get_node_logging_interface(), this->get_node_clock_interface());

    filter->registerCallback(
        [this](const typename vortex_msgs::msg::LandmarkArray::ConstSharedPtr
                   msg) {
            vortex_msgs::msg::LandmarkArray pose_tf;
            try {
                vortex::utils::ros_transforms::transform_pose(
                    *tf2_buffer_, *msg, target_frame_, pose_tf);
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN(this->get_logger(),
                            "TF transform failed from '%s' to '%s': %s",
                            msg->header.frame_id.c_str(), target_frame_.c_str(),
                            ex.what());
                return;
            }

            this->measurements_ = ros_msg_to_landmarks(pose_tf);
            if (enu_ned_rotation_) {
                std::ranges::for_each(this->measurements_, [](auto& m) {
                    m.pose.set_ori(vortex::utils::math::enu_ned_rotation(
                        m.pose.ori_quaternion()));
                });
            }
        });

    landmark_sub_ = sub;
    tf_filter_ = filter;
}

void LandmarkServerNode::create_polling_action_server() {
    std::string landmark_polling_action_name =
        this->declare_parameter<std::string>("action_servers.landmark_polling");
    landmark_polling_server_ =
        rclcpp_action::create_server<vortex_msgs::action::LandmarkPolling>(
            this, landmark_polling_action_name,

            [this](auto goal_id, auto goal) {
                return handle_landmark_polling_goal(goal_id, goal);
            },

            [this](auto goal_id) {
                return handle_landmark_polling_cancel(goal_id);
            },

            [this](auto goal_handle) {
                return handle_landmark_polling_accepted(goal_handle);
            });
}

void LandmarkServerNode::create_convergence_action_server() {
    std::string landmark_convergence_action_name =
        this->declare_parameter<std::string>(
            "action_servers.landmark_convergence");
    landmark_convergence_server_ =
        rclcpp_action::create_server<vortex_msgs::action::LandmarkConvergence>(
            this, landmark_convergence_action_name,

            [this](auto goal_id, auto goal) {
                return handle_landmark_convergence_goal(goal_id, goal);
            },

            [this](auto goal_id) {
                return handle_landmark_convergence_cancel(goal_id);
            },

            [this](auto goal_handle) {
                return handle_landmark_convergence_accepted(goal_handle);
            });
}

rclcpp_action::GoalResponse
LandmarkServerNode::handle_landmark_convergence_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const vortex_msgs::action::LandmarkConvergence::Goal>
        goal_msg) {
    const int requested_id = goal_msg->id;
    if (!track_manager_->has_track(requested_id)) {
        spdlog::warn(
            "Requested landmark id for LandmarkConvergence action does not "
            "exist.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (active_reference_filter_goal_) {
        reference_filter_client_->async_cancel_goal(
            active_reference_filter_goal_);
        // active_reference_filter_goal_.reset();  // should i call reset here?
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void LandmarkServerNode::handle_landmark_convergence_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::LandmarkConvergence>> goal_handle) {
    auto status = active_reference_filter_goal_->get_status();
}

rclcpp_action::CancelResponse
LandmarkServerNode::handle_landmark_convergence_cancel(
    const std::shared_ptr<LandmarkConvergenceGoalHandle> /*goal_handle*/) {
    spdlog::info("LandmarkConvergence: cancel requested");

    if (active_reference_filter_goal_) {
        reference_filter_client_->async_cancel_goal(
            active_reference_filter_goal_);
        active_reference_filter_goal_.reset();
    }

    return rclcpp_action::CancelResponse::ACCEPT;
}

void LandmarkServerNode::create_reference_action_client() {
    std::string reference_action_name =
        this->declare_parameter<std::string>("action_servers.reference_filter");
    reference_filter_client_ = rclcpp_action::create_client<
        vortex_msgs::action::ReferenceFilterWaypoint>(this,
                                                      reference_action_name);
    if (!reference_filter_client_->wait_for_action_server(
            std::chrono::seconds(3))) {
        spdlog::warn("ReferenceFilter server not ready");
    }
}

rclcpp_action::GoalResponse LandmarkServerNode::handle_landmark_polling_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const vortex_msgs::action::LandmarkPolling::Goal>
    /*goal_msg*/) {
    if (active_landmark_polling_goal_ &&
        active_landmark_polling_goal_->is_active()) {
        auto polling_result =
            std::make_shared<vortex_msgs::action::LandmarkPolling_Result>();
        active_landmark_polling_goal_->abort(polling_result);
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void LandmarkServerNode::handle_landmark_polling_accepted(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::LandmarkPolling>>
        goal_handle) {
    spdlog::info(
        "Accepted landmark polling action for type: {} and subtype: {}",
        goal_handle->get_goal()->identifier.type,
        goal_handle->get_goal()->identifier.subtype);
    active_landmark_polling_goal_ = goal_handle;
}

rclcpp_action::CancelResponse
LandmarkServerNode::handle_landmark_polling_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::LandmarkPolling>> /*goal_handle*/) {
    spdlog::info("LandmarkPolling action cancelled");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void LandmarkServerNode::create_track_manager() {
    vortex::filtering::TrackManagerConfig config;

    config.existence.confirmation_threshold = this->declare_parameter<double>(
        "track_config.existence.confirmation_threshold");
    config.existence.deletion_threshold = this->declare_parameter<double>(
        "track_config.existence.deletion_threshold");
    config.existence.initial_existence_probability =
        this->declare_parameter<double>(
            "track_config.existence.initial_existence_probability");

    config.default_class_config.min_pos_error = this->declare_parameter<double>(
        "track_config.default.gate.min_pos_error");
    config.default_class_config.max_pos_error = this->declare_parameter<double>(
        "track_config.default.gate.max_pos_error");
    config.default_class_config.min_ori_error = this->declare_parameter<double>(
        "track_config.default.gate.min_ori_error");
    config.default_class_config.max_ori_error = this->declare_parameter<double>(
        "track_config.default.gate.max_ori_error");

    config.default_class_config.dyn_std_dev =
        this->declare_parameter<double>("track_config.default.dyn_mod_std_dev");
    config.default_class_config.sens_std_dev = this->declare_parameter<double>(
        "track_config.default.sens_mod_std_dev");

    config.default_class_config.init_pos_std = this->declare_parameter<double>(
        "track_config.default.init_pos_std_dev");
    config.default_class_config.init_ori_std = this->declare_parameter<double>(
        "track_config.default.init_ori_std_dev");

    config.default_class_config.mahalanobis_threshold =
        this->declare_parameter<double>(
            "track_config.default.mahalanobis_gate_threshold");
    config.default_class_config.prob_of_detection =
        this->declare_parameter<double>(
            "track_config.default.prob_of_detection");
    config.default_class_config.clutter_intensity =
        this->declare_parameter<double>(
            "track_config.default.clutter_intensity");
    config.default_class_config.prob_of_survival =
        this->declare_parameter<double>(
            "track_config.default.prob_of_survival");
    config.default_class_config.estimate_clutter =
        this->declare_parameter<bool>("track_config.default.estimate_clutter");

    track_manager_ =
        std::make_unique<vortex::filtering::PoseTrackManager>(config);
}

void LandmarkServerNode::timer_callback() {
    track_manager_->step(measurements_, filter_dt_seconds_);
    measurements_.clear();
    if (active_landmark_polling_goal_ &&
        active_landmark_polling_goal_->is_active()) {
        const auto goal = active_landmark_polling_goal_->get_goal();
        const auto type = goal->identifier.type;
        const auto subtype = goal->identifier.subtype;
        if (!track_manager_->has_track(type, subtype)) {
            return;
        }
        vortex_msgs::msg::LandmarkArray landmarks =
            tracks_to_landmark_msgs(type, subtype);
        auto polling_result =
            std::make_shared<vortex_msgs::action::LandmarkPolling_Result>();
        polling_result->landmarks = landmarks;
        active_landmark_polling_goal_->succeed(polling_result);
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(LandmarkServerNode)

}  // namespace vortex::mission
