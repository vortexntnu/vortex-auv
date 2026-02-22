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
#include <vortex_msgs/msg/landmark_track_array.hpp>

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

    debug_ = this->declare_parameter<bool>("debug.enable");
    if (debug_) {
        setup_debug_publishers();
    }
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
  const rclcpp_action::GoalUUID&,
  std::shared_ptr<const vortex_msgs::action::LandmarkConvergence::Goal> goal_msg)
{
  const int id = goal_msg->id;

  // Validate parameters
  if (goal_msg->convergence_threshold <= 0.0) {
    RCLCPP_WARN(get_logger(), 
                "LandmarkConvergence: reject, invalid convergence_threshold=%.3f", 
                goal_msg->convergence_threshold);
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal_msg->dead_reckoning_offset < 0.0) {
    RCLCPP_WARN(get_logger(), 
                "LandmarkConvergence: reject, invalid dead_reckoning_offset=%.3f", 
                goal_msg->dead_reckoning_offset);
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!track_manager_->has_track(id)) {
    RCLCPP_WARN(get_logger(), 
                "LandmarkConvergence: reject, no CONFIRMED track for id=%d", id);
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Check if ReferenceFilter server is available
  if (!reference_filter_client_->action_server_is_ready()) {
    RCLCPP_ERROR(get_logger(), 
                 "LandmarkConvergence: reject, ReferenceFilter server not available");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (active_landmark_convergence_goal_ &&
      active_landmark_convergence_goal_->is_active())
  {
    auto res = std::make_shared<vortex_msgs::action::LandmarkConvergence::Result>();
    active_landmark_convergence_goal_->abort(res);
  }

  if (active_reference_filter_goal_) {
    reference_filter_client_->async_cancel_goal(active_reference_filter_goal_);
    active_reference_filter_goal_.reset();
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void LandmarkServerNode::handle_landmark_convergence_accepted(
    const std::shared_ptr<LandmarkConvergenceGoalHandle> goal_handle)
{
    active_landmark_convergence_goal_ = goal_handle;

    const auto goal = goal_handle->get_goal();

    ++convergence_session_id_; //New session

    convergence_active_ = true;
    convergence_landmark_id_ = goal->id;
    convergence_offset_ = goal->convergence_offset;
    convergence_threshold_ = goal->convergence_threshold;
    convergence_dead_reckoning_offset_ = goal->dead_reckoning_offset;
    convergence_dead_reckoning_handoff_ = false;
    convergence_last_target_pose_.reset();

    RCLCPP_INFO(get_logger(),
                "Starting convergence session %lu: landmark_id=%d, threshold=%.3f, dr_offset=%.3f",
                convergence_session_id_, goal->id, goal->convergence_threshold, goal->dead_reckoning_offset);

    // Compute initial target
    try
    {
        auto target =
            compute_target_pose(goal->id,
                                goal->convergence_offset,
                                now());

        convergence_last_target_pose_ = target;

        vortex_msgs::action::ReferenceFilterWaypoint::Goal rf_goal;

        vortex_msgs::msg::Waypoint wp;
        wp.pose = target.pose;
        wp.mode = vortex_msgs::msg::Waypoint::FULL_POSE;

        rf_goal.waypoint = wp;
        rf_goal.convergence_threshold = goal->convergence_threshold;

        send_reference_filter_goal(rf_goal, convergence_session_id_);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(get_logger(),
                     "Failed to compute initial convergence target: %s",
                     e.what());

        auto res =
          std::make_shared<
            vortex_msgs::action::LandmarkConvergence::Result>();

        goal_handle->abort(res);
        convergence_active_ = false;
    }
}


rclcpp_action::CancelResponse
LandmarkServerNode::handle_landmark_convergence_cancel(
    const std::shared_ptr<LandmarkConvergenceGoalHandle> /*goal_handle*/)
{
    RCLCPP_INFO(get_logger(), "Canceling convergence session %lu", convergence_session_id_);
    
    ++convergence_session_id_;
    convergence_active_ = false;

    if (active_reference_filter_goal_)
    {
        auto goal_to_cancel = active_reference_filter_goal_;
        active_reference_filter_goal_.reset();
        reference_filter_client_->async_cancel_goal(goal_to_cancel);
    }

    return rclcpp_action::CancelResponse::ACCEPT;
}

void LandmarkServerNode::handle_convergence_update()
{
    if (!convergence_active_)
        return;

    if (!active_landmark_convergence_goal_ ||
        !active_landmark_convergence_goal_->is_active())
    {
        RCLCPP_WARN(get_logger(), "Cleaning up orphaned convergence state");
        convergence_active_ = false;
        return;
    }

    const int id = convergence_landmark_id_;

    const bool track_exists = track_manager_->has_track(id);

    if (!track_exists && !convergence_dead_reckoning_handoff_)
    {
        RCLCPP_ERROR(get_logger(), 
                    "Track lost before dead reckoning handoff, aborting convergence");
        
        auto res =
          std::make_shared<
            vortex_msgs::action::LandmarkConvergence::Result>();

        active_landmark_convergence_goal_->abort(res);
        convergence_active_ = false;
        return;
    }

    
    if (track_exists)
    {
        try
        {
            auto target =
                compute_target_pose(
                    id,
                    convergence_offset_,
                    now());

            convergence_last_target_pose_ = target;

            if (!convergence_dead_reckoning_handoff_) {
                publish_reference_pose(target);
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(get_logger(),
                        "Convergence target recompute failed: %s",
                        e.what());
        }
    }

    // Dead reckoning check
    if (!convergence_dead_reckoning_handoff_ &&
        convergence_last_target_pose_)
    {
        std::optional<vortex_msgs::action::ReferenceFilterWaypoint::Feedback> fb_copy;
        {
            std::lock_guard<std::mutex> lock(rf_fb_mtx_);
            if (last_rf_feedback_) {
                fb_copy = last_rf_feedback_;
            }
        }

        if (fb_copy) {
            const auto& cur = fb_copy->reference;
            const auto& tgt = convergence_last_target_pose_->pose.position;

            double dx = cur.x - tgt.x;
            double dy = cur.y - tgt.y;
            double dz = cur.z - tgt.z;

            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

            if (!std::isfinite(dist)) {
                RCLCPP_WARN(get_logger(), 
                           "Invalid distance calculation in dead reckoning check");
                return;
            }

            if (dist <= convergence_dead_reckoning_offset_) {
                convergence_dead_reckoning_handoff_ = true;
                RCLCPP_INFO(get_logger(), 
                           "Dead reckoning handoff triggered, distance=%.3f", dist);
            }
        }
    }
}


void LandmarkServerNode::send_reference_filter_goal(
    const vortex_msgs::action::ReferenceFilterWaypoint::Goal& goal_msg,
    uint64_t session_id)
{
    if (active_reference_filter_goal_)
    {
        auto goal_to_cancel = active_reference_filter_goal_;
        active_reference_filter_goal_.reset();
        reference_filter_client_->async_cancel_goal(goal_to_cancel);
    }

    using RF = vortex_msgs::action::ReferenceFilterWaypoint;
    rclcpp_action::Client<RF>::SendGoalOptions options;

    options.goal_response_callback =
        [this, session_id](ReferenceFilterGoalHandle::SharedPtr gh)
    {
        if (!gh)
        {
            RCLCPP_ERROR(get_logger(),
                        "ReferenceFilter goal rejected - aborting convergence");
            
            // Abort convergence if RF goal was rejected
            if (session_id == convergence_session_id_ && 
                active_landmark_convergence_goal_ &&
                active_landmark_convergence_goal_->is_active()) {
                
                auto res = std::make_shared<vortex_msgs::action::LandmarkConvergence::Result>();
                active_landmark_convergence_goal_->abort(res);
                convergence_active_ = false;
            }
            return;
        }

        // Ignore old sessions
        if (session_id != convergence_session_id_)
        {
            RCLCPP_DEBUG(get_logger(), "Canceling stale ReferenceFilter goal");
            reference_filter_client_->async_cancel_goal(gh);
            return;
        }

        active_reference_filter_goal_ = gh;
        RCLCPP_DEBUG(get_logger(), "ReferenceFilter goal accepted");
    };


    options.feedback_callback =
        [this, session_id](
            ReferenceFilterGoalHandle::SharedPtr,
            const std::shared_ptr<const RF::Feedback> fb)
    {
        if (session_id != convergence_session_id_)
            return;

        {
            std::lock_guard<std::mutex> lock(rf_fb_mtx_);
            last_rf_feedback_ = *fb;
        }
    };

    options.result_callback =
        [this, session_id](
            const ReferenceFilterGoalHandle::WrappedResult& res)
    {
        if (session_id != convergence_session_id_)
            return;

        active_reference_filter_goal_.reset();

        if (!convergence_active_ ||
            !active_landmark_convergence_goal_)
            return;

        auto conv_res =
          std::make_shared<
            vortex_msgs::action::LandmarkConvergence::Result>();

        switch (res.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(get_logger(),
                            "Convergence succeeded (RF succeeded)");
                active_landmark_convergence_goal_->succeed(conv_res);
                break;

            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(get_logger(),
                            "Convergence canceled (RF canceled)");
                active_landmark_convergence_goal_->canceled(conv_res);
                break;

            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_WARN(get_logger(),
                            "Convergence aborted (RF aborted)");
                active_landmark_convergence_goal_->abort(conv_res);
                break;

            default:
                RCLCPP_ERROR(get_logger(),
                             "Convergence aborted (unknown RF result)");
                active_landmark_convergence_goal_->abort(conv_res);
                break;
        }

        convergence_active_ = false;
    };

    reference_filter_client_->async_send_goal(goal_msg, options);
}

geometry_msgs::msg::PoseStamped LandmarkServerNode::compute_target_pose(
    int landmark_id,
    const geometry_msgs::msg::Pose& convergence_offset,
    const rclcpp::Time& stamp)
{
    const vortex::filtering::Track* selected = nullptr;

    for (const auto& t : track_manager_->get_tracks()) {
        if (t.confirmed && t.id == landmark_id) {
            selected = &t;
            break;
        }
    }

    if (!selected) {
        throw std::runtime_error("Track not found for landmark_id=" + std::to_string(landmark_id));
    }

    const auto landmark_pose = selected->to_pose();

    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = stamp;
    out.header.frame_id = target_frame_;


    Eigen::Vector3d p_landmark = landmark_pose.pos_vector();

    if (!p_landmark.allFinite()) {
        throw std::runtime_error("Invalid landmark position (contains NaN/Inf)");
    }

    Eigen::Quaterniond q_landmark(
        landmark_pose.ori_quaternion().w(),
        landmark_pose.ori_quaternion().x(),
        landmark_pose.ori_quaternion().y(),
        landmark_pose.ori_quaternion().z());

    q_landmark.normalize();


    Eigen::Vector3d p_offset(
        convergence_offset.position.x,
        convergence_offset.position.y,
        convergence_offset.position.z);


    if (!p_offset.allFinite()) {
        throw std::runtime_error("Invalid convergence offset (contains NaN/Inf)");
    }

    Eigen::Quaterniond q_offset(
        convergence_offset.orientation.w,
        convergence_offset.orientation.x,
        convergence_offset.orientation.y,
        convergence_offset.orientation.z);

    q_offset.normalize();

    Eigen::Vector3d p_target =
        p_landmark + q_landmark * p_offset;

    Eigen::Quaterniond q_target =
        (q_landmark * q_offset).normalized();

    if (!p_target.allFinite()) {
        throw std::runtime_error("Invalid target position computed (contains NaN/Inf)");
    }


    out.pose.position.x = p_target.x();
    out.pose.position.y = p_target.y();
    out.pose.position.z = p_target.z();

    out.pose.orientation.x = q_target.x();
    out.pose.orientation.y = q_target.y();
    out.pose.orientation.z = q_target.z();
    out.pose.orientation.w = q_target.w();

    return out;
}



void LandmarkServerNode::publish_reference_pose(const geometry_msgs::msg::PoseStamped& pose)
{
  reference_pose_pub_->publish(pose);
}

void LandmarkServerNode::setup_debug_publishers() {
    std::string debug_topic =
        this->declare_parameter<std::string>("debug.topic_name");
    auto qos = vortex::utils::qos_profiles::sensor_data_profile(10);
    landmark_track_debug_pub_ =
        this->create_publisher<vortex_msgs::msg::LandmarkTrackArray>(
            debug_topic, qos);
}

void LandmarkServerNode::publish_debug_tracks() {
    vortex_msgs::msg::LandmarkTrackArray msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = target_frame_;

    for (const auto& t : track_manager_->get_tracks()) {
        vortex_msgs::msg::LandmarkTrack track_msg;

        track_msg.landmark.id = t.id;
        track_msg.landmark.type.value = t.class_key.type;
        track_msg.landmark.subtype.value = t.class_key.subtype;
        track_msg.landmark.header.stamp = this->now();
        track_msg.landmark.header.frame_id = target_frame_;

        const auto pose = t.to_pose();
        track_msg.landmark.pose.pose.position.x = pose.pos_vector().x();
        track_msg.landmark.pose.pose.position.y = pose.pos_vector().y();
        track_msg.landmark.pose.pose.position.z = pose.pos_vector().z();
        track_msg.landmark.pose.pose.orientation.x = pose.ori_quaternion().x();
        track_msg.landmark.pose.pose.orientation.y = pose.ori_quaternion().y();
        track_msg.landmark.pose.pose.orientation.z = pose.ori_quaternion().z();
        track_msg.landmark.pose.pose.orientation.w = pose.ori_quaternion().w();

        track_msg.existence_probability = t.existence_probability;

        msg.landmark_tracks.push_back(track_msg);
    }

    landmark_track_debug_pub_->publish(msg);
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
        goal_handle->get_goal()->type.value,
        goal_handle->get_goal()->subtype.value);
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

    if (debug_) {
        publish_debug_tracks();
    }

    handle_convergence_update();
    if (active_landmark_polling_goal_ &&
        active_landmark_polling_goal_->is_active()) {
        const auto goal = active_landmark_polling_goal_->get_goal();
        const auto type = goal->type.value;
        const auto subtype = goal->subtype.value;
        if (!track_manager_->has_track(
                vortex::filtering::LandmarkClassKey{type, subtype})) {
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
