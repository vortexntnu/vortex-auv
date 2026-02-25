#include "landmark_server/landmark_server_ros.hpp"
#include <spdlog/spdlog.h>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
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
    create_odom_subscription();
    create_polling_action_server();
    create_convergence_action_server();
    create_reference_action_client();

    debug_ = this->declare_parameter<bool>("debug.enable");
    if (debug_) {
        setup_debug_publishers();
    }

    body_z_offset_ =
        this->declare_parameter<double>("body_z_offset");
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
                spdlog::warn("TF transform failed from '{}' to '{}': {}",
                             msg->header.frame_id, target_frame_, ex.what());
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

void LandmarkServerNode::create_odom_subscription() {
    std::string odom_topic =
        this->declare_parameter<std::string>("topics.odom");
    auto qos = vortex::utils::qos_profiles::sensor_data_profile(5);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, qos,
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lock(odom_mtx_);
            last_odom_position_ = msg->pose.pose.position;
        });
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

rclcpp_action::GoalResponse LandmarkServerNode::handle_landmark_convergence_goal( const rclcpp_action::GoalUUID&,
  std::shared_ptr<const vortex_msgs::action::LandmarkConvergence::Goal> goal_msg){
    if (goal_msg->convergence_threshold <= 0.0) {
      spdlog::warn("LandmarkConvergence: reject, invalid convergence_threshold={:.3f}",
                   goal_msg->convergence_threshold);
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal_msg->dead_reckoning_offset < 0.0) {
      spdlog::warn("LandmarkConvergence: reject, invalid dead_reckoning_offset={:.3f}",
                   goal_msg->dead_reckoning_offset);
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal_msg->track_loss_timeout_sec < 0.0) {
      spdlog::warn("LandmarkConvergence: reject, invalid track_loss_timeout_sec={:.3f}",
                   goal_msg->track_loss_timeout_sec);
      return rclcpp_action::GoalResponse::REJECT;
    }

    const vortex::filtering::LandmarkClassKey key{
        goal_msg->type.value, goal_msg->subtype.value};
    
    if (track_manager_->get_tracks_by_type(key).empty()) {
      spdlog::warn("LandmarkConvergence: no confirmed track for type={} subtype={} at goal time – "
                   "accepting anyway, track-loss timeout ({:.1f}s) will apply.",
                   key.type, key.subtype, goal_msg->track_loss_timeout_sec);
    }

    if (!reference_filter_client_->wait_for_action_server(std::chrono::seconds(5))) {
      spdlog::error("LandmarkConvergence: reject, ReferenceFilter server not available after 5s");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (active_landmark_convergence_goal_ &&
        active_landmark_convergence_goal_->is_active())
    {
      active_landmark_convergence_goal_->abort(
          std::make_shared<vortex_msgs::action::LandmarkConvergence::Result>(
              build_convergence_result(false)));
    }

    cancel_reference_filter_goal();

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void LandmarkServerNode::handle_landmark_convergence_accepted(
    const std::shared_ptr<LandmarkConvergenceGoalHandle> goal_handle)
{
    active_landmark_convergence_goal_ = goal_handle;

    const auto goal = goal_handle->get_goal();

    ++convergence_session_id_;

    convergence_active_ = true;
    convergence_class_key_ = {goal->type.value, goal->subtype.value};
    convergence_offset_ = goal->convergence_offset;
    convergence_threshold_ = goal->convergence_threshold;
    convergence_dead_reckoning_offset_ = goal->dead_reckoning_offset;
    convergence_dead_reckoning_handoff_ = false;
    convergence_last_target_pose_.reset();
    convergence_last_known_track_.reset();
    convergence_track_loss_timeout_sec_ = goal->track_loss_timeout_sec;
    convergence_track_lost_ = false;

    spdlog::info("Starting convergence session {}: type={}, subtype={}, "
                 "threshold={:.3f}, dr_offset={:.3f}, track_loss_timeout_sec={:.3f}",
                 convergence_session_id_,
                 convergence_class_key_.type, convergence_class_key_.subtype,
                 goal->convergence_threshold, goal->dead_reckoning_offset,
                 goal->track_loss_timeout_sec);

    const vortex::filtering::Track* track = get_convergence_track();
    if (!track) {
        convergence_track_lost_ = true;
        convergence_track_lost_since_ = now();
        spdlog::warn("Convergence session {}: no track at start (type={}, subtype={}), "
                     "waiting up to {:.1f}s for track to appear.",
                     convergence_session_id_,
                     convergence_class_key_.type, convergence_class_key_.subtype,
                     convergence_track_loss_timeout_sec_);
        return;
    }

    try
    {
        auto target = compute_target_pose(*track, convergence_offset_, now());
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
        spdlog::error("Failed to compute initial convergence target: {}", e.what());
        goal_handle->abort(
            std::make_shared<vortex_msgs::action::LandmarkConvergence::Result>(
                build_convergence_result(false)));
        convergence_active_ = false;
    }
}


rclcpp_action::CancelResponse
LandmarkServerNode::handle_landmark_convergence_cancel(
    const std::shared_ptr<LandmarkConvergenceGoalHandle> /*goal_handle*/)
{
    spdlog::info("Canceling convergence session {}", convergence_session_id_);
    
    ++convergence_session_id_;
    convergence_active_ = false;

    cancel_reference_filter_goal();

    return rclcpp_action::CancelResponse::ACCEPT;
}

void LandmarkServerNode::handle_convergence_update()
{
    if (!convergence_active_)
        return;

    if (!is_convergence_goal_active())
    {
        convergence_active_ = false;
        return;
    }

    const vortex::filtering::Track* track = get_convergence_track();
    if (!track) {
        handle_track_loss();
        return;
    }

    if (convergence_track_lost_) {
        spdlog::info("Convergence: track reacquired (type={}, subtype={})",
                     convergence_class_key_.type, convergence_class_key_.subtype);
        convergence_track_lost_ = false;
    }

    convergence_last_known_track_ = *track;

    update_convergence_target(*track);
    check_dead_reckoning_handoff();
}

bool LandmarkServerNode::track_loss_timeout_exceeded() const
{
    if (!convergence_track_lost_)
        return false;

    const double elapsed = (now() - convergence_track_lost_since_).seconds();
    return elapsed >= convergence_track_loss_timeout_sec_;
}

void LandmarkServerNode::abort_convergence_due_to_track_loss()
{
    spdlog::error("Convergence: track loss timeout ({:.1f} s) exceeded, "
                  "aborting (type={}, subtype={})",
                  convergence_track_loss_timeout_sec_,
                  convergence_class_key_.type, convergence_class_key_.subtype);

    cancel_reference_filter_goal();

    active_landmark_convergence_goal_->abort(
        std::make_shared<vortex_msgs::action::LandmarkConvergence::Result>(
            build_convergence_result(false)));
    convergence_active_ = false;
}

void LandmarkServerNode::handle_track_loss()
{
    if (convergence_dead_reckoning_handoff_)
        return;

    if (!convergence_track_lost_) {
        convergence_track_lost_ = true;
        convergence_track_lost_since_ = now();
        spdlog::warn("Convergence: track lost (type={}, subtype={}), "
                     "starting {:.1f} s timeout",
                     convergence_class_key_.type, convergence_class_key_.subtype,
                     convergence_track_loss_timeout_sec_);
        return;
    }

    if (track_loss_timeout_exceeded()) {
        abort_convergence_due_to_track_loss();
    }
}

void LandmarkServerNode::update_convergence_target(
    const vortex::filtering::Track& track)
{
    if (convergence_dead_reckoning_handoff_)
        return;

    if (!active_reference_filter_goal_) {
        try {
            auto target = compute_target_pose(track, convergence_offset_, now());
            convergence_last_target_pose_ = target;

            vortex_msgs::action::ReferenceFilterWaypoint::Goal rf_goal;
            vortex_msgs::msg::Waypoint wp;
            wp.pose = target.pose;
            wp.mode = vortex_msgs::msg::Waypoint::FULL_POSE;
            rf_goal.waypoint = wp;
            rf_goal.convergence_threshold = convergence_threshold_;

            spdlog::info("Convergence: sending RF goal (session {})",
                         convergence_session_id_);
            send_reference_filter_goal(rf_goal, convergence_session_id_);
        } catch (const std::exception& e) {
            spdlog::warn("Convergence: failed to send RF goal: {}", e.what());
        }
        return;
    }

    // RF goal is active,keep the published reference pose up to date.
    try {
        auto target = compute_target_pose(track, convergence_offset_, now());
        convergence_last_target_pose_ = target;
        publish_reference_pose(target);
    } catch (const std::exception& e) {
        spdlog::warn("Convergence target recompute failed: {}", e.what());
    }
}

void LandmarkServerNode::check_dead_reckoning_handoff()
{
    if (convergence_dead_reckoning_handoff_ || !convergence_last_target_pose_)
        return;

    std::optional<geometry_msgs::msg::Point> odom_pos;
    {
        std::lock_guard<std::mutex> lock(odom_mtx_);
        odom_pos = last_odom_position_;
    }

    if (!odom_pos) {
        spdlog::warn("Dead reckoning check: no odometry received yet on '{}'",
                     this->get_parameter("topics.odom").as_string());
        return;
    }

    const auto& cur = *odom_pos;
    const auto& tgt = convergence_last_target_pose_->pose.position;
    const double dx = cur.x - tgt.x;
    const double dy = cur.y - tgt.y;
    const double dz = cur.z - tgt.z;
    const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    if (!std::isfinite(dist)) {
        spdlog::warn("Invalid distance in dead reckoning check");
        return;
    }

    if (dist <= convergence_dead_reckoning_offset_) {
        convergence_dead_reckoning_handoff_ = true;
        spdlog::info("Dead reckoning handoff triggered at distance={:.3f}",
                     dist);
    }
}


void LandmarkServerNode::send_reference_filter_goal(
    const vortex_msgs::action::ReferenceFilterWaypoint::Goal& goal_msg,
    uint64_t session_id)
{
    cancel_reference_filter_goal();

    rclcpp_action::Client<RF>::SendGoalOptions options;

    options.goal_response_callback =
        [this, session_id](ReferenceFilterGoalHandle::SharedPtr gh)
    {
        if (!gh)
        {
            spdlog::error("ReferenceFilter goal rejected, aborting convergence");
            
            if (session_id == convergence_session_id_ && is_convergence_goal_active()) {
                active_landmark_convergence_goal_->abort(
                    std::make_shared<vortex_msgs::action::LandmarkConvergence::Result>(
                        build_convergence_result(false)));
                convergence_active_ = false;
            }
            return;
        }

        // Ignore old sessions
        if (session_id != convergence_session_id_)
        {
            reference_filter_client_->async_cancel_goal(gh);
            return;
        }

        active_reference_filter_goal_ = gh;
        spdlog::debug("ReferenceFilter goal accepted");
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

        handle_rf_result(res.code);
    };

    reference_filter_client_->async_send_goal(goal_msg, options);
}

void LandmarkServerNode::handle_rf_result(rclcpp_action::ResultCode code)
{
    using RC = rclcpp_action::ResultCode;
    auto make_result = [this](bool success) {
        return std::make_shared<vortex_msgs::action::LandmarkConvergence::Result>(
            build_convergence_result(success));
    };

    switch (code)
    {
        case RC::SUCCEEDED:
            spdlog::info("Convergence succeeded (RF succeeded)");
            active_landmark_convergence_goal_->succeed(make_result(true));
            break;

        case RC::CANCELED:
            spdlog::info("Convergence canceled (RF canceled)");
            active_landmark_convergence_goal_->canceled(make_result(false));
            break;

        case RC::ABORTED:
            spdlog::warn("Convergence aborted (RF aborted)");
            active_landmark_convergence_goal_->abort(make_result(false));
            break;

        default:
            spdlog::error("Convergence aborted (unknown RF result)");
            active_landmark_convergence_goal_->abort(make_result(false));
            break;
    }

    convergence_active_ = false;
}

const vortex::filtering::Track* LandmarkServerNode::get_convergence_track() const
{
    const auto tracks = track_manager_->get_tracks_by_type(convergence_class_key_);
    return tracks.empty() ? nullptr : tracks.front();
}

bool LandmarkServerNode::is_convergence_goal_active() const
{
    return convergence_active_ &&
           active_landmark_convergence_goal_ &&
           active_landmark_convergence_goal_->is_active();
}

void LandmarkServerNode::cancel_reference_filter_goal()
{
    if (!active_reference_filter_goal_)
        return;

    auto goal_to_cancel = active_reference_filter_goal_;
    active_reference_filter_goal_.reset();
    reference_filter_client_->async_cancel_goal(goal_to_cancel);
}

vortex_msgs::action::LandmarkConvergence::Result
LandmarkServerNode::build_convergence_result(bool success) const
{
    vortex_msgs::action::LandmarkConvergence::Result res;
    res.success = success;

    if (!convergence_last_known_track_) {
        return res;
    }

    const auto& track = *convergence_last_known_track_;
    res.landmark.id = track.id;
    res.landmark.type.value  = track.class_key.type;
    res.landmark.subtype.value = track.class_key.subtype;
    res.landmark.header.stamp = now();
    res.landmark.header.frame_id = target_frame_;
    res.landmark.pose =
        vortex::filtering::ros_conversions::track_to_pose_with_covariance(track);

    return res;
}

geometry_msgs::msg::PoseStamped LandmarkServerNode::compute_target_pose(
    const vortex::filtering::Track& track,
    const geometry_msgs::msg::Pose& convergence_offset,
    const rclcpp::Time& stamp) {
    const auto landmark_pose = track.to_pose();

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

    p_target.z() += body_z_offset_;

    Eigen::Quaterniond q_target =
        (q_landmark * q_offset).normalized();

    out.pose.position.x = p_target.x();
    out.pose.position.y = p_target.y();
    out.pose.position.z = p_target.z();

    out.pose.orientation.x = q_target.x();
    out.pose.orientation.y = q_target.y();
    out.pose.orientation.z = q_target.z();
    out.pose.orientation.w = q_target.w();

    return out;
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

void LandmarkServerNode::publish_reference_pose(const geometry_msgs::msg::PoseStamped& pose)
{
  reference_pose_pub_->publish(pose);
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
        publish_convergence_landmark_debug();
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
