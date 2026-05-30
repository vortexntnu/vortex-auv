#include "reference_filter_dp_quat/ros/reference_filter_ros.hpp"
#include <spdlog/spdlog.h>
#include <mutex>
#include <rclcpp_components/register_node_macro.hpp>
#include <thread>
#include <vortex/utils/ros/qos_profiles.hpp>
#include <vortex/utils/ros/waypoint_ros_conversions.hpp>
#include "reference_filter_dp_quat/ros/reference_filter_ros_utils.hpp"

const auto start_message = R"(
██████╗ ███████╗███████╗███████╗██████╗ ███████╗███╗   ██╗ ██████╗███████╗    ███████╗██╗██╗  ████████╗███████╗██████╗      ██████╗ ██╗   ██╗ █████╗ ████████╗
██╔══██╗██╔════╝██╔════╝██╔════╝██╔══██╗██╔════╝████╗  ██║██╔════╝██╔════╝    ██╔════╝██║██║  ╚══██╔══╝██╔════╝██╔══██╗    ██╔═══██╗██║   ██║██╔══██╗╚══██╔══╝
██████╔╝█████╗  █████╗  █████╗  ██████╔╝█████╗  ██╔██╗ ██║██║     █████╗      █████╗  ██║██║     ██║   █████╗  ██████╔╝    ██║   ██║██║   ██║███████║   ██║
██╔══██╗██╔══╝  ██╔══╝  ██╔══╝  ██╔══██╗██╔══╝  ██║╚██╗██║██║     ██╔══╝      ██╔══╝  ██║██║     ██║   ██╔══╝  ██╔══██╗    ██║▄▄ ██║██║   ██║██╔══██║   ██║
██║  ██║███████╗██║     ███████╗██║  ██║███████╗██║ ╚████║╚██████╗███████╗    ██║     ██║███████╗██║   ███████╗██║  ██║    ╚██████╔╝╚██████╔╝██║  ██║   ██║
╚═╝  ╚═╝╚══════╝╚═╝     ╚══════╝╚═╝  ╚═╝╚══════╝╚═╝  ╚═══╝ ╚═════╝╚══════╝    ╚═╝     ╚═╝╚══════╝╚═╝   ╚══════╝╚═╝  ╚═╝     ╚══▀▀═╝  ╚═════╝ ╚═╝  ╚═╝   ╚═╝
 )";

namespace vortex::guidance {

ReferenceFilterNode::ReferenceFilterNode(const rclcpp::NodeOptions& options)
    : Node("reference_filter_node", options) {
    int time_step_ms = this->declare_parameter<int>("time_step_ms");
    time_step_ = std::chrono::milliseconds(time_step_ms);

    set_subscribers_and_publisher();

    set_action_server();

    set_refererence_filter();

    setup_reset_subscription();

    setup_debug_publisher();

    spdlog::info(start_message);
}

ReferenceFilterNode::~ReferenceFilterNode() {
    preempted_ = true;
    if (execute_thread_.joinable()) {
        execute_thread_.join();
    }
}

void ReferenceFilterNode::set_subscribers_and_publisher() {
    this->declare_parameter<std::string>("topics.pose");
    this->declare_parameter<std::string>("topics.twist");
    this->declare_parameter<std::string>("topics.guidance.dp_quat");
    this->declare_parameter<std::string>("topics.reference_pose");
    altitude_control_enabled_ =
        this->declare_parameter<bool>("altitude_control_enabled", false);

    std::string pose_topic = this->get_parameter("topics.pose").as_string();
    std::string twist_topic = this->get_parameter("topics.twist").as_string();
    std::string guidance_topic =
        this->get_parameter("topics.guidance.dp_quat").as_string();
    std::string reference_pose_topic =
        this->get_parameter("topics.reference_pose").as_string();

    auto qos_sensor_data = vortex::utils::qos_profiles::sensor_data_profile(1);
    reference_pub_ =
        this->create_publisher<vortex_msgs::msg::ReferenceFilterQuat>(
            guidance_topic, qos_sensor_data);

    publish_rpy_debug_ = this->declare_parameter<bool>("publish_rpy_debug");
    if (publish_rpy_debug_) {
        std::string rpy_topic = this->declare_parameter<std::string>(
            "topics.guidance.dp_rpy", guidance_topic + "_rpy");
        rpy_debug_pub_ =
            this->create_publisher<vortex_msgs::msg::ReferenceFilter>(
                rpy_topic, qos_sensor_data);
        spdlog::info("RPY debug publisher enabled on topic: {}", rpy_topic);
    }

    reference_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        reference_pose_topic, qos_sensor_data,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            // Introduce altitude handling here??
            follower_->set_reference(
                vortex::utils::ros_conversions::ros_pose_to_pose(msg->pose));
        });

    pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, qos_sensor_data,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
                   msg) {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            current_pose_ = vortex::utils::ros_conversions::ros_pose_to_pose(
                msg->pose.pose);
        });

    twist_sub_ = this->create_subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>(
        twist_topic, qos_sensor_data,
        [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr
                   msg) {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            current_twist_ = vortex::utils::ros_conversions::ros_twist_to_twist(
                msg->twist.twist);
        });

    if (altitude_control_enabled_) {
        this->declare_parameter<std::string>("topics.dvl_altitude");
        this->declare_parameter<double>("altitude_lp_alpha", 0.9);

        std::string dvl_altitude_topic =
            this->get_parameter("topics.dvl_altitude").as_string();
        altitude_lp_alpha_ =
            this->get_parameter("altitude_lp_alpha").as_double();

        altitude_sub_ =
            this->create_subscription<vortex_msgs::msg::DVLAltitude>(
                dvl_altitude_topic, qos_sensor_data,
                [this](const vortex_msgs::msg::DVLAltitude::SharedPtr msg) {
                    std::lock_guard<std::mutex> lock(sensor_mutex_);
                    if (msg->altitude <= 0.0) {
                        return;  // Ignore invalid altitude readings
                    }
                    if (!altitude_valid_) {
                        current_altitude_ = msg->altitude;
                        altitude_valid_ = true;
                    } else {
                        current_altitude_ =
                            altitude_lp_alpha_ * current_altitude_ +
                            (1.0 - altitude_lp_alpha_) * msg->altitude;
                    }
                });

        spdlog::info("Altitude control enabled, subscribing to '{}'",
                     dvl_altitude_topic);
    }
}

void ReferenceFilterNode::setup_reset_subscription() {
    reset_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "mission/wipe", vortex::utils::qos_profiles::reliable_profile(1),
        [this](std_msgs::msg::Empty::ConstSharedPtr msg) {
            on_system_reset(msg);
        });
}

void ReferenceFilterNode::setup_debug_publisher() {
    const std::string mode_str =
        this->declare_parameter<std::string>("debug.goal_publish_mode", "none");

    if (mode_str == "timer") {
        debug_mode_ = DebugPublishMode::timer;
    } else if (mode_str == "on_new_goal") {
        debug_mode_ = DebugPublishMode::on_new_goal;
    } else {
        debug_mode_ = DebugPublishMode::none;
        return;
    }

    const std::string topic = this->declare_parameter<std::string>(
        "topics.guidance.dp_quat_target_pose", "guidance/dp_quat_target_pose");

    debug_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        topic, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

    spdlog::info("Reference goal debug publisher active (mode: {}, topic: {})",
                 mode_str, topic);
}

void ReferenceFilterNode::publish_debug_goal() {
    if (!debug_goal_pub_ || !executing_.load()) {
        return;
    }
    const Pose goal = follower_->waypoint_goal();
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "odom";
    msg.pose.position.x = goal.x;
    msg.pose.position.y = goal.y;
    msg.pose.position.z = goal.z;
    msg.pose.orientation.w = goal.qw;
    msg.pose.orientation.x = goal.qx;
    msg.pose.orientation.y = goal.qy;
    msg.pose.orientation.z = goal.qz;
    debug_goal_pub_->publish(msg);
}

void ReferenceFilterNode::on_system_reset(
    std_msgs::msg::Empty::ConstSharedPtr) {
    std::lock_guard<std::mutex> lock(execute_mutex_);
    preempted_ = true;
    if (execute_thread_.joinable()) {
        execute_thread_.join();
    }
    preempted_ = false;
    spdlog::info("ReferenceFilter: reset complete");
}

void ReferenceFilterNode::set_action_server() {
    this->declare_parameter<std::string>("action_servers.reference_filter");
    std::string action_server_name =
        this->get_parameter("action_servers.reference_filter").as_string();

    action_server_ =
        rclcpp_action::create_server<vortex_msgs::action::GuidanceWaypoint>(
            this, action_server_name,
            [this](const auto& uuid, auto goal) {
                return handle_goal(uuid, std::move(goal));
            },
            [this](auto goal_handle) { return handle_cancel(goal_handle); },
            [this](auto goal_handle) { handle_accepted(goal_handle); });
}

void ReferenceFilterNode::set_refererence_filter() {
    this->declare_parameter<std::vector<double>>("zeta");
    this->declare_parameter<std::vector<double>>("omega");

    std::vector<double> zeta = this->get_parameter("zeta").as_double_array();
    std::vector<double> omega = this->get_parameter("omega").as_double_array();

    Eigen::Vector6d zeta_eigen = Eigen::Map<Eigen::Vector6d>(zeta.data());
    Eigen::Vector6d omega_eigen = Eigen::Map<Eigen::Vector6d>(omega.data());

    filter_params_ = ReferenceFilterParams{omega_eigen, zeta_eigen};

    double dt_seconds = time_step_.count() / 1000.0;
    follower_ = std::make_unique<WaypointFollower>(filter_params_, dt_seconds);
}

rclcpp_action::GoalResponse ReferenceFilterNode::handle_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const vortex_msgs::action::GuidanceWaypoint::Goal>
    /*goal*/) {
    spdlog::info("Accepted goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ReferenceFilterNode::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::GuidanceWaypoint>> /*goal_handle*/) {
    spdlog::info("Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ReferenceFilterNode::handle_accepted(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::GuidanceWaypoint>>
        goal_handle) {
    std::lock_guard<std::mutex> lock(execute_mutex_);
    const bool retarget = executing_.load();
    preempted_ = true;
    if (execute_thread_.joinable()) {
        execute_thread_.join();
    }
    preempted_ = false;

    execute_thread_ = std::thread(
        [this, goal_handle, retarget]() { execute(goal_handle, retarget); });
}

void ReferenceFilterNode::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::GuidanceWaypoint>> goal_handle,
    bool retarget) {
    executing_ = true;

    double threshold = goal_handle->get_goal()->convergence_threshold;
    if (threshold <= 0.0) {
        threshold = 0.1;
        spdlog::warn(
            "ReferenceFilter: invalid convergence_threshold (<= 0), using 0.1");
    }

    auto wp = vortex::utils::waypoints::waypoint_from_ros(
        goal_handle->get_goal()->waypoint);

    if (wp.keep_altitude && altitude_control_enabled_ &&
        wp.desired_altitude <= 0.0) {
        executing_ = false;
        auto result =
            std::make_shared<vortex_msgs::action::GuidanceWaypoint::Result>();
        result->success = false;
        goal_handle->abort(result);
        spdlog::error(
            "ReferenceFilter: desired_altitude must be > 0, got {:.3f}",
            wp.desired_altitude);
        return;
    }

    if (wp.keep_altitude && altitude_control_enabled_) {
        const auto [pose, current_alt, alt_valid] = [this] {
            std::lock_guard lock(sensor_mutex_);
            return std::tuple{current_pose_, current_altitude_,
                              altitude_valid_};
        }();
        if (!alt_valid) {
            spdlog::warn(
                "ReferenceFilter: keep_altitude requested but no DVL altitude "
                "received yet; proceeding with waypoint z as-is");
        } else {
            wp.pose.z = pose.z + current_alt - wp.desired_altitude;
        }
        spdlog::info(
            "Altitude-hold mode: desired_altitude={:.2f} m, "
            "initial_altitude={:.2f}, initial z_goal={:.3f}",
            wp.desired_altitude, current_altitude_, wp.pose.z);
    } else if (wp.keep_altitude && !altitude_control_enabled_) {
        spdlog::warn(
            "ReferenceFilter: keep_altitude requested but altitude control is "
            "not enabled; proceeding with waypoint z as-is");
    }

    if (retarget) {
        follower_->retarget(wp, threshold);
        spdlog::info("Executing goal (filter state preserved)");
    } else {
        const auto [pose, twist] = [this] {
            std::lock_guard lock(sensor_mutex_);
            return std::pair{current_pose_, current_twist_};
        }();
        follower_->start(pose, twist, wp, threshold);
        spdlog::info("Executing goal (cold start)");
    }

    if (debug_mode_ == DebugPublishMode::on_new_goal) {
        publish_debug_goal();
    }

    const bool keep_altitude = wp.keep_altitude && altitude_control_enabled_;
    const double desired_altitude = wp.desired_altitude;

    auto result =
        std::make_shared<vortex_msgs::action::GuidanceWaypoint::Result>();

    rclcpp::Rate loop_rate(1000.0 / time_step_.count());

    while (rclcpp::ok()) {
        if (preempted_.load()) {
            executing_ = false;
            result->success = false;
            goal_handle->abort(result);
            spdlog::info("Goal preempted by newer goal");
            return;
        }

        if (goal_handle->is_canceling()) {
            executing_ = false;
            result->success = false;
            goal_handle->canceled(result);
            spdlog::info("Goal canceled");
            return;
        }

        follower_->step();

        if (keep_altitude) {
            const auto [current_z, current_alt] = [this] {
                std::lock_guard lock(sensor_mutex_);
                return std::pair{current_pose_.z, current_altitude_};
            }();
            follower_->update_z_goal(current_z + current_alt -
                                     desired_altitude);
        }

        reference_pub_->publish(
            fill_reference_msg(follower_->pose(), follower_->velocity()));
        if (publish_rpy_debug_) {
            rpy_debug_pub_->publish(fill_reference_rpy_msg(
                follower_->pose(), follower_->velocity()));
        }
        if (debug_mode_ == DebugPublishMode::timer) {
            publish_debug_goal();
        }

        const auto current_pose = [this] {
            std::lock_guard lock(sensor_mutex_);
            return current_pose_;
        }();

        const bool converged =
            (keep_altitude && !wp.require_altitude_convergence)
                ? follower_->within_convergance_ignore_z(current_pose)
                : follower_->within_convergance(current_pose);

        if (converged) {
            follower_->snap_state_to_reference();

            reference_pub_->publish(
                fill_reference_msg(follower_->pose(), follower_->velocity()));
            if (publish_rpy_debug_) {
                rpy_debug_pub_->publish(fill_reference_rpy_msg(
                    follower_->pose(), follower_->velocity()));
            }

            executing_ = false;
            result->success = true;
            goal_handle->succeed(result);
            spdlog::info("Goal reached");
            return;
        }

        loop_rate.sleep();
    }

    if (!rclcpp::ok() && goal_handle->is_active()) {
        executing_ = false;
        result->success = false;
        try {
            goal_handle->abort(result);
        } catch (...) {
        }
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ReferenceFilterNode)

}  // namespace vortex::guidance
