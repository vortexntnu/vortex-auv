#include "reference_filter_dp/ros/reference_filter_ros.hpp"
#include <spdlog/spdlog.h>
#include <mutex>
#include <rclcpp_components/register_node_macro.hpp>
#include <thread>
#include <vortex/utils/ros/qos_profiles.hpp>
#include <vortex/utils/ros/ros_conversions.hpp>
#include "reference_filter_dp/ros/reference_filter_ros_utils.hpp"

const auto start_message = R"(
  ____       __                                _____ _ _ _
 |  _ \ ___ / _| ___ _ __ ___ _ __   ___ ___  |  ___(_) | |_ ___ _ __
 | |_) / _ \ |_ / _ \ '__/ _ \ '_ \ / __/ _ \ | |_  | | | __/ _ \ '__|
 |  _ <  __/  _|  __/ | |  __/ | | | (_|  __/ |  _| | | | ||  __/ |
 |_| \_\___|_|  \___|_|  \___|_| |_|\___\___| |_|   |_|_|\__\___|_|

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
    this->declare_parameter<std::string>("topics.guidance.dp_rpy");
    this->declare_parameter<std::string>("topics.reference_pose");
    altitude_control_enabled_ =
        this->declare_parameter<bool>("altitude_control_enabled", false);
    default_tolerance_.position =
        this->declare_parameter<double>("default_position_tolerance", 0.1);
    default_tolerance_.orientation =
        this->declare_parameter<double>("default_orientation_tolerance", 0.1);

    std::string pose_topic = this->get_parameter("topics.pose").as_string();
    std::string twist_topic = this->get_parameter("topics.twist").as_string();
    std::string guidance_topic =
        this->get_parameter("topics.guidance.dp_rpy").as_string();
    std::string reference_pose_topic =
        this->get_parameter("topics.reference_pose").as_string();

    auto qos_sensor_data = vortex::utils::qos_profiles::sensor_data_profile(1);
    reference_pub_ = this->create_publisher<vortex_msgs::msg::ReferenceFilter>(
        guidance_topic, qos_sensor_data);

    reference_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        reference_pose_topic, qos_sensor_data,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            follower_->set_reference(
                vortex::utils::ros_conversions::ros_pose_to_pose_euler(
                    msg->pose));
        });

    pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, qos_sensor_data,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
                   msg) {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            current_pose_ =
                vortex::utils::ros_conversions::ros_pose_to_pose_euler(
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
    std::shared_ptr<const vortex_msgs::action::GuidanceWaypoint::Goal> goal) {
    try {
        waypoint_mode_from_ros(goal->waypoint.waypoint_mode);
    } catch (const std::invalid_argument& e) {
        spdlog::warn("ReferenceFilter: rejected goal: {}", e.what());
        return rclcpp_action::GoalResponse::REJECT;
    }
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
    // Keep the filter state if a goal is still executing, else cold start.
    const bool retarget = executing_.load();
    preempted_ = true;
    if (execute_thread_.joinable()) {
        execute_thread_.join();
    }
    preempted_ = false;
    executing_ = true;

    execute_thread_ = std::thread(
        [this, goal_handle, retarget]() { execute(goal_handle, retarget); });
}

void ReferenceFilterNode::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::GuidanceWaypoint>> goal_handle,
    bool retarget) {
    const auto& waypoint_msg = goal_handle->get_goal()->waypoint;
    // A tolerance the waypoint does not set (<= 0) uses the default
    const ConvergenceTolerance tolerance{
        waypoint_msg.position_tolerance > 0.0 ? waypoint_msg.position_tolerance
                                              : default_tolerance_.position,
        waypoint_msg.orientation_tolerance > 0.0
            ? waypoint_msg.orientation_tolerance
            : default_tolerance_.orientation};

    auto result =
        std::make_shared<vortex_msgs::action::GuidanceWaypoint::Result>();

    auto wp = waypoint_from_ros(waypoint_msg);
    const bool keep_altitude = wp.keep_altitude && altitude_control_enabled_;
    if (keep_altitude && wp.desired_altitude <= 0.0) {
        result->success = false;
        executing_ = false;
        goal_handle->abort(result);
        spdlog::error(
            "ReferenceFilter: desired_altitude must be > 0, got {:.3f}",
            wp.desired_altitude);
        return;
    }

    const auto [pose, twist, altitude, altitude_valid] = [this] {
        std::lock_guard lock(sensor_mutex_);
        return std::tuple{current_pose_, current_twist_, current_altitude_,
                          altitude_valid_};
    }();

    if (keep_altitude) {
        if (altitude_valid) {
            wp.pose.z = pose.z + altitude - wp.desired_altitude;
        } else {
            spdlog::warn(
                "ReferenceFilter: keep_altitude requested but no DVL altitude "
                "received yet; using the waypoint z");
        }
        spdlog::info("Altitude hold: desired_altitude={:.2f} m, z_goal={:.3f}",
                     wp.desired_altitude, wp.pose.z);
    }

    if (retarget) {
        follower_->retarget(wp, tolerance);
        spdlog::info("Executing goal (filter state kept)");
    } else {
        follower_->start(pose, twist, wp, tolerance);
        spdlog::info("Executing goal (cold start)");
    }

    rclcpp::Rate loop_rate(1000.0 / time_step_.count());

    while (rclcpp::ok()) {
        if (preempted_.load()) {
            result->success = false;
            executing_ = false;
            goal_handle->abort(result);
            spdlog::info("Goal preempted by newer goal");
            return;
        }

        if (goal_handle->is_canceling()) {
            result->success = false;
            executing_ = false;
            goal_handle->canceled(result);
            spdlog::info("Goal canceled");
            return;
        }

        Eigen::Vector18d filter_state = follower_->step();

        if (keep_altitude) {
            const auto [current_z, current_altitude] = [this] {
                std::lock_guard lock(sensor_mutex_);
                return std::pair{current_pose_.z, current_altitude_};
            }();
            follower_->update_z_goal(current_z + current_altitude -
                                     wp.desired_altitude);
        }

        reference_pub_->publish(fill_reference_msg(filter_state));

        const auto current_pose_vector = [this] {
            std::lock_guard lock(sensor_mutex_);
            return current_pose_.to_vector();
        }();

        // In altitude hold the z goal follows a noisy measurement: leave z out
        const bool target_reached = follower_->update_convergence(
            current_pose_vector, this->get_clock()->now().seconds(),
            waypoint_msg.hold_time_sec, keep_altitude);

        if (target_reached) {
            follower_->snap_state_to_reference();
            reference_pub_->publish(fill_reference_msg(follower_->state()));

            result->success = true;
            executing_ = false;
            goal_handle->succeed(result);
            spdlog::info("Goal reached");
            return;
        }

        loop_rate.sleep();
    }

    executing_ = false;
    if (!rclcpp::ok() && goal_handle->is_active()) {
        result->success = false;
        try {
            goal_handle->abort(result);
        } catch (...) {
        }
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ReferenceFilterNode)

}  // namespace vortex::guidance
