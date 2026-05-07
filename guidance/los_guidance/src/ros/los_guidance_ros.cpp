#include "los_guidance/ros/los_guidance_ros.hpp"
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <spdlog/spdlog.h>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <vortex/utils/math.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>

#ifdef NDEBUG
constexpr bool debug = false;
#else
constexpr bool debug = true;
#endif

const auto start_message = R"(
██╗      ██████╗ ███████╗     ██████╗ ██╗   ██╗██╗██████╗  █████╗ ███╗   ██╗ ██████╗███████╗
██║     ██╔═══██╗██╔════╝    ██╔════╝ ██║   ██║██║██╔══██╗██╔══██╗████╗  ██║██╔════╝██╔════╝
██║     ██║   ██║███████╗    ██║  ███╗██║   ██║██║██║  ██║███████║██╔██╗ ██║██║     █████╗
██║     ██║   ██║╚════██║    ██║   ██║██║   ██║██║██║  ██║██╔══██║██║╚██╗██║██║     ██╔══╝
███████╗╚██████╔╝███████║    ╚██████╔╝╚██████╔╝██║██████╔╝██║  ██║██║ ╚████║╚██████╗███████╗
╚══════╝ ╚═════╝ ╚══════╝     ╚═════╝  ╚═════╝ ╚═╝╚═════╝ ╚═╝  ╚═╝╚═╝  ╚═══╝ ╚═════╝╚══════╝
)";

namespace vortex::guidance::los {

// Constructor
LosGuidanceNode::LosGuidanceNode(const rclcpp::NodeOptions& options)
    : Node("los_guidance_node", options) {
    double time_step_s = 0.1;
    time_step_ =
        std::chrono::milliseconds(static_cast<int>(time_step_s * 1000));

    const std::string yaml_path =
        this->declare_parameter<std::string>("los_config_file_path");

    // Initialize the state manager
    state_manager_ = std::make_unique<LosGuidanceStateManager>(yaml_path);

    set_subscribers_and_publisher();
    set_action_server();
    set_service_server();

    spdlog::info(start_message);
}

// Subscribers + publishers
void LosGuidanceNode::set_subscribers_and_publisher() {
    this->declare_parameter<std::string>("topics.pose");
    this->declare_parameter<std::string>("topics.guidance.los");
    this->declare_parameter<std::string>("topics.waypoint");
    this->declare_parameter<std::string>("topics.odom");
    this->declare_parameter<std::string>(
        "topics.odom_tf_rpy", "/utils/message_publisher/odom_tf_rpy");

    std::string pose_topic = this->get_parameter("topics.pose").as_string();
    std::string guidance_topic =
        this->get_parameter("topics.guidance.los").as_string();
    std::string waypoint_topic =
        this->get_parameter("topics.waypoint").as_string();
    std::string odom_topic = this->get_parameter("topics.odom").as_string();
    std::string odom_tf_rpy_topic =
        this->get_parameter("topics.odom_tf_rpy").as_string();

    auto qos_sensor_data = vortex::utils::qos_profiles::sensor_data_profile(1);

    reference_pub_ = this->create_publisher<vortex_msgs::msg::LOSGuidance>(
        guidance_topic, qos_sensor_data);

    state_debug_pub_ = this->create_publisher<vortex_msgs::msg::LOSGuidance>(
        "state_debug", qos_sensor_data);

    waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        waypoint_topic, qos_sensor_data,
        std::bind(&LosGuidanceNode::waypoint_callback, this,
                  std::placeholders::_1));

    pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, qos_sensor_data,
        std::bind(&LosGuidanceNode::pose_callback, this,
                  std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, qos_sensor_data,
        std::bind(&LosGuidanceNode::odom_callback, this,
                  std::placeholders::_1));

    message_pub_sub_ =
        this->create_subscription<vortex_msgs::msg::PoseEulerStamped>(
            odom_tf_rpy_topic, qos_sensor_data,
            std::bind(&LosGuidanceNode::odom_msg_callback, this,
                      std::placeholders::_1));
}

// Action server setup
void LosGuidanceNode::set_action_server() {
    this->declare_parameter<std::string>("action_servers.los");
    std::string action_server_name =
        this->get_parameter("action_servers.los").as_string();

    cb_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    action_server_ =
        rclcpp_action::create_server<vortex_msgs::action::GuidanceWaypoint>(
            this, action_server_name,
            std::bind(&LosGuidanceNode::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&LosGuidanceNode::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&LosGuidanceNode::handle_accepted, this,
                      std::placeholders::_1),
            rcl_action_server_get_default_options(), cb_group_);
}

// Service server setup
void LosGuidanceNode::set_service_server() {
    this->declare_parameter<std::string>("services.los_mode", "set_los_mode");
    std::string service_name =
        this->get_parameter("services.los_mode").as_string();

    los_mode_service_ = this->create_service<vortex_msgs::srv::SetLosMode>(
        service_name, std::bind(&LosGuidanceNode::set_los_mode, this,
                                std::placeholders::_1, std::placeholders::_2));
}

// Waypoint callback
void LosGuidanceNode::waypoint_callback(
    const geometry_msgs::msg::PointStamped::SharedPtr wp_msg) {
    const auto new_wp = types::Point::point_from_ros(wp_msg->point);

    state_manager_->update_waypoint(new_wp);

    spdlog::info("Received waypoint: ({}, {}, {})", new_wp.x, new_wp.y,
                 new_wp.z);
}

// Pose callback
void LosGuidanceNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
        current_pose) {
    types::Point position =
        types::Point::point_from_ros(current_pose->pose.pose.position);

    state_manager_->update_position(position);
}

// Odometry callback
void LosGuidanceNode::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::unique_lock<std::mutex> lock(mutex_);
    debug_current_odom_ = msg;
    lock.unlock();
}

// Euler (yaw) callback
void LosGuidanceNode::odom_msg_callback(
    const vortex_msgs::msg::PoseEulerStamped::SharedPtr msg) {
    state_manager_->update_yaw(msg->yaw);
}

// Goal handler
rclcpp_action::GoalResponse LosGuidanceNode::handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const vortex_msgs::action::GuidanceWaypoint::Goal> goal) {
    if (!state_manager_->is_goal_feasible(goal)) {
        RCLCPP_WARN(this->get_logger(),
                    "Rejected goal request: waypoint is not reachable with "
                    "current pitch limit");
        return rclcpp_action::GoalResponse::REJECT;
    }

    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (goal_handle_ && goal_handle_->is_active()) {
            RCLCPP_INFO(this->get_logger(),
                        "Aborting current goal and accepting new goal");
            preempted_goal_id_ = goal_handle_->get_goal_id();
            lock.unlock();
        }
    }

    RCLCPP_INFO(this->get_logger(), "Accepted goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// Cancel handler
rclcpp_action::CancelResponse LosGuidanceNode::handle_cancel(
    const std::shared_ptr<GoalHandleGuidanceWaypoint> goal_handle) {
    spdlog::info("Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

// Accepted handler
void LosGuidanceNode::handle_accepted(
    const std::shared_ptr<GoalHandleGuidanceWaypoint> goal_handle) {
    std::thread{[this, goal_handle]() { execute(goal_handle); }}.detach();
}

// Service callback
void LosGuidanceNode::set_los_mode(
    const std::shared_ptr<vortex_msgs::srv::SetLosMode::Request> request,
    std::shared_ptr<vortex_msgs::srv::SetLosMode::Response> response) {
    state_manager_->set_los_method(
        static_cast<types::ActiveLosMethod>(request->mode));

    spdlog::info("LOS mode set to {}", static_cast<int>(request->mode));
    response->success = true;
}

// Fill LOS reference message
vortex_msgs::msg::LOSGuidance LosGuidanceNode::fill_los_reference(
    types::Outputs outputs) {
    vortex_msgs::msg::LOSGuidance reference_msg;

    double max_pitch_angle = state_manager_->get_max_pitch_angle();
    double current_yaw = state_manager_->get_current_yaw();
    double u_desired = state_manager_->get_u_desired();

    const double clamped_pitch =
        std::clamp(outputs.theta_d, -max_pitch_angle, max_pitch_angle);

    reference_msg.pitch = clamped_pitch;
    reference_msg.yaw = outputs.psi_d;

    double yaw_error = vortex::utils::math::ssa(outputs.psi_d - current_yaw);
    double abs_err = std::abs(yaw_error);

    double u_cmd = u_desired / (1.0 + 0.5 * abs_err);
    u_cmd = std::clamp(u_cmd, 0.15, u_desired);

    reference_msg.surge = u_cmd;

    return reference_msg;
}

// Execute action
void LosGuidanceNode::execute(
    const std::shared_ptr<GoalHandleGuidanceWaypoint> goal_handle) {
    {
        std::unique_lock<std::mutex> lock(mutex_);
        this->goal_handle_ = goal_handle;
        lock.unlock();
    }

    spdlog::info("Executing goal");

    const geometry_msgs::msg::Point los_waypoint =
        goal_handle->get_goal()->waypoint.pose.position;

    const auto new_wp = types::Point::point_from_ros(los_waypoint);

    state_manager_->initialize_goal(new_wp);

    auto result =
        std::make_shared<vortex_msgs::action::GuidanceWaypoint::Result>();

    rclcpp::Rate loop_rate(1000.0 / time_step_.count());

    while (rclcpp::ok()) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (goal_handle->get_goal_id() == preempted_goal_id_) {
                result->success = false;
                goal_handle->abort(result);
                return;
            }
            lock.unlock();
        }

        if (goal_handle->is_canceling()) {
            result->success = false;
            goal_handle->canceled(result);
            spdlog::info("Goal canceled");
            return;
        }

        nav_msgs::msg::Odometry::SharedPtr odom_copy;
        double goal_reached_tol_copy;

        {
            std::unique_lock<std::mutex> lock(mutex_);
            odom_copy = debug_current_odom_;
            goal_reached_tol_copy =
                goal_handle->get_goal()->convergence_threshold;
            lock.unlock();
        }

        if (state_manager_->is_goal_missed()) {
            result->success = false;
            goal_handle->abort(result);
            spdlog::info("Aborting goal: waypoint missed");
            return;
        }

        types::Outputs outputs = state_manager_->calculate_outputs();

        auto reference_msg = std::make_unique<vortex_msgs::msg::LOSGuidance>(
            fill_los_reference(outputs));

        if (state_manager_->is_goal_reached(goal_reached_tol_copy)) {
            reference_msg->pitch = 0.0;
            reference_msg->surge = 0.0;

            result->success = true;
            goal_handle->succeed(result);
            spdlog::info("Goal reached");
            return;
        }

        reference_pub_->publish(std::move(reference_msg));

        if (debug && odom_copy) {
            const auto& v = odom_copy->twist.twist.linear;
            double surge = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);

            vortex_msgs::msg::LOSGuidance state_debug_msg;
            Eigen::Vector3d euler = vortex::utils::math::quat_to_euler(
                Eigen::Quaterniond(odom_copy->pose.pose.orientation.w,
                                   odom_copy->pose.pose.orientation.x,
                                   odom_copy->pose.pose.orientation.y,
                                   odom_copy->pose.pose.orientation.z));

            state_debug_msg.pitch = euler.y();
            state_debug_msg.yaw = euler.z();
            state_debug_msg.surge = surge;

            state_debug_pub_->publish(state_debug_msg);
        }

        loop_rate.sleep();
    }
}

}  // namespace vortex::guidance::los

RCLCPP_COMPONENTS_REGISTER_NODE(vortex::guidance::los::LosGuidanceNode)
