#include "los_guidance/los_guidance_ros.hpp"
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <spdlog/spdlog.h>
#include <yaml-cpp/node/node.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>

#include "los_guidance/lib/types.hpp"

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
    double time_step_s = this->declare_parameter<double>("time_step");
    time_step_ = std::chrono::milliseconds(static_cast<int>(
        time_step_s * 1000));  // Convert seconds to milliseconds

    const std::string yaml_path =
        this->declare_parameter<std::string>("los_config_file");

    YAML::Node config = get_los_config(yaml_path);

    parse_common_config(config["common"]);
    set_subscribers_and_publisher();
    set_action_server();
    set_service_server();
    set_adaptive_los_guidance(config);
    set_proportional_los_guidance(config);
    set_integral_los_guidance(config);
    set_vector_field_guidance(config);

    spdlog::info(start_message);
}

void LosGuidanceNode::set_subscribers_and_publisher() {
    this->declare_parameter<std::string>("topics.pose");
    this->declare_parameter<std::string>("topics.guidance.los");
    this->declare_parameter<std::string>("topics.waypoint");
    this->declare_parameter<std::string>("topics.odom");

    std::string pose_topic = this->get_parameter("topics.pose").as_string();
    std::string guidance_topic =
        this->get_parameter("topics.guidance.los").as_string();
    std::string waypoint_topic =
        this->get_parameter("topics.waypoint").as_string();
    std::string odom_topic = this->get_parameter("topics.odom").as_string();

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
}

void LosGuidanceNode::set_action_server() {
    this->declare_parameter<std::string>("action_servers.los");
    std::string action_server_name =
        this->get_parameter("action_servers.los").as_string();

    cb_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    action_server_ =
        rclcpp_action::create_server<vortex_msgs::action::LOSGuidance>(
            this, action_server_name,
            std::bind(&LosGuidanceNode::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&LosGuidanceNode::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&LosGuidanceNode::handle_accepted, this,
                      std::placeholders::_1),
            rcl_action_server_get_default_options(), cb_group_);
}

void LosGuidanceNode::set_service_server() {
    this->declare_parameter<std::string>("services.los_mode", "set_los_mode");
    std::string service_name =
        this->get_parameter("services.los_mode").as_string();

    los_mode_service_ = this->create_service<vortex_msgs::srv::SetLosMode>(
        service_name, std::bind(&LosGuidanceNode::set_los_mode, this,
                                std::placeholders::_1, std::placeholders::_2));
}

void LosGuidanceNode::set_adaptive_los_guidance(YAML::Node config) {
    auto adaptive_los_config = config["adaptive_los"];
    auto params = AdaptiveLosParams{};

    try {
        params.lookahead_distance_h =
            adaptive_los_config["lookahead_distance_h"].as<double>();
        params.lookahead_distance_v =
            adaptive_los_config["lookahead_distance_v"].as<double>();
        params.adaptation_gain_h =
            adaptive_los_config["adaptation_gain_h"].as<double>();
        params.adaptation_gain_v =
            adaptive_los_config["adaptation_gain_v"].as<double>();
        params.time_step = static_cast<double>(time_step_.count()) / 1000.0;

        adaptive_los_ = std::make_unique<AdaptiveLOSGuidance>(params);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(
            std::string("Failed to load adaptive_los parameters: ") + e.what());
    }
}

void LosGuidanceNode::set_proportional_los_guidance(YAML::Node config) {
    auto proportional_los_config = config["prop_los"];
    auto params = ProportionalLosParams{};

    try {
        params.lookahead_distance_h =
            proportional_los_config["lookahead_distance_h"].as<double>();
        params.lookahead_distance_v =
            proportional_los_config["lookahead_distance_v"].as<double>();

        proportional_los_ = std::make_unique<ProportionalLOSGuidance>(params);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(
            std::string("Failed to load proportional_los parameters: ") +
            e.what());
    }
}

void LosGuidanceNode::set_integral_los_guidance(YAML::Node config) {
    auto integral_los_config = config["integer_los"];
    auto params = IntegralLosParams{};

    try {
        params.proportional_gain_h =
            integral_los_config["proportional_gain_h"].as<double>();
        params.proportional_gain_v =
            integral_los_config["proportional_gain_v"].as<double>();
        params.integral_gain_h =
            integral_los_config["integral_gain_h"].as<double>();
        params.integral_gain_v =
            integral_los_config["integral_gain_v"].as<double>();
        params.time_step = static_cast<double>(time_step_.count()) / 1000.0;

        integral_los_ = std::make_unique<IntegralLOSGuidance>(params);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(
            std::string("Failed to load integral_los parameters: ") + e.what());
    }
}

void LosGuidanceNode::set_vector_field_guidance(YAML::Node config) {
    auto vector_field_config = config["vector_field_los"];
    auto params = VectorFieldLosParams{};

    try {
        params.max_approach_angle_h =
            vector_field_config["max_approach_angle_h"].as<double>();
        params.max_approach_angle_v =
            vector_field_config["max_approach_angle_v"].as<double>();
        params.proportional_gain_h =
            vector_field_config["proportional_gain_h"].as<double>();
        params.proportional_gain_v =
            vector_field_config["proportional_gain_v"].as<double>();
        params.time_step = static_cast<double>(time_step_.count()) / 1000.0;

        vector_field_los_ = std::make_unique<VectorFieldLOSGuidance>(params);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(
            std::string("Failed to load vector_field_los parameters: ") +
            e.what());
    }
}

void LosGuidanceNode::waypoint_callback(
    const geometry_msgs::msg::PointStamped::SharedPtr wp_msg) {
    std::unique_lock<std::mutex> lock(mutex_);

    const auto new_wp = types::Point::point_from_ros(wp_msg->point);

    if (!has_active_segment_) {
        path_inputs_.prev_point = path_inputs_.current_position;
        path_inputs_.next_point = new_wp;
        has_active_segment_ = true;
    } else {
        path_inputs_.prev_point = path_inputs_.next_point;
        path_inputs_.next_point = new_wp;
    }

    lock.unlock();

    spdlog::info("Received waypoint: ({}, {}, {})", new_wp.x, new_wp.y,
                 new_wp.z);
}

void LosGuidanceNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
        current_pose) {
    std::unique_lock<std::mutex> lock(mutex_);
    path_inputs_.current_position =
        types::Point::point_from_ros(current_pose->pose.pose.position);
    lock.unlock();
}

void LosGuidanceNode::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::unique_lock<std::mutex> lock(mutex_);
    debug_current_odom_ = msg;
    lock.unlock();
}

rclcpp_action::GoalResponse LosGuidanceNode::handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const vortex_msgs::action::LOSGuidance::Goal> goal) {
    types::Inputs inputs_copy;

    {
        std::unique_lock<std::mutex> lock(mutex_);
        inputs_copy = path_inputs_;
        lock.unlock();
    }

    if (!is_goal_feasible(inputs_copy, goal)) {
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
        }
        lock.unlock();
    }

    RCLCPP_INFO(this->get_logger(), "Accepted goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LosGuidanceNode::handle_cancel(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::LOSGuidance>>
        goal_handle) {
    spdlog::info("Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void LosGuidanceNode::handle_accepted(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::LOSGuidance>>
        goal_handle) {
    std::thread{[this, goal_handle]() { execute(goal_handle); }}.detach();
}

void LosGuidanceNode::set_los_mode(
    const std::shared_ptr<vortex_msgs::srv::SetLosMode::Request> request,
    std::shared_ptr<vortex_msgs::srv::SetLosMode::Response> response) {
    {
        std::unique_lock<std::mutex> lock(mutex_);
        method_ = static_cast<types::ActiveLosMethod>(request->mode);
        lock.unlock();
    }

    spdlog::info("LOS mode set to {}", static_cast<int>(request->mode));
    response->success = true;
}

vortex_msgs::msg::LOSGuidance LosGuidanceNode::fill_los_reference(
    types::Outputs outputs) {
    vortex_msgs::msg::LOSGuidance reference_msg;

    const double clamped_pitch =
        std::clamp(outputs.theta_d, -max_pitch_angle_, max_pitch_angle_);
    reference_msg.pitch = clamped_pitch;
    reference_msg.yaw = outputs.psi_d;
    reference_msg.surge = u_desired_;
    return reference_msg;
}

bool LosGuidanceNode::is_goal_feasible(
    const types::Inputs& inputs,
    std::shared_ptr<const vortex_msgs::action::LOSGuidance::Goal> goal) {
    const auto& current_position = inputs.current_position;
    const auto& goal_point = goal->goal.point;

    const double dx = goal_point.x - current_position.x;
    const double dy = goal_point.y - current_position.y;
    const double dz = goal_point.z - current_position.z;

    const double horizontal_distance = std::sqrt(dx * dx + dy * dy);
    const double required_pitch = std::atan2(-dz, horizontal_distance);

    return std::abs(required_pitch) <= max_pitch_angle_;
}

bool LosGuidanceNode::is_goal_missed(const types::Inputs& inputs) {
    const double distance_to_goal =
        (inputs.current_position - inputs.next_point).as_vector().norm();

    const double dt = static_cast<double>(time_step_.count()) / 1000.0;

    if (distance_to_goal < nearest_been_to_goal_) {
        nearest_been_to_goal_ = distance_to_goal;
        time_since_nearest_goal_ = 0.0;
        return false;
    }

    if (distance_to_goal >
        nearest_been_to_goal_ + missed_goal_distance_margin_) {
        time_since_nearest_goal_ += dt;
    } else {
        time_since_nearest_goal_ = 0.0;
    }

    return time_since_nearest_goal_ >= missed_goal_timeout_;
}

YAML::Node LosGuidanceNode::get_los_config(std::string yaml_file_path) {
    try {
        YAML::Node config = YAML::LoadFile(yaml_file_path);
        return config;
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(
            std::string("Failed to load LOS config file '") + yaml_file_path +
            "': " + e.what());
    }
}

void LosGuidanceNode::parse_common_config(YAML::Node common_config) {
    try {
        std::unique_lock<std::mutex> lock(mutex_);
        u_desired_ = common_config["u_desired"].as<double>();
        max_pitch_angle_ = common_config["max_pitch_angle"].as<double>();
        goal_reached_tol_ = common_config["goal_reached_tol"].as<double>();
        missed_goal_timeout_ =
            common_config["missed_goal_timeout"].as<double>();
        missed_goal_distance_margin_ =
            common_config["missed_goal_distance_margin"].as<double>();

        method_ = static_cast<types::ActiveLosMethod>(
            common_config["active_los_method"].as<int>());

        lock.unlock();
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(
            std::string("Failed to load common parameters: ") + e.what());
    }
}

void LosGuidanceNode::execute(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::LOSGuidance>>
        goal_handle) {
    {
        std::unique_lock<std::mutex> lock(mutex_);
        this->goal_handle_ = goal_handle;
        lock.unlock();
    }

    spdlog::info("Executing goal");

    const geometry_msgs::msg::PointStamped los_waypoint =
        goal_handle->get_goal()->goal;

    const auto new_wp = types::Point::point_from_ros(los_waypoint.point);

    {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!has_active_segment_) {
            path_inputs_.prev_point = path_inputs_.current_position;
            path_inputs_.next_point = new_wp;
            has_active_segment_ = true;
        } else {
            path_inputs_.prev_point = path_inputs_.next_point;
            path_inputs_.next_point = new_wp;
        }
        lock.unlock();
    }

    auto result = std::make_shared<vortex_msgs::action::LOSGuidance::Result>();
    nearest_been_to_goal_ = std::numeric_limits<double>::infinity();
    time_since_nearest_goal_ = 0.0;

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

        types::Inputs inputs_copy;
        types::ActiveLosMethod method_copy;
        nav_msgs::msg::Odometry::SharedPtr odom_copy;
        double goal_reached_tol_copy;

        {
            std::unique_lock<std::mutex> lock(mutex_);
            inputs_copy = path_inputs_;
            method_copy = method_;
            odom_copy = debug_current_odom_;
            goal_reached_tol_copy = goal_reached_tol_;
            lock.unlock();
        }

        if (is_goal_missed(inputs_copy)) {
            result->success = false;
            goal_handle->abort(result);
            spdlog::info("Aborting goal: waypoint missed");
            return;
        }

        types::Outputs outputs;

        switch (method_copy) {
            case types::ActiveLosMethod::ADAPTIVE:
                outputs = adaptive_los_->calculate_outputs(inputs_copy);
                break;
            case types::ActiveLosMethod::PROPORTIONAL:
                outputs = proportional_los_->calculate_outputs(inputs_copy);
                break;
            case types::ActiveLosMethod::INTEGRAL:
                outputs = integral_los_->calculate_outputs(inputs_copy);
                break;
            case types::ActiveLosMethod::VECTOR_FIELD:
                outputs = vector_field_los_->calculate_outputs(inputs_copy);
                break;
            default:
                spdlog::error("Invalid LOS method selected");
                result->success = false;
                goal_handle->abort(result);
                return;
        }

        auto reference_msg = std::make_unique<vortex_msgs::msg::LOSGuidance>(
            fill_los_reference(outputs));

        if ((inputs_copy.current_position - inputs_copy.next_point)
                .as_vector()
                .norm() < goal_reached_tol_copy) {
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
