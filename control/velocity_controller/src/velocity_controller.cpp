#include "velocity_controller/velocity_controller.hpp"
#include <rmw/types.h>
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <lifecycle_msgs/msg/detail/transition__struct.hpp>
#include <numbers>
#include <rclcpp/utilities.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <vector>
#include "velocity_controller/PID_setup.hpp"
#include "velocity_controller/utilities.hpp"
#include "vortex/utils/math.hpp"

Velocity_node::Velocity_node(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("velocity_controller_lifecycle", options),
      lqr_controller(),
      pub_QoS(10),
      sub_QoS(10) {
    get_new_parameters();
    pub_QoS.keep_last(10)
        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    sub_QoS.keep_last(10)
        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // PID initialization
    PID_surge.set_output_limits(-max_force, max_force);
    PID_pitch.set_output_limits(-max_force, max_force);
    PID_yaw.set_output_limits(-max_force, max_force);
    PID_surge.set_parameters(surge_params, publish_rate / 1000.0);
    PID_pitch.set_parameters(pitch_params, publish_rate / 1000.0);
    PID_yaw.set_parameters(yaw_params, publish_rate / 1000.0);

    // LQR
    if (!lqr_controller.set_matrices(Q, R, inertia_matrix, max_force,
                                     dampening_matrix_low) ||
        !lqr_controller.set_interval(static_cast<double>(publish_rate) /
                                     1000)) {
        controller_type = 1;
        RCLCPP_INFO(this->get_logger(), "Switching to PID");
    };
    // Automatically start in activate if auto_start is true
    if (auto_start) {
        startup_timer_ =
            create_wall_timer(std::chrono::milliseconds(0), [this]() {
                startup_timer_->cancel();
                trigger_transition(
                    lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
            });
    }
    RCLCPP_INFO(this->get_logger(), "Velocity control node has been started.");

    return;
}

void Velocity_node::calc_thrust() {
    if (odometry_dropout_guard) {
        publish_counter++;
        if (publish_counter >= 100) {
            reset_controllers();
            RCLCPP_WARN(this->get_logger(), "Odometry dropout, no thrust");
            return;
        }
    }
    // TODO(henrimha): Do I need ssa here?
    angle ref_in_body =
        angle_NED_to_body({0, vortex::utils::math::ssa(guidance_values.pitch),
                           vortex::utils::math::ssa(guidance_values.yaw)},
                          current_state.get_angle());
    Guidance_data error = {guidance_values.surge - current_state.surge,
                           -ref_in_body.thetat, -ref_in_body.psit};

    if (anti_overshoot) {
        if (abs(error.yaw) < std::numbers::pi / 2 ||
            abs(error.yaw) < std::numbers::pi / 2) {
            error.surge =
                guidance_values.surge * cos(error.yaw) * cos(error.pitch);
        }
    }
    switch (controller_type) {
        case 1: {
            PID_surge.calculate_thrust(error.surge);
            PID_pitch.calculate_thrust(error.pitch, -current_state.pitch_rate);
            PID_yaw.calculate_thrust(error.yaw, -current_state.yaw_rate);
            thrust_out.wrench.force.x = PID_surge.get_output();
            thrust_out.wrench.torque.y = PID_pitch.get_output();
            thrust_out.wrench.torque.z = PID_yaw.get_output();

            break;
        }
        case 2: {
            if (!lqr_controller.calculate_thrust(current_state, error)) {
                controller_type = 1;
                RCLCPP_ERROR(this->get_logger(), "Switching to PID");
            } else {
                Eigen::Vector3d u = lqr_controller.get_thrust();
                thrust_out.wrench.force.x = u[0];
                thrust_out.wrench.torque.y = u[1];
                thrust_out.wrench.torque.z = u[2];
            }
            break;
        }
        default: {
            RCLCPP_ERROR(this->get_logger(),
                         "Unknown controller set, switching to PID");
            controller_type = 1;
            calc_thrust();
            return;
        }
    }
    publisher_thrust->publish(thrust_out);
    return;
}

// Callback functions
void Velocity_node::guidance_callback(
    const vortex_msgs::msg::LOSGuidance::SharedPtr msg_ptr) {
    if (reset_on_new_ref) {  // On big step changes, reset the controllers to
                             // avoid big overshoots
        if (abs(msg_ptr->surge - guidance_values.surge) >= 0.1)
            reset_controllers(1);
        if (abs(msg_ptr->pitch - guidance_values.pitch) > std::numbers::pi / 4)
            reset_controllers(2);
        if (abs(msg_ptr->yaw - guidance_values.yaw) < std::numbers::pi / 4)
            reset_controllers(3);
    }
    guidance_values = msg_ptr;  // overloaded to fix all the internal states

    return;
}

void Velocity_node::odometry_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg_ptr) {
    publish_counter = 0;
    current_state = msg_ptr;  // overloaded to fix all the internal states
    return;
}

void Velocity_node::get_new_parameters() {
    // topics
    this->declare_parameter<std::string>("topics.wrench_input");
    topic_thrust = this->get_parameter("topics.wrench_input").as_string();
    this->declare_parameter<std::string>("topics.guidance.los");
    topic_guidance = this->get_parameter("topics.guidance.los").as_string();
    this->declare_parameter<std::string>("topics.odom");
    topic_odometry = this->get_parameter("topics.odom").as_string();

    // Settings
    this->declare_parameter<double>("Settings.max_force");
    max_force = this->get_parameter("Settings.max_force").as_double();
    this->declare_parameter<int>("Settings.publish_rate");
    publish_rate = this->get_parameter("Settings.publish_rate").as_int();
    this->declare_parameter<int>("Settings.controller_type");
    controller_type = this->get_parameter("Settings.controller_type").as_int();
    this->declare_parameters<bool>("Settings",
                                   {{"auto_start", false},
                                    {"reset_on_new_ref", true},
                                    {"anti_overshoot", true},
                                    {"odometry_dropout_guard", true}});
    auto settings = this->get_parameters(
        {"Settings.auto_start", "Settings.reset_on_new_ref",
         "Settings.anti_overshoot", "Settings.odometry_dropout_guard"});
    auto_start = settings[0].as_bool();
    reset_on_new_ref = settings[1].as_bool();
    anti_overshoot = settings[2].as_bool();
    odometry_dropout_guard = settings[3].as_bool();

    // PID Params
    this->declare_parameter<std::vector<double>>("PID_params.surge");
    surge_params = this->get_parameter("PID_params.surge").as_double_array();
    this->declare_parameter<std::vector<double>>("PID_params.pitch");
    pitch_params = this->get_parameter("PID_params.pitch").as_double_array();
    this->declare_parameter<std::vector<double>>("PID_params.yaw");
    yaw_params = this->get_parameter("PID_params.yaw").as_double_array();

    // LQR Parameters

    this->declare_parameter<std::vector<double>>("LQR_params.Q");
    Q = this->get_parameter("LQR_params.Q").as_double_array();
    this->declare_parameter<std::vector<double>>("LQR_params.R");
    R = this->get_parameter("LQR_params.R").as_double_array();
    this->declare_parameter<std::vector<double>>("physical.mass_matrix");
    inertia_matrix =
        this->get_parameter("physical.mass_matrix").as_double_array();

    // D
    this->declare_parameter<std::vector<double>>("dampening_matrix_low");
    this->declare_parameter<std::vector<double>>("dampening_matrix_high");
    this->dampening_matrix_low =
        this->get_parameter("dampening_matrix_low").as_double_array();
    this->dampening_matrix_high =
        this->get_parameter("dampening_matrix_high").as_double_array();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Velocity_node::on_configure(const rclcpp_lifecycle::State&) {
    RCLCPP_INFO(get_logger(), "Configure VC");

    // Publishers
    publisher_thrust = create_publisher<geometry_msgs::msg::WrenchStamped>(
        topic_thrust, pub_QoS);

    // Subscribers
    subscriber_Odometry = this->create_subscription<nav_msgs::msg::Odometry>(
        topic_odometry, sub_QoS,
        std::bind(&Velocity_node::odometry_callback, this,
                  std::placeholders::_1));
    subscriber_guidance =
        this->create_subscription<vortex_msgs::msg::LOSGuidance>(
            topic_guidance, sub_QoS,
            std::bind(&Velocity_node::guidance_callback, this,
                      std::placeholders::_1));
    // Timer
    if (first_start && auto_start) {
        startup_timer_ =
            create_wall_timer(std::chrono::milliseconds(0), [this]() {
                startup_timer_->cancel();
                trigger_transition(
                    lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
            });
    }
    first_start = false;
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Velocity_node::on_activate(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(get_logger(), "Activating...");
    timer_calculation =
        this->create_wall_timer(std::chrono::milliseconds(publish_rate),
                                std::bind(&Velocity_node::calc_thrust, this));
    auto ret = LifecycleNode::on_activate(state);

    return ret;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Velocity_node::on_deactivate(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(get_logger(), "Deactivating...");
    auto ret = LifecycleNode::on_deactivate(state);
    timer_calculation.reset();
    reset_controllers();
    return ret;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Velocity_node::on_cleanup(const rclcpp_lifecycle::State&) {
    RCLCPP_INFO(get_logger(), "Cleaning up...");
    timer_calculation.reset();
    publisher_thrust.reset();
    subscriber_guidance.reset();
    subscriber_Odometry.reset();
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Velocity_node::on_shutdown(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(get_logger(), "Shutting down from state %s",
                state.label().c_str());
    if (timer_calculation)
        timer_calculation->cancel();
    timer_calculation.reset();
    publisher_thrust.reset();
    subscriber_guidance.reset();
    subscriber_Odometry.reset();
    should_exit_ = true;
    return CallbackReturn::SUCCESS;
}

void Velocity_node::reset_controllers(int nr) {
    switch (nr) {
        case 0:
            PID_pitch.reset_controller();
            PID_surge.reset_controller();
            PID_yaw.reset_controller();
            lqr_controller.reset_controller();
            break;
        case 1:
            PID_surge.reset_controller();
            lqr_controller.reset_controller(1);
            break;

        case 2:
            PID_pitch.reset_controller();
            lqr_controller.reset_controller(2);
            break;

        case 3:
            PID_yaw.reset_controller();
            lqr_controller.reset_controller(3);
            break;
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(Velocity_node)
