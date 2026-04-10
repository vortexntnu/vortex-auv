#include "velocity_controller/velocity_controller_ros.hpp"
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
#include "velocity_controller/control_manager.hpp"
#include "velocity_controller/utilities.hpp"
#include "velocity_controller/lib/3DOF_PID.hpp"

Velocity_node::Velocity_node(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("velocity_controller_lifecycle", options),
      pub_QoS(10),
      sub_QoS(10) {
    get_new_parameters();
    initialize_controllers();
    pub_QoS.keep_last(10)
        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    sub_QoS.keep_last(10)
        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // Automatically start in activate if auto_start is true
    if (node_settings.auto_start) {
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
//TODO(henrimha): Split the ROS part and the c++ part into seperate files/classes
void Velocity_node::publish_thrust() {
    if (node_settings.odometry_dropout_guard) {
        publish_counter++;
        if (publish_counter >= 100) {
            control_manager_ptr->reset_controllers();
            RCLCPP_WARN(this->get_logger(), "Odometry dropout, no thrust");
            return;
        }
    }
    thrust_out = control_manager_ptr->get_output(guidance_values, current_state);
    /*geometry_msgs::msg::WrenchStamped test;
    test.wrench.force.set__y(25);*/
    publisher_thrust->publish(thrust_out);
    return;
}

// Callback functions
void Velocity_node::guidance_callback(
    const vortex_msgs::msg::LOSGuidance::SharedPtr msg_ptr) {
    if (node_settings.reset_on_new_ref) {  // On big step changes, reset the controllers to
                             // avoid big overshoots
        if (abs(msg_ptr->surge - guidance_values.surge) >= 0.1)
            control_manager_ptr->reset_controllers(1);
        if (abs(msg_ptr->pitch - guidance_values.pitch) > std::numbers::pi / 4)
            control_manager_ptr->reset_controllers(2);
        if (abs(msg_ptr->yaw - guidance_values.yaw) > std::numbers::pi / 4)
            control_manager_ptr->reset_controllers(3);
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
    node_settings.topic_thrust = this->get_parameter("topics.wrench_input").as_string();
    this->declare_parameter<std::string>("topics.guidance.los");
    node_settings.topic_guidance = this->get_parameter("topics.guidance.los").as_string();
    this->declare_parameter<std::string>("topics.odom");
    node_settings.topic_odometry = this->get_parameter("topics.odom").as_string();
    
    // Control manager settings
    this->declare_parameter<double>("Control_manager_settings.max_force");
    this->declare_parameter<int>("Control_manager_settings.publish_rate");
    this->declare_parameter<int>("Control_manager_settings.controller_type");
    this->declare_parameter<bool>("Control_manager_settings.anti_overshoot", true);

    //Node settings
    this->declare_parameter<bool>("Node_settings.auto_start", false);
    this->declare_parameter<bool>("Node_settings.reset_on_new_ref", true);
    this->declare_parameter<bool>("Node_settings.odometry_dropout_guard", true);
    
    // PID Params
    this->declare_parameter<std::vector<double>>("3DOF_PID_params.surge");
    this->declare_parameter<std::vector<double>>("3DOF_PID_params.pitch");
    this->declare_parameter<std::vector<double>>("3DOF_PID_params.yaw");

    // LQR Parameters
    this->declare_parameter<std::vector<double>>("LQR_params.Q");
    this->declare_parameter<std::vector<double>>("LQR_params.R");
    this->declare_parameter<std::vector<double>>("physical.mass_matrix");

    // D
    this->declare_parameter<std::vector<double>>("dampening_matrix_low");
    this->declare_parameter<std::vector<double>>("dampening_matrix_high");
    
}
void Velocity_node::initialize_controllers() {
    //TODO(henrimha): parameter validation or in control manager
    //Initialize Node
    node_settings.auto_start = this->get_parameter("Node_settings.auto_start").as_bool();
    node_settings.reset_on_new_ref = this->get_parameter("Node_settings.reset_on_new_ref").as_bool();
    node_settings.publish_rate = this->get_parameter("Control_manager_settings.publish_rate").as_int();
    node_settings.odometry_dropout_guard = this->get_parameter("Node_settings.odometry_dropout_guard").as_bool();


    //Initialize control manager parameters
    auto control_type = this->get_parameter("Control_manager_settings.controller_type").as_int();
    auto anti_overshoot = this->get_parameter("Control_manager_settings.anti_overshoot").as_bool();
    control_manager_params control_params(control_type, anti_overshoot, 1);


    //Some general parameters
    double max_force = this->get_parameter("Control_manager_settings.max_force").as_double();
    double dt = node_settings.publish_rate / 1000.0;  // Convert ms to seconds
    
    // Initialize 3DOF_PID params
    auto surge_gains = this->get_parameter("3DOF_PID_params.surge").as_double_array();
    auto pitch_gains = this->get_parameter("3DOF_PID_params.pitch").as_double_array();
    auto yaw_gains = this->get_parameter("3DOF_PID_params.yaw").as_double_array();
    PID_3DOF_params pid_3dof_params(surge_gains, pitch_gains, yaw_gains, dt, max_force, -max_force);

    //Initialize LQR controller params
    auto Q = this->get_parameter("LQR_params.Q").as_double_array();
    auto R = this->get_parameter("LQR_params.R").as_double_array();
    auto inertia_matrix = this->get_parameter("physical.mass_matrix").as_double_array();
    auto D_low = this->get_parameter("dampening_matrix_low").as_double_array();
    auto D_high = this->get_parameter("dampening_matrix_high").as_double_array();
    LQR_params lqr_params(Q, R, inertia_matrix, max_force, D_low, D_high, dt);
    
    //Initalize all the controllers in control manager
    control_manager_ptr = std::make_unique<control_manager>(control_params);
    control_manager_ptr->initialize_3DOF_controller(pid_3dof_params);
    control_manager_ptr->initialize_LQR_controller(lqr_params);
    return;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Velocity_node::on_configure(const rclcpp_lifecycle::State&) {
    RCLCPP_INFO(get_logger(), "Configure VC");

    // Publishers
    publisher_thrust = create_publisher<geometry_msgs::msg::WrenchStamped>(
        node_settings.topic_thrust, pub_QoS);

    // Subscribers
    subscriber_Odometry = this->create_subscription<nav_msgs::msg::Odometry>(
        node_settings.topic_odometry, sub_QoS,
        std::bind(&Velocity_node::odometry_callback, this,
                  std::placeholders::_1));
    subscriber_guidance =
        this->create_subscription<vortex_msgs::msg::LOSGuidance>(
            node_settings.topic_guidance, sub_QoS,
            std::bind(&Velocity_node::guidance_callback, this,
                      std::placeholders::_1));
    // Timer
    if (first_start && node_settings.auto_start) {
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
        this->create_wall_timer(std::chrono::milliseconds(node_settings.publish_rate),
                                std::bind(&Velocity_node::publish_thrust, this));
    auto ret = LifecycleNode::on_activate(state);

    return ret;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Velocity_node::on_deactivate(const rclcpp_lifecycle::State& state) {
    RCLCPP_INFO(get_logger(), "Deactivating...");
    auto ret = LifecycleNode::on_deactivate(state);
    timer_calculation.reset();
    control_manager_ptr->reset_controllers();
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

RCLCPP_COMPONENTS_REGISTER_NODE(Velocity_node)
