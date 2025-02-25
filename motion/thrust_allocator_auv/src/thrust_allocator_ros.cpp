#include "thrust_allocator_auv/thrust_allocator_ros.hpp"
#include <vortex_msgs/msg/thruster_forces.hpp>
#include "thrust_allocator_auv/pseudoinverse_allocator.hpp"
#include "thrust_allocator_auv/thrust_allocator_utils.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

ThrustAllocator::ThrustAllocator()
    : Node("thrust_allocator_node"),
      pseudoinverse_allocator_(Eigen::MatrixXd::Zero(6, 8)) {
    extract_parameters();

    thrust_update_period_ =
        std::chrono::milliseconds(static_cast<int>(1000 / thrust_update_rate_));

    set_allocator();

    set_subscriber_and_publisher();

    calculate_thrust_timer_ = this->create_wall_timer(
        thrust_update_period_,
        std::bind(&ThrustAllocator::calculate_thrust_timer_cb, this));

    body_frame_forces_.setZero();
    last_msg_time_ = this->now();
}

void ThrustAllocator::extract_parameters() {
    this->declare_parameter<std::vector<double>>("physical.center_of_mass");
    this->declare_parameter<int>("propulsion.dimensions.num");
    this->declare_parameter<int>("propulsion.thrusters.num");
    this->declare_parameter<int>("propulsion.thrusters.min");
    this->declare_parameter<int>("propulsion.thrusters.max");
    this->declare_parameter<double>("propulsion.thrusters.thrust_update_rate");
    this->declare_parameter<std::vector<double>>(
        "propulsion.thrusters.thruster_force_direction");
    this->declare_parameter<std::vector<double>>(
        "propulsion.thrusters.thruster_position");
    this->declare_parameter<double>("propulsion.thrusters.watchdog_timeout");

    center_of_mass_ = double_array_to_eigen_vector3d(
        this->get_parameter("physical.center_of_mass").as_double_array());
    num_dimensions_ = this->get_parameter("propulsion.dimensions.num").as_int();
    num_thrusters_ = this->get_parameter("propulsion.thrusters.num").as_int();
    min_thrust_ = this->get_parameter("propulsion.thrusters.min").as_int();
    max_thrust_ = this->get_parameter("propulsion.thrusters.max").as_int();
    thrust_update_rate_ =
        this->get_parameter("propulsion.thrusters.thrust_update_rate")
            .as_double();
    double timout_treshold_param =
        this->get_parameter("propulsion.thrusters.watchdog_timeout")
            .as_double();
    timeout_treshold_ = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::duration<double>(timout_treshold_param));

    this->declare_parameter<std::string>("topics.wrench_input");
    this->declare_parameter<std::string>("topics.thruster_forces");
}

void ThrustAllocator::set_allocator() {
    thruster_force_direction_ = double_array_to_eigen_matrix(
        this->get_parameter("propulsion.thrusters.thruster_force_direction")
            .as_double_array(),
        num_dimensions_, num_thrusters_);

    thruster_position_ = double_array_to_eigen_matrix(
        this->get_parameter("propulsion.thrusters.thruster_position")
            .as_double_array(),
        num_dimensions_, num_thrusters_);

    thrust_configuration_ = calculate_thrust_allocation_matrix(
        thruster_force_direction_, thruster_position_, center_of_mass_);

    pseudoinverse_allocator_.T_pinv =
        calculate_right_pseudoinverse(thrust_configuration_);
}

void ThrustAllocator::set_subscriber_and_publisher() {
    std::string wrench_input_topic =
        this->get_parameter("topics.wrench_input").as_string();
    std::string thruster_forces_topic =
        this->get_parameter("topics.thruster_forces").as_string();

    wrench_subscriber_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        wrench_input_topic, 1,
        std::bind(&ThrustAllocator::wrench_cb, this, std::placeholders::_1));

    thruster_forces_publisher_ =
        this->create_publisher<vortex_msgs::msg::ThrusterForces>(
            thruster_forces_topic, 5);
}

void ThrustAllocator::calculate_thrust_timer_cb() {
    if ((this->now() - last_msg_time_) > timeout_treshold_) {
        body_frame_forces_.setZero();
    }
    Eigen::VectorXd thruster_forces =
        pseudoinverse_allocator_.calculate_allocated_thrust(body_frame_forces_);

    if (is_invalid_matrix(thruster_forces)) {
        RCLCPP_ERROR(get_logger(), "ThrusterForces vector invalid");
        thruster_forces_publisher_->publish(vortex_msgs::msg::ThrusterForces());
        return;
    }

    if (!saturate_vector_values(thruster_forces, min_thrust_, max_thrust_)) {
        RCLCPP_WARN(get_logger(), "ThrusterForces vector required saturation.");
    }

    vortex_msgs::msg::ThrusterForces msg_out;
    array_eigen_to_msg(thruster_forces, msg_out);
    thruster_forces_publisher_->publish(msg_out);
}

void ThrustAllocator::wrench_cb(const geometry_msgs::msg::Wrench& msg) {
    last_msg_time_ = this->now();
    Eigen::Vector6d msg_vector = wrench_to_vector(msg);

    if (!healthy_wrench(msg_vector)) {
        RCLCPP_ERROR(get_logger(), "Wrench vector invalid, ignoring.");
        body_frame_forces_.setZero();
        return;
    }
    std::swap(msg_vector, body_frame_forces_);
}

bool ThrustAllocator::healthy_wrench(const Eigen::VectorXd& v) const {
    if (is_invalid_matrix(v))
        return false;

    bool within_max_thrust = std::none_of(
        v.begin(), v.end(),
        [this](double val) { return std::abs(val) > max_thrust_; });

    return within_max_thrust;
}
