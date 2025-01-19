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
    declare_parameter("physical.center_of_mass", std::vector<double>{0});
    declare_parameter("propulsion.dimensions.num", 3);
    declare_parameter("propulsion.thrusters.num", 8);
    declare_parameter("propulsion.thrusters.min", -100);
    declare_parameter("propulsion.thrusters.max", 100);
    declare_parameter("propulsion.thrusters.thrust_update_rate", 10.0);
    declare_parameter("propulsion.thrusters.thruster_force_direction",
                      std::vector<double>{0});
    declare_parameter("propulsion.thrusters.thruster_position",
                      std::vector<double>{0});

    center_of_mass_ = double_array_to_eigen_vector3d(
        get_parameter("physical.center_of_mass").as_double_array());

    num_dimensions_ = get_parameter("propulsion.dimensions.num").as_int();
    num_thrusters_ = get_parameter("propulsion.thrusters.num").as_int();
    min_thrust_ = get_parameter("propulsion.thrusters.min").as_int();
    max_thrust_ = get_parameter("propulsion.thrusters.max").as_int();
    thrust_update_period_ = std::chrono::milliseconds(static_cast<int>(
        1000 /
        get_parameter("propulsion.thrusters.thrust_update_rate").as_double()));

    thruster_force_direction_ = double_array_to_eigen_matrix(
        get_parameter("propulsion.thrusters.thruster_force_direction")
            .as_double_array(),
        num_dimensions_, num_thrusters_);

    thruster_position_ = double_array_to_eigen_matrix(
        get_parameter("propulsion.thrusters.thruster_position")
            .as_double_array(),
        num_dimensions_, num_thrusters_);

    thrust_configuration_ = calculate_thrust_allocation_matrix(
        thruster_force_direction_, thruster_position_, center_of_mass_);

    wrench_subscriber_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        "thrust/wrench_input", 1,
        std::bind(&ThrustAllocator::wrench_cb, this, std::placeholders::_1));

    thruster_forces_publisher_ =
        this->create_publisher<vortex_msgs::msg::ThrusterForces>(
            "thrust/thruster_forces", 5);

    calculate_thrust_timer_ = this->create_wall_timer(
        thrust_update_period_,
        std::bind(&ThrustAllocator::calculate_thrust_timer_cb, this));

    pseudoinverse_allocator_.T_pinv =
        calculate_right_pseudoinverse(thrust_configuration_);

    body_frame_forces_.setZero();
}

void ThrustAllocator::calculate_thrust_timer_cb() {
    Eigen::VectorXd thruster_forces =
        pseudoinverse_allocator_.calculate_allocated_thrust(body_frame_forces_);

    if (is_invalid_matrix(thruster_forces)) {
        RCLCPP_ERROR(get_logger(), "ThrusterForces vector invalid");
        return;
    }

    if (!saturate_vector_values(thruster_forces, min_thrust_, max_thrust_)) {
        RCLCPP_WARN(get_logger(),
                    "Thruster forces vector required saturation.");
    }

    vortex_msgs::msg::ThrusterForces msg_out;
    array_eigen_to_msg(thruster_forces, msg_out);
    thruster_forces_publisher_->publish(msg_out);
}

void ThrustAllocator::wrench_cb(const geometry_msgs::msg::Wrench& msg) {
    Eigen::Vector6d msg_vector;
    msg_vector(0) = msg.force.x;   // surge
    msg_vector(1) = msg.force.y;   // sway
    msg_vector(2) = msg.force.z;   // heave
    msg_vector(3) = msg.torque.x;  // roll
    msg_vector(4) = msg.torque.y;  // pitch
    msg_vector(5) = msg.torque.z;  // yaw

    if (!healthy_wrench(msg_vector)) {
        RCLCPP_ERROR(get_logger(), "ASV wrench vector invalid, ignoring.");
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
