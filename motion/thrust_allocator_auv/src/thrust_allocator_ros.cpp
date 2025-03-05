#include "thrust_allocator_auv/thrust_allocator_ros.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include "thrust_allocator_auv/pseudoinverse_allocator.hpp"
#include "thrust_allocator_auv/thrust_allocator_utils.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

ThrustAllocator::ThrustAllocator(const rclcpp::NodeOptions& options)
    : Node("thrust_allocator_node", options),
      pseudoinverse_allocator_(Eigen::MatrixXd::Zero(6, 8)) {
    extract_parameters();

    set_allocator();

    set_subscriber_and_publisher();

    watchdog_timer_ = this->create_wall_timer(
        500ms, std::bind(&ThrustAllocator::watchdog_callback, this));
    last_msg_time_ = this->now();
}

void ThrustAllocator::extract_parameters() {
    this->declare_parameter<std::vector<double>>("physical.center_of_mass");
    this->declare_parameter<int>("propulsion.dimensions.num");
    this->declare_parameter<int>("propulsion.thrusters.num");
    this->declare_parameter<int>("propulsion.thrusters.min");
    this->declare_parameter<int>("propulsion.thrusters.max");
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
        calculate_pseudoinverse(thrust_configuration_);
}

void ThrustAllocator::set_subscriber_and_publisher() {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto best_effort_qos = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    std::string wrench_input_topic =
        this->get_parameter("topics.wrench_input").as_string();
    std::string thruster_forces_topic =
        this->get_parameter("topics.thruster_forces").as_string();

    wrench_subscriber_ =
        this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            wrench_input_topic, best_effort_qos,
            std::bind(&ThrustAllocator::wrench_cb, this,
                      std::placeholders::_1));

    thruster_forces_publisher_ =
        this->create_publisher<vortex_msgs::msg::ThrusterForces>(
            thruster_forces_topic, best_effort_qos);
}

void ThrustAllocator::wrench_cb(const geometry_msgs::msg::WrenchStamped& msg) {
    last_msg_time_ = this->now();
    watchdog_triggered_ = false;
    Eigen::Vector6d wrench_vector = wrench_to_vector(msg);
    Eigen::VectorXd zero_forces = Eigen::VectorXd::Zero(8);

    if (!healthy_wrench(wrench_vector)) {
        RCLCPP_ERROR(get_logger(), "Wrench vector invalid, publishing zeros.");
        vortex_msgs::msg::ThrusterForces msg_out =
            array_eigen_to_msg(zero_forces);
        thruster_forces_publisher_->publish(msg_out);
        return;
    }

    Eigen::VectorXd thruster_forces =
        pseudoinverse_allocator_.calculate_allocated_thrust(wrench_vector);

    if (is_invalid_matrix(thruster_forces)) {
        RCLCPP_ERROR(get_logger(),
                     "ThrusterForces vector invalid, publishing zeros.");
        vortex_msgs::msg::ThrusterForces msg_out =
            array_eigen_to_msg(zero_forces);
        thruster_forces_publisher_->publish(msg_out);
        return;
    }

    if (!saturate_vector_values(thruster_forces, min_thrust_, max_thrust_)) {
        RCLCPP_WARN(get_logger(), "ThrusterForces vector required saturation.");
    }

    vortex_msgs::msg::ThrusterForces msg_out =
        array_eigen_to_msg(thruster_forces);
    thruster_forces_publisher_->publish(msg_out);
}

void ThrustAllocator::watchdog_callback() {
    auto now = this->now();
    Eigen::VectorXd zero_forces = Eigen::VectorXd::Zero(8);
    if ((now - last_msg_time_) >= timeout_treshold_ && !watchdog_triggered_) {
        watchdog_triggered_ = true;
        RCLCPP_WARN(get_logger(), "Watchdog triggered, publishing zeros.");
        vortex_msgs::msg::ThrusterForces msg_out =
            array_eigen_to_msg(zero_forces);
        thruster_forces_publisher_->publish(msg_out);
    }
}

bool ThrustAllocator::healthy_wrench(const Eigen::VectorXd& v) const {
    if (is_invalid_matrix(v))
        return false;

    bool within_max_thrust = std::none_of(
        v.begin(), v.end(),
        [this](double val) { return std::abs(val) > max_thrust_; });

    return within_max_thrust;
}

RCLCPP_COMPONENTS_REGISTER_NODE(ThrustAllocator)
