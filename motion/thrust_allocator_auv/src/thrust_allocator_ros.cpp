#include "thrust_allocator_auv/thrust_allocator_ros.hpp"
#include <spdlog/spdlog.h>
#include <chrono>
#include <functional>
#include <rclcpp_components/register_node_macro.hpp>
#include <string_view>
#include <vortex/utils/qos_profiles.hpp>
#include "vortex/utils/types.hpp"
#include "thrust_allocator_auv/pseudoinverse_allocator.hpp"
#include "thrust_allocator_auv/thrust_allocator_utils.hpp"

using namespace std::chrono_literals;
using vortex::utils::types::Vector6d;

auto start_message{R"(
  _____ _                    _        _    _ _                 _
 |_   _| |__  _ __ _   _ ___| |_     / \  | | | ___   ___ __ _| |_ ___  _ __
   | | | '_ \| '__| | | / __| __|   / _ \ | | |/ _ \ / __/ _` | __/ _ \| '__|
   | | | | | | |  | |_| \__ \ |_   / ___ \| | | (_) | (_| (_| | || (_) | |
   |_| |_| |_|_|   \__,_|___/\__| /_/   \_\_|_|\___/ \___\__,_|\__\___/|_|

)"};

ThrustAllocator::ThrustAllocator(const rclcpp::NodeOptions& options)
    : Node("thrust_allocator_node", options),
      pseudoinverse_allocator_(Eigen::MatrixXd::Zero(6, 8)) {
    extract_parameters();
    set_allocator();
    set_subscriber_and_publisher();

    watchdog_timer_ = this->create_wall_timer(
        500ms, std::bind(&ThrustAllocator::watchdog_callback, this));
    last_msg_time_ = this->now();
    spdlog::info(start_message);
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
    this->declare_parameter<std::string>("propulsion.type");
    this->declare_parameter<std::string>("topics.wrench_input");
    this->declare_parameter<std::string>("topics.thruster_forces");

    center_of_mass_ = double_array_to_eigen_vector3d(
        this->get_parameter("physical.center_of_mass").as_double_array());
    num_dimensions_ = this->get_parameter("propulsion.dimensions.num").as_int();
    num_thrusters_ = this->get_parameter("propulsion.thrusters.num").as_int();
    min_thrust_ = this->get_parameter("propulsion.thrusters.min").as_int();
    max_thrust_ = this->get_parameter("propulsion.thrusters.max").as_int();
    solver_type_ = this->get_parameter("propulsion.type").as_string();

    double timout_treshold_param =
        this->get_parameter("propulsion.thrusters.watchdog_timeout")
            .as_double();
    timeout_treshold_ = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::duration<double>(timout_treshold_param));

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
    
    if solver_type_ == "pseudo_inverse" {
        pseudoinverse_allocator_.T_pinv =
            calculate_pseudoinverse(thrust_configuration_);
    }

    if solver_type_ == "QP" {
        // TODO: implement solver lol
    }

}

void ThrustAllocator::set_subscriber_and_publisher() {
    auto best_effort_qos = vortex::utils::qos_profiles::sensor_data_profile(1);

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
    Vector6d wrench_vector = wrench_to_vector(msg);
    Eigen::VectorXd zero_forces = Eigen::VectorXd::Zero(8);

    if (!healthy_wrench(wrench_vector)) {
        spdlog::error("Wrench vector invalid, publishing zeros.");
        vortex_msgs::msg::ThrusterForces msg_out =
            array_eigen_to_msg(zero_forces);
        thruster_forces_publisher_->publish(msg_out);
        return;
    }

    Eigen::VectorXd thruster_forces =
        pseudoinverse_allocator_.calculate_allocated_thrust(wrench_vector);

    if (is_invalid_matrix(thruster_forces)) {
        spdlog::error("ThrusterForces vector invalid, publishing zeros.");
        vortex_msgs::msg::ThrusterForces msg_out =
            array_eigen_to_msg(zero_forces);
        thruster_forces_publisher_->publish(msg_out);
        return;
    }

    if (!saturate_vector_values(thruster_forces, min_thrust_, max_thrust_)) {
        spdlog::warn("ThrusterForces vector required saturation.");
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
        spdlog::warn("Watchdog triggered, publishing zeros.");
        vortex_msgs::msg::ThrusterForces msg_out =
            array_eigen_to_msg(zero_forces);
        thruster_forces_publisher_->publish(msg_out);
    }
}

bool ThrustAllocator::healthy_wrench(const Eigen::VectorXd& v) const {
    if (is_invalid_matrix(v))
        return false;

    bool within_max_thrust = std::ranges::none_of(
        v, [this](double val) { return std::abs(val) > max_thrust_; });

    return within_max_thrust;
}

RCLCPP_COMPONENTS_REGISTER_NODE(ThrustAllocator)
