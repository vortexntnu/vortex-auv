// TODO: Implement reading killswitch and publishing empty wrench

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
using rclcpp::ParameterType::PARAMETER_STRING;
using rclcpp::ParameterType::PARAMETER_DOUBLE;
using rclcpp::ParameterType::PARAMETER_INTEGER;
using rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY;

auto start_message{R"(
  _____ _                    _        _    _ _                 _
 |_   _| |__  _ __ _   _ ___| |_     / \  | | | ___   ___ __ _| |_ ___  _ __
   | | | '_ \| '__| | | / __| __|   / _ \ | | |/ _ \ / __/ _` | __/ _ \| '__|
   | | | | | | |  | |_| \__ \ |_   / ___ \| | | (_) | (_| (_| | || (_) | |
   |_| |_| |_|_|   \__,_|___/\__| /_/   \_\_|_|\___/ \___\__,_|\__\___/|_|

)"};

ThrustAllocator::ThrustAllocator(const rclcpp::NodeOptions& options)
    : Node("thrust_allocator_node", options) {
    extract_parameters();
    set_allocator();
    set_subscriber_and_publisher();

    watchdog_timer_ = this->create_wall_timer(
        500ms, std::bind(&ThrustAllocator::watchdog_callback, this));
    last_msg_time_ = this->now();
    spdlog::info(start_message);
}

void ThrustAllocator::extract_parameters() {
    //params that need further work e.g transform to eigen matrix
    this->declare_parameter(
        "physical.center_of_mass", PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter(
        "propulsion.thrusters.thruster_force_direction", PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter(
        "propulsion.thrusters.thruster_position", PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter(
        "propulsion.thrusters.constraints.input_matrix_weights", PARAMETER_DOUBLE_ARRAY);
    this->declare_parameter(
        "propulsion.thrusters.constraints.slack_matrix_weights", PARAMETER_DOUBLE_ARRAY);

    num_dimensions_ = this->declare_parameter(
        "propulsion.dimensions.num", PARAMETER_INTEGER).get<int>();
    num_thrusters_ = this->declare_parameter(
        "propulsion.thrusters.num", PARAMETER_INTEGER).get<int>();
    min_thrust_ = this->declare_parameter(
        "propulsion.thrusters.constraints.min_force", PARAMETER_DOUBLE).get<double>();
    max_thrust_ = this->declare_parameter(
        "propulsion.thrusters.constraints.max_force", PARAMETER_DOUBLE).get<double>();
    double timout_treshold_param = this->declare_parameter(
        "propulsion.thrusters.watchdog_timeout", PARAMETER_DOUBLE).get<double>();

    center_of_mass_ = double_array_to_eigen_vector3d(
        this->get_parameter("physical.center_of_mass").as_double_array());

    timeout_treshold_ = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::duration<double>(timout_treshold_param));
}

void ThrustAllocator::set_allocator() {
    thruster_force_direction_ = double_array_to_eigen_matrix(
        this->get_parameter("propulsion.thrusters.thruster_force_direction")
            .as_double_array(),
        num_dimensions_, num_thrusters_);
    
    // allocation_config params
    
    std::string allocator_type = this->declare_parameter(
        "propulsion.allocator_type", PARAMETER_STRING).get<std::string>();
    thruster_position_ = double_array_to_eigen_matrix(
        this->get_parameter("propulsion.thrusters.thruster_position")
            .as_double_array(),
        num_dimensions_, num_thrusters_);

    thrust_configuration_ = calculate_thrust_configuration_matrix(
        thruster_force_direction_, thruster_position_, center_of_mass_);

    Eigen::MatrixXd input_weight_matrix = double_array_to_eigen_vector3d(
        this->get_parameter("propulsion.thrusters.constraints.input_matrix_weights")
        .as_double_array()).asDiagonal();

    Eigen::MatrixXd slack_weight_matrix = double_array_to_eigen_vector3d(
        this->get_parameter("propulsion.thrusters.constraints.slack_matrix_weights")
        .as_double_array()).asDiagonal();
    
    Eigen::VectorXd min_force_vec = Eigen::VectorXd::Constant(num_thrusters_, min_thrust_);
    Eigen::VectorXd max_force_vec = Eigen::VectorXd::Constant(num_thrusters_, max_thrust_);
    double beta = this->declare_parameter(
        "propulsion.thrusters.constraints.beta", PARAMETER_DOUBLE).get<double>();

    allocator_ = Factory::make_allocator(
    allocator_type,
    AllocatorConfig{
        .extended_thrust_matrix = thrust_configuration_,
        .min_force              = min_force_vec,
        .max_force              = max_force_vec,
        .input_weight_matrix    = input_weight_matrix,
        .slack_weight_matrix    = slack_weight_matrix,
        .beta                   = beta
        }
    );

    if (!allocator_) {
        throw std::runtime_error("Allocator not initialized (did you call set_allocator()?)");
    }
}

void ThrustAllocator::set_subscriber_and_publisher() {
    auto best_effort_qos = vortex::utils::qos_profiles::sensor_data_profile(1);

    std::string wrench_input_topic = this->declare_parameter(
            "topics.wrench_input", PARAMETER_STRING).get<std::string>();
    std::string thruster_forces_topic = this->declare_parameter(
            "topics.thruster_forces", PARAMETER_STRING).get<std::string>();

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

    Eigen::VectorXd thruster_forces = allocator_->calculate_allocated_thrust(wrench_vector);

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
