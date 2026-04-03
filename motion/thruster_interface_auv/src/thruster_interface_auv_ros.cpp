#include "thruster_interface_auv/thruster_interface_auv_ros.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <spdlog/spdlog.h>
#include <vortex/utils/ros/qos_profiles.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <string_view>
#include <vector>

const auto start_message = R"(
  _____ _                    _              ___       _             __
 |_   _| |__  _ __ _   _ ___| |_ ___ _ __  |_ _|_ __ | |_ ___ _ __ / _| __ _  ___ ___
   | | | '_ \| '__| | | / __| __/ _ \ '__|  | || '_ \| __/ _ \ '__| |_ / _` |/ __/ _ \
   | | | | | | |  | |_| \__ \ ||  __/ |     | || | | | ||  __/ |  |  _| (_| | (_|  __/
   |_| |_| |_|_|   \__,_|___/\__\___|_|    |___|_| |_|\__\___|_|  |_|  \__,_|\___\___|

)";




ThrusterInterfaceAUVNode::ThrusterInterfaceAUVNode(
    const rclcpp::NodeOptions& options)
    : Node("thruster_interface_auv_node", options) {
    this->extract_all_parameters();

    auto qos_sensor_data = vortex::utils::qos_profiles::sensor_data_profile(1);

    thruster_forces_subscriber_ =
        this->create_subscription<vortex_msgs::msg::ThrusterForces>(
            subscriber_topic_name_, qos_sensor_data,
            std::bind(&ThrusterInterfaceAUVNode::thruster_forces_callback, this,
                      std::placeholders::_1));

    camera_light_subscriber_ =
        this->create_subscription<std_msgs::msg::Float32>(
            camera_light_topic_name_, qos_sensor_data,
            std::bind(&ThrusterInterfaceAUVNode::camera_light_callback, this,
                      std::placeholders::_1));

    thruster_pwm_publisher_ =
        this->create_publisher<std_msgs::msg::Int16MultiArray>(
            publisher_topic_name_,
            vortex::utils::qos_profiles::reliable_profile(1));

    flt_event_publisher_ =
        this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "thruster_interface_auv/fault_event",
            vortex::utils::qos_profiles::reliable_profile(10));

    pgood_event_publisher_ =
        this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "thruster_interface_auv/pgood_event",
            vortex::utils::qos_profiles::reliable_profile(10));

    killswitch_event_publisher_ =
        this->create_publisher<std_msgs::msg::Bool>(
            "thruster_interface_auv/killswitch_event",
            vortex::utils::qos_profiles::reliable_profile(10));

    current_measurements_publisher_ =
        this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "thruster_interface_auv/current_measurements",
            vortex::utils::qos_profiles::sensor_data_profile(10));

    thruster_driver_ = std::make_unique<ThrusterInterfaceAUVDriver>(
        serial_device_,
        baud_rate_,
        thruster_parameters_,
        right_coeffs_,
        left_coeffs_);

    thruster_driver_->set_fault_event_callback(
        [this](std::uint8_t channel, std::uint8_t code) {
            std_msgs::msg::UInt8MultiArray msg;
            msg.data = {channel, code};
            flt_event_publisher_->publish(msg);
        });

    thruster_driver_->set_pgood_event_callback(
        [this](std::uint8_t channel, std::uint8_t code) {
            std_msgs::msg::UInt8MultiArray msg;
            msg.data = {channel, code};
            pgood_event_publisher_->publish(msg);
        });

    thruster_driver_->set_killswitch_event_callback(
        [this]() {
            std_msgs::msg::Bool msg;
            msg.data = true;
            killswitch_event_publisher_->publish(msg);
        });

    thruster_driver_->set_current_measurements_callback(
        [this](const std::array<float, 8>& currents) {
            std_msgs::msg::Float32MultiArray msg;
            msg.data.assign(currents.begin(), currents.end());
            current_measurements_publisher_->publish(msg);
        });

    if (thruster_driver_->init_uart() != 0) {
        spdlog::error("Failed to initialize UART thruster driver");
    } else {
        spdlog::info("UART thruster driver initialized on {} @ {} baud, packet id 0x{:02X}",
                     serial_device_, baud_rate_);
    }

    thruster_forces_array_ = std::vector<double>(8, 0.0);

    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&ThrusterInterfaceAUVNode::watchdog_callback, this));

    last_msg_time_ = this->now();

    this->initialize_parameter_handler();

    spdlog::info(start_message);
}

void ThrusterInterfaceAUVNode::camera_light_callback(
    const std_msgs::msg::Float32::SharedPtr msg) {
    const float intensity = std::clamp(msg->data, 0.0f, 1.0f);

    if (thruster_driver_->set_camera_light(intensity) != 0) {
        spdlog::warn("Failed to set camera light intensity to {}", intensity);
        return;
    }

    spdlog::debug("Camera light intensity set to {}", intensity);
}

void ThrusterInterfaceAUVNode::thruster_forces_callback(
    const vortex_msgs::msg::ThrusterForces::SharedPtr msg) {
    thruster_forces_array_ = msg->thrust;
    last_msg_time_ = this->now();
    watchdog_triggered_ = false;

    this->pwm_callback();
}


void ThrusterInterfaceAUVNode::pwm_callback() {
    auto thruster_pwm_array_opt =
        thruster_driver_->drive_thrusters(thruster_forces_array_);

    if (!thruster_pwm_array_opt.has_value()) {
        spdlog::warn("Sending PWM values to thrusters failed");
        return;
    }

    const auto& thruster_pwm_array = thruster_pwm_array_opt.value();

    if (debug_flag_) {
        std_msgs::msg::Int16MultiArray pwm_message;
        pwm_message.data = std::vector<std::int16_t>(
            thruster_pwm_array.begin(), thruster_pwm_array.end());

        thruster_pwm_publisher_->publish(pwm_message);
    }
}


void ThrusterInterfaceAUVNode::watchdog_callback() {
    const auto now = this->now();

    if ((now - last_msg_time_) >= watchdog_timeout_ && !watchdog_triggered_) {
        thruster_forces_array_.assign(8, 0.0);

        if (!thruster_driver_->drive_thrusters(thruster_forces_array_).has_value()) {
            spdlog::warn("Watchdog triggered, but failed to send zero command to thrusters");
        } else {
            spdlog::warn("Watchdog triggered, all thrusters set to 0.0");
        }

        watchdog_triggered_ = true;
    }
}

void ThrusterInterfaceAUVNode::initialize_parameter_handler() {
    param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    debug_flag_parameter_cb = param_handler_->add_parameter_callback(
        "debug.flag",
        std::bind(&ThrusterInterfaceAUVNode::update_debug_flag, this,
                  std::placeholders::_1));
}

void ThrusterInterfaceAUVNode::update_debug_flag(const rclcpp::Parameter& p) {
    debug_flag_ = p.get_value<bool>();

    spdlog::info("Received parameter event: debug.flag updated to: {}",
                 debug_flag_ ? "true" : "false");
}

void ThrusterInterfaceAUVNode::extract_all_parameters() {
    this->declare_parameter<std::vector<int64_t>>(
        "propulsion.thrusters.thruster_to_pin_mapping");
    this->declare_parameter<std::vector<int64_t>>(
        "propulsion.thrusters.thruster_direction");
    this->declare_parameter<std::vector<int64_t>>(
        "propulsion.thrusters.thruster_PWM_min");
    this->declare_parameter<std::vector<int64_t>>(
        "propulsion.thrusters.thruster_PWM_max");

    // Approx poly coeffs for 16V from thruster_interface_auv.yaml
    this->declare_parameter<std::vector<double>>("coeffs.16V.LEFT");
    this->declare_parameter<std::vector<double>>("coeffs.16V.RIGHT");

    this->declare_parameter<std::string>("uart.device");
    this->declare_parameter<int>("uart.baud_rate");

    this->declare_parameter<std::string>("topics.thruster_forces");
    this->declare_parameter<std::string>("topics.pwm_output");
    this->declare_parameter<std::string>("topics.camera_light");

    this->declare_parameter<bool>("debug.flag");
    this->declare_parameter<double>("propulsion.thrusters.watchdog_timeout");

    const auto thruster_mapping =
        this->get_parameter("propulsion.thrusters.thruster_to_pin_mapping")
            .as_integer_array();
    const auto thruster_direction =
        this->get_parameter("propulsion.thrusters.thruster_direction")
            .as_integer_array();
    const auto thruster_pwm_min =
        this->get_parameter("propulsion.thrusters.thruster_PWM_min")
            .as_integer_array();
    const auto thruster_pwm_max =
        this->get_parameter("propulsion.thrusters.thruster_PWM_max")
            .as_integer_array();

    left_coeffs_ = this->get_parameter("coeffs.16V.LEFT").as_double_array();
    right_coeffs_ = this->get_parameter("coeffs.16V.RIGHT").as_double_array();

    serial_device_ = this->get_parameter("uart.device").as_string();
    baud_rate_ = static_cast<unsigned int>(
        this->get_parameter("uart.baud_rate").as_int());

    subscriber_topic_name_ =
        this->get_parameter("topics.thruster_forces").as_string();
    publisher_topic_name_ =
        this->get_parameter("topics.pwm_output").as_string();
    camera_light_topic_name_ =
        this->get_parameter("topics.camera_light").as_string();

    debug_flag_ = this->get_parameter("debug.flag").as_bool();

    const auto thruster_count = thruster_mapping.size();

    if (thruster_direction.size() != thruster_count ||
        thruster_pwm_min.size() != thruster_count ||
        thruster_pwm_max.size() != thruster_count) {
        throw std::runtime_error(
            "Thruster parameter arrays must all have the same length");
    }

    if (thruster_count != 8) {
        spdlog::warn(
            "UART packet format expects 8 thrusters, but config contains {} entries",
            thruster_count);
    }

    thruster_parameters_.clear();
    thruster_parameters_.reserve(thruster_count);

    for (std::size_t i = 0; i < thruster_count; ++i) {
        thruster_parameters_.push_back(ThrusterParameters{
            static_cast<std::uint8_t>(thruster_mapping[i]),
            static_cast<std::int8_t>(thruster_direction[i]),
            static_cast<std::uint16_t>(thruster_pwm_min[i]),
            static_cast<std::uint16_t>(thruster_pwm_max[i]),
        });
    }

    const double timeout_threshold_param =
        this->get_parameter("propulsion.thrusters.watchdog_timeout").as_double();

    watchdog_timeout_ = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::duration<double>(timeout_threshold_param));
}

RCLCPP_COMPONENTS_REGISTER_NODE(ThrusterInterfaceAUVNode)
