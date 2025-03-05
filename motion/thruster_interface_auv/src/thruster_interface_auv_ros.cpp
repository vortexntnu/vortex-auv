#include "thruster_interface_auv/thruster_interface_auv_ros.hpp"
#include <rclcpp_components/register_node_macro.hpp>

ThrusterInterfaceAUVNode::ThrusterInterfaceAUVNode(const rclcpp::NodeOptions& options)
    : Node("thruster_interface_auv_node", options) {
    this->extract_all_parameters();

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    thruster_forces_subscriber_ =
        this->create_subscription<vortex_msgs::msg::ThrusterForces>(
            subscriber_topic_name_, qos_sensor_data,
            std::bind(&ThrusterInterfaceAUVNode::thruster_forces_callback, this,
                      std::placeholders::_1));

    thruster_pwm_publisher_ =
        this->create_publisher<std_msgs::msg::Int16MultiArray>(
            publisher_topic_name_, 1);

    thruster_driver_ = std::make_unique<ThrusterInterfaceAUVDriver>(
        i2c_bus_, i2c_address_, thruster_parameters_, poly_coeffs_);

    thruster_forces_array_ = std::vector<double>(8, 0.00);

    watchdog_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&ThrusterInterfaceAUVNode::watchdog_callback, this));
    last_msg_time_ = this->now();

    this->initialize_parameter_handler();

    RCLCPP_INFO(this->get_logger(),
                "\"thruster_interface_auv_node\" correctly initialized");
}

void ThrusterInterfaceAUVNode::thruster_forces_callback(
    const vortex_msgs::msg::ThrusterForces::SharedPtr msg) {
    thruster_forces_array_ = msg->thrust;
    last_msg_time_ = this->now();
    watchdog_triggered_ = false;

    this->pwm_callback();
}

void ThrusterInterfaceAUVNode::pwm_callback() {
    std::vector<uint16_t> thruster_pwm_array =
        thruster_driver_->drive_thrusters(this->thruster_forces_array_);

    if (debug_flag_) {
        std_msgs::msg::Int16MultiArray pwm_message;
        pwm_message.data = std::vector<int16_t>(thruster_pwm_array.begin(),
                                                thruster_pwm_array.end());
        thruster_pwm_publisher_->publish(pwm_message);
    }
}

void ThrusterInterfaceAUVNode::watchdog_callback() {
    auto now = this->now();
    if ((now - last_msg_time_) >= watchdog_timeout_ && !watchdog_triggered_) {
        thruster_forces_array_.assign(8, 0.00);
        thruster_driver_->drive_thrusters(thruster_forces_array_);
        watchdog_triggered_ = true;
        RCLCPP_WARN(this->get_logger(),
                    "Watchdog triggered, all thrusters set to 0.00");
    }
}

void ThrusterInterfaceAUVNode::initialize_parameter_handler() {
    param_handler_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    debug_flag_parameter_cb = param_handler_->add_parameter_callback(
        "debug.flag", std::bind(&ThrusterInterfaceAUVNode::update_debug_flag,
                                this, std::placeholders::_1));
}

void ThrusterInterfaceAUVNode::update_debug_flag(const rclcpp::Parameter& p) {
    debug_flag_ = p.get_value<bool>();
    RCLCPP_INFO(this->get_logger(),
                "Received parameter event: debug.flag updated to: %s",
                debug_flag_ ? "true" : "false");
}

void ThrusterInterfaceAUVNode::extract_all_parameters() {
    this->declare_parameter<std::vector<int>>(
        "propulsion.thrusters.thruster_to_pin_mapping");
    this->declare_parameter<std::vector<int>>(
        "propulsion.thrusters.thruster_direction");
    this->declare_parameter<std::vector<int>>(
        "propulsion.thrusters.thruster_PWM_min");
    this->declare_parameter<std::vector<int>>(
        "propulsion.thrusters.thruster_PWM_max");

    // approx poly coeffs for 16V from thruster_interface_auv.yaml
    this->declare_parameter<std::vector<double>>("coeffs.16V.LEFT");
    this->declare_parameter<std::vector<double>>("coeffs.16V.RIGHT");

    this->declare_parameter<int>("i2c.bus");
    this->declare_parameter<int>("i2c.address");

    this->declare_parameter<std::string>("topics.thruster_forces");
    this->declare_parameter<std::string>("topics.pwm_output");

    this->declare_parameter<bool>("debug.flag");

    this->declare_parameter<double>("propulsion.thrusters.watchdog_timeout");

    //-----------------------------------------------------------------------

    auto thruster_mapping =
        this->get_parameter("propulsion.thrusters.thruster_to_pin_mapping")
            .as_integer_array();
    auto thruster_direction =
        this->get_parameter("propulsion.thrusters.thruster_direction")
            .as_integer_array();
    auto thruster_PWM_min =
        this->get_parameter("propulsion.thrusters.thruster_PWM_min")
            .as_integer_array();
    auto thruster_PWM_max =
        this->get_parameter("propulsion.thrusters.thruster_PWM_max")
            .as_integer_array();

    std::vector<double> left_coeffs =
        this->get_parameter("coeffs.16V.LEFT").as_double_array();
    std::vector<double> right_coeffs =
        this->get_parameter("coeffs.16V.RIGHT").as_double_array();

    this->i2c_bus_ = this->get_parameter("i2c.bus").as_int();
    this->i2c_address_ = this->get_parameter("i2c.address").as_int();

    this->subscriber_topic_name_ =
        this->get_parameter("topics.thruster_forces").as_string();
    this->publisher_topic_name_ =
        this->get_parameter("topics.pwm_output").as_string();

    this->debug_flag_ = this->get_parameter("debug.flag").as_bool();

    std::transform(thruster_mapping.begin(), thruster_mapping.end(),
                   thruster_direction.begin(),
                   std::back_inserter(this->thruster_parameters_),
                   [&](const int64_t& mapping, const int64_t& direction) {
                       size_t index = &mapping - &thruster_mapping[0];
                       return ThrusterParameters{
                           static_cast<uint8_t>(mapping),
                           static_cast<int8_t>(direction),
                           static_cast<uint16_t>(thruster_PWM_min[index]),
                           static_cast<uint16_t>(thruster_PWM_max[index])};
                   });

    this->poly_coeffs_.push_back(left_coeffs);
    this->poly_coeffs_.push_back(right_coeffs);

    double timout_treshold_param =
        this->get_parameter("propulsion.thrusters.watchdog_timeout")
            .as_double();
    watchdog_timeout_ = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::duration<double>(timout_treshold_param));
}

RCLCPP_COMPONENTS_REGISTER_NODE(ThrusterInterfaceAUVNode)