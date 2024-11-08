#include "thruster_interface_auv/thruster_interface_auv_ros.hpp"

ThrusterInterfaceAUVNode::ThrusterInterfaceAUVNode()
    : Node("thruster_interface_auv_node") {
    // extract all .yaml parameters
    this->extract_all_parameters();

    // Set up subscriber and publisher
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    this->thruster_forces_subscriber_ =
        this->create_subscription<vortex_msgs::msg::ThrusterForces>(
            this->subscriber_topic_name_, qos_sensor_data,
            std::bind(&ThrusterInterfaceAUVNode::thruster_forces_callback, this,
                      std::placeholders::_1));

    // publisher only for debugging
    this->thruster_pwm_publisher_ =
        this->create_publisher<std_msgs::msg::Int16MultiArray>(
            this->publisher_topic_name_, qos_sensor_data);

    // call constructor for thruster_driver_
    this->thruster_driver_ = std::make_unique<ThrusterInterfaceAUVDriver>(
        this->i2c_bus_, this->i2c_address_, this->thruster_parameters_,
        this->poly_coeffs_);

    // Declare thruster_forces_array_ in case no topic comes at the
    // very beginning
    this->thruster_forces_array_ = std::vector<double>(8, 0.00);

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(this->thrust_timer_period_),
        std::bind(&ThrusterInterfaceAUVNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
                "\"thruster_interface_auv_node\" correctly initialized");
}

void ThrusterInterfaceAUVNode::thruster_forces_callback(
    const vortex_msgs::msg::ThrusterForces::SharedPtr msg) {
    this->thruster_forces_array_ = msg->thrust;
}

void ThrusterInterfaceAUVNode::timer_callback() {
    // drive thrusters...
    std::vector<int16_t> thruster_pwm_array =
        thruster_driver_->drive_thrusters(this->thruster_forces_array_);

    //..and publish PWM values for debugging purposes
    std_msgs::msg::Int16MultiArray pwm_message;
    pwm_message.data = thruster_pwm_array;
    thruster_pwm_publisher_->publish(pwm_message);
}

void ThrusterInterfaceAUVNode::extract_all_parameters() {
    // thruster parameters from orca.yaml
    this->declare_parameter<std::vector<int>>(
        "propulsion.thrusters.thruster_to_pin_mapping");
    this->declare_parameter<std::vector<int>>(
        "propulsion.thrusters.thruster_direction");
    this->declare_parameter<std::vector<int>>(
        "propulsion.thrusters.thruster_PWM_min");
    this->declare_parameter<std::vector<int>>(
        "propulsion.thrusters.thruster_PWM_max");
    this->declare_parameter<double>("propulsion.thrusters.thrust_update_rate");

    // approx poly coeffs for 16V from thruster_interface_auv.yaml
    this->declare_parameter<std::vector<double>>("coeffs.16V.LEFT");
    this->declare_parameter<std::vector<double>>("coeffs.16V.RIGHT");

    // i2c parameters
    this->declare_parameter<int>("i2c.bus");
    this->declare_parameter<int>("i2c.address");

    // topics
    this->declare_parameter<std::string>("topics.thruster_forces_subscriber");
    this->declare_parameter<std::string>("topics.pwm_publisher");

    //-----------------------------------------------------------------------
    // get them
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
    this->thrust_timer_period_ =
        1.0 / this->get_parameter("propulsion.thrusters.thrust_update_rate")
                  .as_double();

    std::vector<double> left_coeffs =
        this->get_parameter("coeffs.16V.LEFT").as_double_array();
    std::vector<double> right_coeffs =
        this->get_parameter("coeffs.16V.RIGHT").as_double_array();

    this->i2c_bus_ = this->get_parameter("i2c.bus").as_int();
    this->i2c_address_ = this->get_parameter("i2c.address").as_int();

    this->subscriber_topic_name_ =
        this->get_parameter("topics.thruster_forces_subscriber").as_string();
    this->publisher_topic_name_ =
        this->get_parameter("topics.pwm_publisher").as_string();

    // create ThrusterParameters and PolyCoeffs structs
    ThrusterParameters thruster_parameters_struct;
    thruster_parameters_struct.mapping =
        std::vector<uint8_t>(thruster_mapping.begin(), thruster_mapping.end());
    thruster_parameters_struct.direction = std::vector<int8_t>(
        thruster_direction.begin(), thruster_direction.end());
    thruster_parameters_struct.pwm_min =
        std::vector<uint16_t>(thruster_PWM_min.begin(), thruster_PWM_min.end());
    thruster_parameters_struct.pwm_max =
        std::vector<uint16_t>(thruster_PWM_max.begin(), thruster_PWM_max.end());

    PolyCoeffs coeffs_struct;
    coeffs_struct.left =
        std::vector<double>(left_coeffs.begin(), left_coeffs.end());
    coeffs_struct.right =
        std::vector<double>(right_coeffs.begin(), right_coeffs.end());

    this->thruster_parameters_.push_back(thruster_parameters_struct);
    this->poly_coeffs_.push_back(coeffs_struct);
}
