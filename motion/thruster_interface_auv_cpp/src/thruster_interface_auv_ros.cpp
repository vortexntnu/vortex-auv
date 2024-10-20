#include "ament_index_cpp/get_package_share_directory.hpp"
#include "thruster_interface_auv_cpp/thruster_interface_auv_ros.hpp"

ThrusterInterfaceAUVNode::ThrusterInterfaceAUVNode() : Node("thruster_interface_auv_node") {
    // Declare parameters
    this->declare_parameter<std::vector<int>>("propulsion.thrusters.thruster_to_pin_mapping", {7,6,5,4,3,2,1,0});
    this->declare_parameter<std::vector<int>>("propulsion.thrusters.thruster_direction", {1,1,1,1,1,1,1,1});
    this->declare_parameter<std::vector<int>>("propulsion.thrusters.thruster_PWM_offset", {0,0,0,0,0,0,0,0});
    this->declare_parameter<std::vector<int>>("propulsion.thrusters.thruster_PWM_min", {1100,1100,1100,1100,1100,1100,1100,1100});
    this->declare_parameter<std::vector<int>>("propulsion.thrusters.thruster_PWM_max", {1900,1900,1900,1900,1900,1900,1900,1900});
    this->declare_parameter<double>("propulsion.thrusters.thrust_update_rate", 10.0);

    // Coefficients parameters
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;

    std::map<int, std::map<std::string, std::vector<double>>> coeffs;
    std::vector<int> voltage_levels = {10, 12, 14, 16, 18, 20};
    for (int voltage : voltage_levels) {
        std::string left_param = "coeffs." + std::to_string(voltage) + "V.LEFT";
        std::string right_param = "coeffs." + std::to_string(voltage) + "V.RIGHT";
        this->declare_parameter(left_param, std::vector<double>{}, descriptor);
        this->declare_parameter(right_param, std::vector<double>{}, descriptor);

        auto left_coeffs = this->get_parameter(left_param).as_double_array();
        auto right_coeffs = this->get_parameter(right_param).as_double_array();

        coeffs[voltage]["LEFT"] = left_coeffs;
        coeffs[voltage]["RIGHT"] = right_coeffs;
    }

    // Get parameters
    auto thruster_mapping = this->get_parameter("propulsion.thrusters.thruster_to_pin_mapping").as_integer_array();
    auto thruster_direction = this->get_parameter("propulsion.thrusters.thruster_direction").as_integer_array();
    auto thruster_PWM_offset = this->get_parameter("propulsion.thrusters.thruster_PWM_offset").as_integer_array();
    auto thruster_PWM_min = this->get_parameter("propulsion.thrusters.thruster_PWM_min").as_integer_array();
    auto thruster_PWM_max = this->get_parameter("propulsion.thrusters.thruster_PWM_max").as_integer_array();
    thrust_timer_period_ = 1.0 / this->get_parameter("propulsion.thrusters.thrust_update_rate").as_double();

    // Initialize thruster driver
    thruster_driver_ = ThrusterInterfaceAUVDriver(
        1, 0x21, 16.0, ament_index_cpp::get_package_share_directory("thruster_interface_auv"),
        std::vector<int>(thruster_mapping.begin(), thruster_mapping.end()),
        std::vector<int>(thruster_direction.begin(), thruster_direction.end()),
        std::vector<int>(thruster_PWM_offset.begin(), thruster_PWM_offset.end()),
        std::vector<int>(thruster_PWM_min.begin(), thruster_PWM_min.end()),
        std::vector<int>(thruster_PWM_max.begin(), thruster_PWM_max.end()),
        coeffs
    );

    // Initialize thruster forces array
    thruster_forces_array_ = std::vector<double>(8, 0.0);

    // Set up subscriber and publisher
    thruster_forces_subscriber_ = this->create_subscription<vortex_msgs::msg::ThrusterForces>(
        "thrust/thruster_forces", 10,
        std::bind(&ThrusterInterfaceAUVNode::thruster_forces_callback, this, std::placeholders::_1)
    );

    thruster_pwm_publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("pwm_APPROX", 10);

    // Set up timer
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(thrust_timer_period_),
        std::bind(&ThrusterInterfaceAUVNode::timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "\"thruster_interface_auv_node\" has been started");
}

void ThrusterInterfaceAUVNode::thruster_forces_callback(const vortex_msgs::msg::ThrusterForces::SharedPtr msg) {
    thruster_forces_array_ = msg->thrust;
}

void ThrusterInterfaceAUVNode::timer_callback() {
    // Send thruster forces to be converted into PWM signals
    auto thruster_pwm_array = thruster_driver_.drive_thrusters(thruster_forces_array_);

    // Publish PWM values for debugging
    std_msgs::msg::Int16MultiArray pwm_message;
    pwm_message.data = thruster_pwm_array;
    thruster_pwm_publisher_->publish(pwm_message);
}
