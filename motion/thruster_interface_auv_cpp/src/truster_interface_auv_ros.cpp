#include "thruster_interface_auv_cpp/thruster_interface_auv_ros.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

ThrusterInterfaceAUVNode::ThrusterInterfaceAUVNode() 
    : Node("thruster_interface_auv_cpp_node"), 
      thruster_forces_array_(8, 0.0) {
    // Subscriber and Publisher setup
    thruster_forces_subscriber_ = this->create_subscription<vortex_msgs::msg::ThrusterForces>(
        "thrust/thruster_forces", 10, std::bind(&ThrusterInterfaceAUVNode::thruster_forces_callback, this, std::placeholders::_1));
    
    thruster_pwm_publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("pwm_APPROX", 10);

    // Get parameters
    this->declare_parameter<std::vector<int>>("propulsion.thrusters.thruster_to_pin_mapping", std::vector<int>{7, 6, 5, 4, 3, 2, 1, 0});
    this->declare_parameter<std::vector<int>>("propulsion.thrusters.thruster_direction", std::vector<int>{1, 1, 1, 1, 1, 1, 1, 1});
    this->declare_parameter<std::vector<int>>("propulsion.thrusters.thruster_PWM_offset", std::vector<int>{0, 0, 0, 0, 0, 0, 0, 0});
    this->declare_parameter<std::vector<int>>("propulsion.thrusters.thruster_PWM_min", std::vector<int>{1100, 1100, 1100, 1100, 1100, 1100, 1100, 1100});
    this->declare_parameter<std::vector<int>>("propulsion.thrusters.thruster_PWM_max", std::vector<int>{1900, 1900, 1900, 1900, 1900, 1900, 1900, 1900});
    this->declare_parameter<double>("propulsion.thrusters.thrust_update_rate", 10.0);

    this->get_parameter("propulsion.thrusters.thruster_to_pin_mapping", thruster_mapping_);
    this->get_parameter("propulsion.thrusters.thruster_direction", thruster_direction_);
    this->get_parameter("propulsion.thrusters.thruster_PWM_offset", thruster_pwm_offset_);
    this->get_parameter("propulsion.thrusters.thruster_PWM_min", pwm_min_);
    this->get_parameter("propulsion.thrusters.thruster_PWM_max", pwm_max_);
    this->get_parameter("propulsion.thrusters.thrust_update_rate", thrust_timer_period_);
    thrust_timer_period_ = 1.0 / thrust_timer_period_;

    // Get the coefficients for polynomial interpolation of thruster forces
    std::map<int, std::map<std::string, Eigen::VectorXd>> coeffs;
    std::vector<int> voltages = {10, 12, 14, 16, 18, 20};
    for (int voltage : voltages) {
        coeffs[voltage]["LEFT"] = Eigen::VectorXd::Map(this->declare_parameter<std::vector<double>>("coeffs." + std::to_string(voltage) + "V.LEFT", {}).data(), 6);
        coeffs[voltage]["RIGHT"] = Eigen::VectorXd::Map(this->declare_parameter<std::vector<double>>("coeffs." + std::to_string(voltage) + "V.RIGHT", {}).data(), 6);
    }

    // Initialize the thruster driver
    thruster_driver_ = ThrusterInterfaceAUVDriver(
        ament_index_cpp::get_package_share_directory("thruster_interface_auv"),
        thruster_mapping_, thruster_direction_, thruster_pwm_offset_, pwm_min_, pwm_max_, coeffs
    );

    // Timer for periodic thruster control updates
    timer_ = this->create_wall_timer(std::chrono::duration<double>(thrust_timer_period_), std::bind(&ThrusterInterfaceAUVNode::timer_callback, this));

    // Debugging
    RCLCPP_INFO(this->get_logger(), "Thruster Interface AUV Node has been started.");
}

void ThrusterInterfaceAUVNode::thruster_forces_callback(const vortex_msgs::msg::ThrusterForces::SharedPtr msg) {
    thruster_forces_array_ = msg->thrust;
}

void ThrusterInterfaceAUVNode::timer_callback() {
    // Send forces to be converted into PWM and published
    std::vector<int> thruster_pwm_array = thruster_driver_.drive_thrusters(thruster_forces_array_);

    // Convert std::vector<int> to std::vector<short int>
    std::vector<short int> thruster_pwm_array_short(thruster_pwm_array.begin(), thruster_pwm_array.end());

    // Publish PWM values for debugging
    std_msgs::msg::Int16MultiArray pwm_message;
    pwm_message.data = thruster_pwm_array_short;
    thruster_pwm_publisher_->publish(pwm_message);
}