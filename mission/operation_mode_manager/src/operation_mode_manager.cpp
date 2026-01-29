#include "operation_mode_manager.hpp"

OperationModeManager::OperationModeManager(const rclcpp::NodeOptions& options)
    : Node("operation_mode_manager", options) {
    declare_parameters();
    setup_publishers();
    setup_service();
    set_initial_values();

    RCLCPP_INFO(this->get_logger(),
                "Operation Mode Manager Node started. Initial mode: Manual, "
                "Killswitch: true");
}

void OperationModeManager::declare_parameters() {
    this->declare_parameter<std::string>("topics.wrench_input");
    this->declare_parameter<std::string>("topics.killswitch");
    this->declare_parameter<std::string>("topics.operation_mode");
    this->declare_parameter<int>("initial_mode");
    this->declare_parameter<bool>("initial_killswitch");
}

void OperationModeManager::set_initial_values() {
    mode_ = this->get_parameter("initial_mode").as_int();
    killswitch_ = this->get_parameter("initial_killswitch").as_bool();
}

void OperationModeManager::setup_publishers() {
    const auto wrench_input_topic =
        this->get_parameter("topics.wrench_input").as_string();
    const auto killswitch_topic =
        this->get_parameter("topics.killswitch").as_string();
    const auto operation_mode_topic =
        this->get_parameter("topics.operation_mode").as_string();

    wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
        wrench_input_topic,
        vortex::utils::qos_profiles::sensor_data_profile(1));

    killswitch_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        killswitch_topic, vortex::utils::qos_profiles::reliable_profile(1));

    mode_pub_ = this->create_publisher<vortex_msgs::msg::OperationMode>(
        operation_mode_topic, vortex::utils::qos_profiles::reliable_profile(1));

    publish_mode();
}

void OperationModeManager::setup_service() {
    operation_mode_service_ =
        this->create_service<vortex_msgs::srv::SetOperationMode>(
            "set_operation_mode",
            [this](const std::shared_ptr<
                       vortex_msgs::srv::SetOperationMode::Request> request,
                   std::shared_ptr<vortex_msgs::srv::SetOperationMode::Response>
                       response) {
                set_operation_mode_callback(request, response);
            });

    toggle_killswitch_service_ =
        this->create_service<vortex_msgs::srv::ToggleKillswitch>(
            "toggle_killswitch",
            [this](const std::shared_ptr<
                       vortex_msgs::srv::ToggleKillswitch::Request> /*request*/,
                   std::shared_ptr<vortex_msgs::srv::ToggleKillswitch::Response>
                       response) { toggle_killswitch_callback(response); });

    set_killswitch_service_ =
        this->create_service<vortex_msgs::srv::SetKillswitch>(
            "set_killswitch",
            [this](
                const std::shared_ptr<vortex_msgs::srv::SetKillswitch::Request>
                    request,
                std::shared_ptr<vortex_msgs::srv::SetKillswitch::Response>
                    response) { set_killswitch_callback(request, response); });
}

void OperationModeManager::set_operation_mode_callback(
    const std::shared_ptr<vortex_msgs::srv::SetOperationMode::Request> request,
    std::shared_ptr<vortex_msgs::srv::SetOperationMode::Response> response) {
    switch (request->requested_operation_mode.operation_mode) {
        case vortex_msgs::msg::OperationMode::AUTONOMOUS:
            RCLCPP_INFO(this->get_logger(), "Mode set to AUTONOMOUS");
            mode_ = request->requested_operation_mode.operation_mode;
            break;

        case vortex_msgs::msg::OperationMode::MANUAL:
            RCLCPP_INFO(this->get_logger(), "Mode set to MANUAL");
            mode_ = request->requested_operation_mode.operation_mode;
            break;
        case vortex_msgs::msg::OperationMode::REFERENCE:
            RCLCPP_INFO(this->get_logger(), "Mode set to REFERENCE");
            mode_ = request->requested_operation_mode.operation_mode;
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Invalid mode requested");
            break;
    }

    publish_mode();

    response->current_operation_mode.operation_mode = mode_;
    response->killswitch_status = killswitch_;
}

void OperationModeManager::toggle_killswitch_callback(
    std::shared_ptr<vortex_msgs::srv::ToggleKillswitch::Response> response) {
    killswitch_ = !killswitch_;
    RCLCPP_INFO(this->get_logger(), "Killswitch set to %s",
                killswitch_ ? "true" : "false");

    publish_mode();

    response->current_operation_mode.operation_mode = mode_;
    response->killswitch_status = killswitch_;
}

void OperationModeManager::set_killswitch_callback(
    const std::shared_ptr<vortex_msgs::srv::SetKillswitch::Request> request,
    std::shared_ptr<vortex_msgs::srv::SetKillswitch::Response> response) {
    killswitch_ = request->killswitch_on;
    RCLCPP_INFO(this->get_logger(), "Killswitch set to %s",
                killswitch_ ? "true" : "false");

    publish_mode();

    response->current_operation_mode.operation_mode = mode_;
    response->killswitch_status = killswitch_;
}

void OperationModeManager::publish_mode() {
    vortex_msgs::msg::OperationMode mode_msg;
    mode_msg.operation_mode = mode_;
    mode_pub_->publish(mode_msg);

    std_msgs::msg::Bool killswitch_msg;
    killswitch_msg.data = killswitch_;
    killswitch_pub_->publish(killswitch_msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OperationModeManager>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
