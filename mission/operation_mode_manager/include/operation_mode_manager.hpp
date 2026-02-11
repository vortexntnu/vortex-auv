#ifndef OPERATION_MODE_MANAGER_HPP_
#define OPERATION_MODE_MANAGER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "vortex/utils/ros/qos_profiles.hpp"

#include "vortex/utils/types.hpp"
#include "vortex_msgs/msg/operation_mode.hpp"
#include "vortex_msgs/srv/set_killswitch.hpp"
#include "vortex_msgs/srv/set_operation_mode.hpp"
#include "vortex_msgs/srv/toggle_killswitch.hpp"

namespace vortex::mission {

class OperationModeManager : public rclcpp::Node {
   public:
    explicit OperationModeManager(const rclcpp::NodeOptions& options);

   private:
    void declare_parameters();
    void setup_publishers();
    void setup_service();
    void set_initial_values();

    void set_operation_mode_callback(
        const std::shared_ptr<vortex_msgs::srv::SetOperationMode::Request>
            request,
        std::shared_ptr<vortex_msgs::srv::SetOperationMode::Response> response);

    void toggle_killswitch_callback(
        std::shared_ptr<vortex_msgs::srv::ToggleKillswitch::Response> response);

    void set_killswitch_callback(
        const std::shared_ptr<vortex_msgs::srv::SetKillswitch::Request> request,
        std::shared_ptr<vortex_msgs::srv::SetKillswitch::Response> response);

    void publish_mode();

    bool killswitch_;
    int mode_;

    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr killswitch_pub_;
    rclcpp::Publisher<vortex_msgs::msg::OperationMode>::SharedPtr mode_pub_;

    rclcpp::Service<vortex_msgs::srv::SetOperationMode>::SharedPtr
        operation_mode_service_;
    rclcpp::Service<vortex_msgs::srv::ToggleKillswitch>::SharedPtr
        toggle_killswitch_service_;
    rclcpp::Service<vortex_msgs::srv::SetKillswitch>::SharedPtr
        set_killswitch_service_;
};

}  // namespace vortex::mission

#endif  // OPERATION_MODE_MANAGER_HPP_
