#ifndef JOY_PROCESSOR_ROS
#define JOY_PROCESSOR_ROS

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "joystick_interface_auv/joy_processor_auv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoystickInterfaceNode : public rclcpp::Node {
   public:
    explicit JoystickInterfaceNode();

   private:
    void pose_cb(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

    void joystick_cb(const sensor_msgs::msg::Joy::SharedPtr msg);

    void get_parameters();
    void set_publishers_and_subscribers();

    void transition_to_xbox_mode();
    void transition_to_reference_mode();
    void transition_to_autonomous_mode();

    void handle_killswitch_button();

    JoystickInterface joystick_interface_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _pose_sub_;

    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr wrench_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr ref_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr
        software_killswitch_signal_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr
        operational_mode_signal_pub_;
}
#endif
