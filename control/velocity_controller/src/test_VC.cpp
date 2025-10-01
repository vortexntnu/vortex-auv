#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <cmath>
//Denne noden er kun for å teste velocity_controller noden

class test_VC : public rclcpp::Node{
    public:
    test_VC() : Node("test_VC_node")
    {
        this->declare_parameter<std::string>("topics.guidance_topic");
        topic_guidance=this->get_parameter("topics.guidance_topic").as_string();
        publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(topic_guidance,10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&test_VC::send_velocity, this));
        clock_ = this->get_clock();
        RCLCPP_INFO(this->get_logger(), "Test_VC node has been started");
    }

    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Clock::SharedPtr clock_;
    std::string topic_guidance;
    void send_velocity()
    {
        auto message = geometry_msgs::msg::WrenchStamped();
        message.wrench.force.x = std::sin(clock_->now().seconds());
        message.wrench.force.y = 0.0;
        message.wrench.force.z = 0.0;
        message.wrench.torque.x = 0.0;
        message.wrench.torque.y = 0.0;
        message.wrench.torque.z = 0.0;
        publisher_->publish(message);
    }
};
int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<test_VC>());
    rclcpp::shutdown();
    return 0;
}
