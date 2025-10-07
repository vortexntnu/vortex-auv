#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <cmath>
#include <std_msgs/msg/float64.hpp>
//#include "LQR_setup.hpp"
//Denne noden er kun for å teste velocity_controller noden

class test_VC : public rclcpp::Node{
    public:
    test_VC() : Node("test_VC_node")
    {
        this->declare_parameter<std::string>("topics.guidance_topic");
        topic_guidance=this->get_parameter("topics.guidance_topic").as_string();
        publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(topic_guidance,10);
        this->declare_parameter<std::string>("topics.thrust_topic");
        topic_subscription = this->get_parameter("topics.thrust_topic").as_string();
        subscription_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            topic_subscription, 10,
            std::bind(&test_VC::listen, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&test_VC::send_velocity, this));
        clock_ = this->get_clock();
        thrust_pub = this->create_publisher<std_msgs::msg::Float64>("thrust_value", 10);

        RCLCPP_INFO(this->get_logger(), "Test_VC node has been started");

        message.wrench.force.x=1;
    }

    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Clock::SharedPtr clock_;
    std::string topic_guidance;
    std::string topic_subscription;
    std::vector<double> thrust_vector;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thrust_pub;
    geometry_msgs::msg::WrenchStamped message;

    void send_velocity()
    {
        message.wrench.force.x = std::sin(clock_->now().seconds()*2*3.14159/10); //sinuskurve med periode 10 sekunder og amplitude 1
        publisher_->publish(message);
    }

    void listen(geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        thrust_vector.push_back(msg->wrench.force.x);
        std_msgs::msg::Float64 pub_info;
        pub_info.data = thrust_vector.back();
        thrust_pub->publish(pub_info);
        message.wrench.force.x+=0.01*msg->wrench.force.x;
        //RCLCPP_INFO(this->get_logger(), "Received thrust: '%f'", msg->wrench.force.x);
        return;
    }

};
int main(int argc, char const *argv[])
{
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<test_VC>());
    rclcpp::shutdown();
    return 0;
}
