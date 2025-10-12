#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <cmath>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "velocity_controller/PID_setup.hpp"
#include "velocity_controller/test_VC.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
//#include "velocity_controller/velocity_controller.hpp"
//#include "LQR_setup.hpp"
//Denne noden er kun for å teste velocity_controller noden

test_VC::test_VC() : Node("test_VC_node"), current_state(0,2,2)
{
    this->declare_parameter<std::string>("topics.guidance_topic");
    this->declare_parameter<std::string>("topics.thrust_topic");
    this->declare_parameter<std::string>("topics.twist_topic");
    this->declare_parameter<std::string>("topics.pose_topic");
    topic_thrust = this->get_parameter("topics.thrust_topic").as_string();
    topic_twist = this->get_parameter("topics.twist_topic").as_string();
    topic_pose = this->get_parameter("topics.pose_topic").as_string();

    topic_guidance = this->get_parameter("topics.guidance_topic").as_string();
    publisher_guidance = this->create_publisher<std_msgs::msg::Float64MultiArray>(topic_guidance, 10);
    publisher_twist = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(topic_twist,10);
    publisher_pose = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(topic_pose,10);
    
    subscription_thrust = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        topic_thrust, 10,
        std::bind(&test_VC::read_thrust, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&test_VC::send_guidance, this));
    clock_ = this->get_clock();
    RCLCPP_INFO(this->get_logger(), "Test_VC node has been started");
    reference_msg.data={2.0, 0.0, 0.0}; //Surge, pitch, yaw
    
} 

void test_VC::send_guidance()
{
    publisher_guidance->publish(reference_msg);
    send_state();
}

void test_VC::read_thrust(geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    current_state.surge += 0.01 * msg->wrench.force.x;
    current_state.pitch += 0.01 * msg->wrench.torque.x;
    current_state.yaw += 0.01 * msg->wrench.torque.y;
    RCLCPP_INFO(this->get_logger(),"info: '%f'", current_state.surge);
    RCLCPP_INFO(this->get_logger(),"info: '%f'", current_state.pitch);
    RCLCPP_INFO(this->get_logger(),"info: '%f'", current_state.yaw);
    return;
}

void test_VC::send_state()
{
    
    twist_msg.header.stamp = clock_->now();
    twist_msg.header.frame_id = "base_link";
    twist_msg.twist.twist.linear.x = current_state.surge;

    pose_msg.header.stamp = clock_->now();
    pose_msg.header.frame_id = "base_link";
    pose_msg.pose.pose.orientation = euler_angle_to_quaternion(0.0, current_state.pitch, current_state.yaw);

    publisher_twist->publish(twist_msg);
    publisher_pose->publish(pose_msg);

    //RCLCPP_INFO(this->get_logger(), "Published state: '%f'", current_state.surge);
    return;
    //RCLCPP_INFO(this->get_logger(), "Published state: '%f'", current_state.pitch);
    //RCLCPP_INFO(this->get_logger(), "Published state: '%f'", current_state.yaw);
}


int main(int argc, char const *argv[])
{
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<test_VC>());
    rclcpp::shutdown();
    return 0;
}

geometry_msgs::msg::Quaternion euler_angle_to_quaternion(double roll, double pitch, double yaw){
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    geometry_msgs::msg::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}