#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <cmath>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "velocity_controller/PID_setup.hpp"
#include "velocity_controller/test_VC.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include "vortex_msgs/msg/los_guidance.hpp" 
//#include "velocity_controller/velocity_controller.hpp"
//#include "LQR_setup.hpp"
//Denne noden er kun for å teste velocity_controller noden

test_VC::test_VC() : Node("test_VC_node"), current_state(0,2,2)
{
    this->declare_parameter<std::string>("topics.guidance_topic");
    this->declare_parameter<std::string>("topics.thrust_topic");
    this->declare_parameter<std::string>("topics.odom_topic");
    topic_thrust = this->get_parameter("topics.thrust_topic").as_string();
    topic_odom = this->get_parameter("topics.odom_topic").as_string();
    topic_guidance = this->get_parameter("topics.guidance_topic").as_string();
    publisher_guidance = this->create_publisher<vortex_msgs::msg::LOSGuidance>(topic_guidance, 10);
    publisher_odom = this->create_publisher<nav_msgs::msg::Odometry>(topic_odom,10);
    
    rclcpp::QoS orca_QoS(2);
    orca_QoS.keep_last(2).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  
    subscription_thrust = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        topic_thrust, orca_QoS,
        std::bind(&test_VC::read_thrust, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&test_VC::send_guidance, this));
    clock_ = this->get_clock();
    RCLCPP_INFO(this->get_logger(), "Test_VC node has been started");
    reference_msg.surge=0.2;reference_msg.pitch=0.3;reference_msg.yaw=0.0; //Surge, pitch, yaw
    
} 

void test_VC::send_guidance()
{
    publisher_guidance->publish(reference_msg);
    //send_state();
}

void test_VC::read_thrust(geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
    /*current_state.surge += 0.01 * msg->wrench.force.x;
    current_state.pitch += 0.01 * msg->wrench.torque.x;
    current_state.yaw += 0.01 * msg->wrench.torque.y;*/
    //RCLCPP_INFO(this->get_logger(),"info: '%f'", current_state.surge);
    //RCLCPP_INFO(this->get_logger(),"info: '%f'", current_state.pitch);
    //RCLCPP_INFO(this->get_logger(),"info: '%f'", current_state.yaw);
    (void) msg;
    return;
}

void test_VC::send_state()
{
    
    odom_msg.header.stamp = clock_->now();
    odom_msg.header.frame_id = "base_link";
    odom_msg.twist.twist.linear.x = current_state.surge;
    odom_msg.pose.pose.orientation = euler_angle_to_quaternion(0.0, current_state.pitch, current_state.yaw);
    odom_msg.twist.twist.linear.y=1;
    odom_msg.twist.twist.linear.z=1;
    odom_msg.twist.twist.angular.x=1;
    odom_msg.twist.twist.angular.y=1;
    odom_msg.twist.twist.angular.z=1;
    odom_msg.twist.twist.linear.y=1;
    odom_msg.twist.twist.linear.z=1;




    publisher_odom->publish(odom_msg);

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