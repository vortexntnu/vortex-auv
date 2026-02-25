#include <numbers>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <cmath>
#include <std_msgs/msg/float64_multi_array.hpp>
///#include "velocity_controller/PID_setup.hpp"
#include "velocity_controller/test_VC.hpp"
#include <rmw/types.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include "vortex_msgs/msg/los_guidance.hpp" 
//#include "velocity_controller/velocity_controller.hpp"
//#include "LQR_setup.hpp"
//Denne noden er kun for å teste velocity_controller noden

test_VC::test_VC() : Node("test_VC_node")
{
    this->declare_parameter<std::string>("topics.guidance_topic");
    this->declare_parameter<std::string>("topics.odom_topic");
    this->topic_guidance=this->get_parameter("topics.guidance_topic").as_string();
    this->topic_odometry=this->get_parameter("topics.odom_topic").as_string();
    rclcpp::QoS pub_QoS(10);
    pub_QoS.keep_last(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT).durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  
    publisher_guidance = this->create_publisher<vortex_msgs::msg::LOSGuidance>(topic_guidance, pub_QoS);
    publisher_state = this->create_publisher<vortex_msgs::msg::LOSGuidance>("/state", pub_QoS);
    
    rclcpp::QoS sub_QoS(10);
    sub_QoS.keep_last(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT).durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    subscription_state = this->create_subscription<nav_msgs::msg::Odometry>(
    topic_odometry,sub_QoS,
    std::bind(&test_VC::odometry_callback,this,std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&test_VC::send_guidance, this));
    clock_ = this->get_clock();
    RCLCPP_INFO(this->get_logger(), "Test_VC node has been started");
    

} 

void test_VC::send_guidance()
{
    /*time1+=0.2;
    reference_msg.yaw=0.6*sin(time1*std::numbers::pi/9);
    reference_msg.pitch=0.3*sin(time1*std::numbers::pi/9);*/
    reference_msg.surge=1.0;reference_msg.pitch=0.3;reference_msg.yaw=-1.57; //Surge, pitch, yaw
    RCLCPP_INFO(this->get_logger(), "guidance callback: %f, %f, %f",reference_msg.surge,reference_msg.pitch,reference_msg.yaw);

    publisher_guidance->publish(reference_msg);
}

void test_VC::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg_ptr){
    //RCLCPP_INFO(this->get_logger(), "odo callback");
    vortex_msgs::msg::LOSGuidance msg;
    angle temp=quaternion_to_euler_angle(msg_ptr->pose.pose.orientation.w, msg_ptr->pose.pose.orientation.x, msg_ptr->pose.pose.orientation.y, msg_ptr->pose.pose.orientation.z);
    msg.set__pitch(temp.thetat);
    msg.set__yaw(temp.psit);
    msg.set__surge(msg_ptr->twist.twist.linear.x);
    publisher_state->publish(msg);

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
   