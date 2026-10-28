#ifndef TEST_VC_HPP_
#define TEST_VC_HPP_

#include <cmath>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include "nav_msgs/msg/odometry.hpp"
#include "velocity_controller/utilities.hpp"
#include "vortex_msgs/msg/los_guidance.hpp"

/** 
    @brief A class that sends a reference signal to the velocity controller 
*/
class test_velocity_controller : public rclcpp::Node {
    public:
    explicit test_velocity_controller();
    test_velocity_controller(const test_velocity_controller&) = delete;  // no copy constructor
    test_velocity_controller& operator=(const test_velocity_controller&) = delete; // no copy assignment
    test_velocity_controller(test_velocity_controller&&) = delete;             // no move constructor
    test_velocity_controller& operator=(test_velocity_controller&&) = delete;  // no move assignment
    ~test_velocity_controller()=default;
    private:
    /**
     * @brief Publishes a reference signal to the reference topic of the velocity controller.
     */
    void send_reference();
    /** 
        * @brief Subscribes to the odometry topic and prints the current state (in euler angles) of the vehicle for debugging.
    */
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg_ptr);

    // Subscribers and publishers
    rclcpp::Publisher<vortex_msgs::msg::LOSGuidance>::SharedPtr publisher_guidance;
    rclcpp::Publisher<vortex_msgs::msg::LOSGuidance>::SharedPtr publisher_state;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_state;
    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Clock::SharedPtr clock_;
    // Messages
    vortex_msgs::msg::LOSGuidance reference_msg;
    // Topics
    std::string topic_guidance;
    std::string topic_state = "/state";
    std::string topic_odometry;
    
    /**
    * @brief The total time elapsed since the start of the simulation. Used to calculate the reference signal as a function of time.
    */

    double totaltime = 0;
};

#endif  // TEST_VC_HPP_