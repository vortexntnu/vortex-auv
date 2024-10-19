#ifndef REFERENCE_FILTER_ROS_HPP
#define REFERENCE_FILTER_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <reference_filter_dp/reference_filter.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vortex_msgs/msg/reference_filter.hpp>
#include <reference_filter_dp/reference_filter_utils.hpp>

class ReferenceFilterNode : public rclcpp::Node
{
    public:
        explicit ReferenceFilterNode();

    private:
        void reference_publisher_callback();

        void reference_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        void set_refererence_filter();

        ReferenceFilter reference_filter_;

        rclcpp::Publisher<vortex_msgs::msg::ReferenceFilter>::SharedPtr reference_pub_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr reference_sub_;

        rclcpp::TimerBase::SharedPtr reference_pub_timer_;

        std::chrono::milliseconds time_step_;

        Vector18d x_;

        Vector6d r_;
};

#endif

