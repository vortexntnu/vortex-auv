#ifndef VELOCITY_CONTROLLER__VELOCITY_CONTROLLER_HPP_
#define VELOCITY_CONTROLLER__VELOCITY_CONTROLLER_HPP_
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
#include "LQR_setup.hpp"
//#include "nav_msgs/msg/odometry.hpp"
#include "velocity_controller/PID_setup.hpp"
//#include "vortex_msgs/msg/los_guidance.hpp"
#include <string>
#endif  // VELOCITY_CONTROLLER__VELOCITY_CONTROLLER_HPP_

class Velocity_node : public rclcpp_lifecycle::LifecycleNode {
   public:
    explicit Velocity_node(const rclcpp::NodeOptions& options);
    Velocity_node(const Velocity_node&) = delete;  // no copy constructor
    Velocity_node& operator=(const Velocity_node&) = delete;  // no copy assignment
    Velocity_node(Velocity_node&&) = delete;  // no move constructor
    Velocity_node& operator=(Velocity_node&&) = delete;  // no move assignment

    private:
    void get_new_parameters();

    // Timer functions
    void calc_thrust();

    // Callback functions
    void guidance_callback(
        const vortex_msgs::msg::LOSGuidance::SharedPtr msg_ptr);
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg_ptr);

    // Publisher instance
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr
        publisher_thrust;

    // Timer instance
    rclcpp::TimerBase::SharedPtr timer_calculation;
    rclcpp::TimerBase::SharedPtr startup_timer_;
    // Subscriber instance
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        subscriber_Odometry;
    rclcpp::Subscription<vortex_msgs::msg::LOSGuidance>::SharedPtr
        subscriber_guidance;

    // Variables for topics
    std::string topic_thrust;
    std::string topic_guidance;
    std::string topic_killswitch;
    std::string topic_odometry;

    // Variables for timers
    int publish_rate;
    double max_force;

    // Stored wrenches values
    vortex_msgs::msg::LOSGuidance reference_in;
    Guidance_data guidance_values;
    State current_state;
    geometry_msgs::msg::WrenchStamped thrust_out;


    // PID controllers
    PID_controller PID_surge;
    std::vector<double> surge_params;
    PID_controller PID_yaw;
    std::vector<double> yaw_params;
    PID_controller PID_pitch;
    std::vector<double> pitch_params;

    // LQR Controller
    LQRController lqr_controller;
    // LQRparameters lqr_parameters;
    std::vector<double> Q;
    std::vector<double> R;
    // std::vector<double> Qi;
    // std::vector<double> Ri;
    std::vector<double> inertia_matrix;
    std::vector<double> dampening_matrix_low;
    std::vector<double> dampening_matrix_high;

    std::atomic_bool should_exit_{false};
    // VC settings
    bool reset_on_new_ref;
    bool anti_overshoot;
    bool auto_start;
    bool odometry_dropout_guard;
    int publish_counter = 0;
    bool first_start = true;
    int controller_type;  // 1 PID, 2 LQR

    // States
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State&) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State& state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State& state) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State&) override;
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State& state) override;

    void reset_controllers(int nr = 0);
    rclcpp::QoS pub_QoS;
    rclcpp::QoS sub_QoS;
};
