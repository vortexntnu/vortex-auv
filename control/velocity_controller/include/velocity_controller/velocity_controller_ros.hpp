#ifndef VELOCITY_CONTROLLER__VELOCITY_CONTROLLER_HPP_
#define VELOCITY_CONTROLLER__VELOCITY_CONTROLLER_HPP_
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>
#include "velocity_controller/control_manager.hpp"

struct Node_settings{
    //special settings
    bool auto_start;
    bool reset_on_new_ref;
    bool odometry_dropout_guard;

    // Variables for timers
    int publish_rate=100; //ms

    // Variables for topics
    std::string topic_thrust;
    std::string topic_guidance;
    std::string topic_killswitch;
    std::string topic_odometry;
};
class Velocity_node : public rclcpp_lifecycle::LifecycleNode {
   public:
    explicit Velocity_node(const rclcpp::NodeOptions& options);
    Velocity_node(const Velocity_node&) = delete;  // no copy constructor
    Velocity_node& operator=(const Velocity_node&) =delete; // no copy assignment
    Velocity_node(Velocity_node&&) = delete;             // no move constructor
    Velocity_node& operator=(Velocity_node&&) = delete;  // no move assignment

   private:
   /** @brief Gets the new parameters for the node.
        This function retrieves the new parameters for the node from yaml files, such as control type, anti-overshoot, and fallback settings.
    */
    void get_new_parameters();
    /** @brief Initializes the controllers.
        This function initializes the controllers based on the provided parameters, such as control type, anti-overshoot, and fallback settings.
    */
    void initialize_controllers();

    /** @brief Publishes the calculated thrust.
        This function calculates the required thrust based on the current state and guidance data, and publishes the calculated thrust to the appropriate topic.
        Takes no arguments as it uses the current state and guidance data stored in the class.
    */
    void publish_thrust();

    /** @brief Callback function for handling guidance messages.
        This function is called when a new guidance message is received. It updates the guidance values stored in the class and resets the controllers if necessary.
        @param msg_ptr: A shared pointer to the received guidance message.
    */
    void guidance_callback(
        const vortex_msgs::msg::LOSGuidance::SharedPtr msg_ptr);
    /** @brief Callback function for handling odometry messages.
        This function is called when a new odometry message is received. It updates the current state of the vehicle based on the received odometry data.
        @param msg_ptr: A shared pointer to the received odometry message.
    */
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg_ptr);

    // Publisher instance
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr
        publisher_thrust;

    // Timer instance
    rclcpp::TimerBase::SharedPtr timer_calculation;
    rclcpp::TimerBase::SharedPtr startup_timer_;
    // Subscriber instancefriend class VelocityNodeTestAccessor;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        subscriber_Odometry;
    rclcpp::Subscription<vortex_msgs::msg::LOSGuidance>::SharedPtr
        subscriber_guidance;

    
    Node_settings node_settings;
    // Control manager instance
    std::unique_ptr<control_manager> control_manager_ptr;
    // Stored wrenches values
    vortex_msgs::msg::LOSGuidance reference_in;
    geometry_msgs::msg::WrenchStamped thrust_out;
    Guidance_data guidance_values;
    State current_state;
    
    std::atomic_bool should_exit_{false};
    int publish_counter = 0;
    bool first_start = true;

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

    rclcpp::QoS pub_QoS;
    rclcpp::QoS sub_QoS;

    friend class VelocityNodeTestAccessor; //For tests
};
#endif  // VELOCITY_CONTROLLER__VELOCITY_CONTROLLER_HPP_

/*
What I’d improve before merging

Remove unfinished TODOs

README has multiple TODOs.
package.xml has <!--TODO(henrimha): Fix package descritpion and license declaration-->.
This makes the branch look unpolished and reduces confidence.
Fix metadata and package hygiene

Fill in a proper package description and license if this is intended for main.
Check whether the dependency list is complete and accurate.
Ensure every find_package(...) in CMake is reflected in package.xml where relevant, especially ROS packages and custom packages.
Verify build/test on a clean workspace

Run a fresh build from a clean build/install/log state.
Confirm colcon build --packages-select velocity_controller succeeds.
Confirm colcon test --packages-select velocity_controller passes.
If the branch is meant to be on main, it should pass on a fresh environment, not only in your local state.
Add CI

At minimum: colcon build + colcon test.
Optional: clang-format, ament_lint, static checks.
This is especially important for ROS packages.
Polish the README

It is good and detailed, but it should read like project documentation, not draft notes.
Remove placeholders like “TODO” and any internal comments.
Clarify:
which launch file is canonical,
which parameter files are required,
expected topic names,
how to run tests locally.
Add stronger tests

The accessor header is a good start, but I would add:
PID edge-case tests (windup, saturation, zero dt)
LQR fallback tests
lifecycle state transition tests
odometry dropout behavior
invalid parameter handling
Make lifecycle behavior explicit

Your README says the node can run without a lifecycle manager, and also says it is lifecycle managed.
That is acceptable, but document the exact state machine and startup behavior clearly.
Make sure auto_start is tested and reproducible.
Standardize naming

Velocity_node vs VelocityNode is inconsistent.
This is not a blocker, but for maintainability it is worth standardizing in the class/API.
Check for hidden risks

Make sure all custom dependencies are available in the workspace on CI.
Confirm the package compiles in a “new developer” environment, not just yours.
Make sure launch/config files are installed correctly.
Good things already present

The README is detailed and structured.
The package already includes lifecycle support and dependency references.
The CMake is fairly complete.
The tests/accessor pattern is a good idea.
My recommendation

Fix the TODOs and metadata first.
Run a clean build/test.
Add CI and at least a few real tests.
Then merge.
If you want, I can help you do a targeted cleanup pass on:

the README,
package.xml,
and the CMake/test setup before you push.
*/