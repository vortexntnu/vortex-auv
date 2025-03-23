#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using LifecycleCallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

enum NodeState {
    UNCONFIGURED,
    INACTIVE,
    ACTIVE,
    FINALIZED,
    UNKNOWN,
};

class LifecycleManager : public rclcpp::Node {
   public:
    /**
     * @brief A constructor for the lifecycle manager
     * @param options Additional options to control creation of the node.
     */
    explicit LifecycleManager(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : rclcpp::Node("LifecycleManager", options) {}

    /**
     * @brief A destructor for the lifecycle manager
     */
    ~LifecycleManager();

   private:
};
