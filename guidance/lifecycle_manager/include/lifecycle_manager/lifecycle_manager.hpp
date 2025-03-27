#pragma once

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
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

/**
 * @brief Function to start a single of a lifecycle node in the "active" state.
 *
 * This function will call the all necessary transitions of the lifecycle node
 * for the node to begin its normal course of operation.
 *
 * @param node_name The name of the lifecycle node to start.
 * @param service_call_timeout The timeout for the service call.
 * @param retries The number of retries for the service call.
 */
void startupLifecycleNode(const std::string& node_name,
                          const std::chrono::seconds service_call_timeout,
                          const int retries = 3);

/**
 * @brief Function to start a list of a lifecycle nodes in the "active" state.
 *
 * This function will take in a list of node names, and start each of them in
 * the "active" state. It will call the all necessary transitions.
 *
 * @param node_names The names of the lifecycles node to start.
 * @param service_call_timeout The timeout for the service call.
 * @param retries The number of retries for the service call.
 */
void startup_lifecylce_nodes(const std::vector<std::string>& node_names_vec,
                             const std::chrono::seconds service_call_timeout =
                                 std::chrono::seconds::max(),
                             const int retries = 3);

/**
 * @brief Function to stop a single lifecycle node.
 *
 * This function will call the all necessary transitions of the lifecycle node
 * to stop the node and put it in the "inactive" state.
 *
 * @param node_name The name of the lifecycle node to stop.
 * @param service_call_timeout The timeout for the service call.
 * @param retries The number of retries for the service call.
 */
void resetLifecycleNode(const std::string& node_name,
                        const std::chrono::seconds service_call_timeout,
                        const int retries = 3);

/**
 * @brief Function to stop a list of lifecycle nodes.
 *
 * This function will take in a list of node names, and stop each of them.
 * It will call the all necessary transitions.
 *
 * @param node_names The names of the lifecycles node to stop.
 * @param service_call_timeout The timeout for the service call.
 * @param retries The number of retries for the service call.
 */
void reset_lifecylce_nodes(const std::vector<std::string>& node_names_vec,
                           const std::chrono::seconds service_call_timeout =
                               std::chrono::seconds::max(),
                           const int retries = 3);

class LifecycleManagerNode : public rclcpp::Node {
   public:
    /**
     * @brief A constructor for the lifecycle manager
     * @param options Additional options to control creation of the node.
     */
    explicit LifecycleManagerNode(
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : rclcpp::Node("LifecycleManager", options) {}

    /**
     * @brief A destructor for the lifecycle manager
     */
    ~LifecycleManagerNode();

   private:
};
