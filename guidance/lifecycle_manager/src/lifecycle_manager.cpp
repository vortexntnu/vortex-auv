#include "lifecycle_manager/lifecycle_manager.hpp"
#include "lifecycle_manager/lifecycle_service_client.hpp"
#include "rclcpp/rclcpp.hpp"

#define RETRY(fn, retries)                          \
    {                                               \
        int count = 0;                              \
        while (true) {                              \
            try {                                   \
                fn;                                 \
                break;                              \
            } catch (const std::runtime_error& e) { \
                ++count;                            \
                if (count > (retries)) {            \
                    throw e;                        \
                }                                   \
            }                                       \
        }                                           \
    }

void startupLifecycleNode(const std::string& node_name,
                          const std::chrono::seconds service_call_timeout,
                          const int retries = 3) {
    LifecycleServiceClientNode ServiceClient(node_name);

    RETRY(ServiceClient.change_state(
              lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
              service_call_timeout),
          retries)

    RETRY(ServiceClient.change_state(
              lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE,
              service_call_timeout),
          retries)
}

void startup_lifecylce_nodes(const std::vector<std::string>& node_names_vec,
                             const std::chrono::seconds service_call_timeout =
                                 std::chrono::seconds::max(),
                             const int retries = 3) {
    for (const auto& node_name : node_names_vec) {
        startupLifecycleNode(node_name, service_call_timeout, retries);
    }
}

void resetLifecycleNode(const std::string& node_name,
                        const std::chrono::seconds service_call_timeout,
                        const int retries = 3) {
    LifecycleServiceClientNode ServiceClient

        RETRY(sc.change_state(
                  lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
                  service_call_timeout),
              retries);

    RETRY(sc.change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
                          service_call_timeout),
          retries);
}

void reset_lifecylce_nodes(const std::vector<std::string>& node_names_vec,
                           const std::chrono::seconds service_call_timeout =
                               std::chrono::seconds::max(),
                           const int retries = 3);
