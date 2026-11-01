#include <thread>
#include "roll_guidance/roll_guidance_ros.hpp"

/**
 * @brief Entry point for the ROLL guidance ROS 2 node.
 *
 * Initialises ROS 2, creates the ROLL guidance node, and
 * executes its callbacks using a multithreaded executor.
 *
 * The node requests ROS 2 shutdown when either:
 * - The roll maneuver finishes, regardless of its result.
 * - The AUV passes the gate.
 *
 * Once shutdown is requested, the executor stops spinning
 * and the application exits.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return 0 when the application exits normally.
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<vortex::guidance::ROLLGuidanceNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    executor.spin();

    return 0;
}