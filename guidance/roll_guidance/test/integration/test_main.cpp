#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>


/**
 * @brief Initialises ROS 2 and executes the integration test suite.
 *
 * Google Test handles test discovery and execution, while ROS 2
 * must be initialised before creating publishers, subscribers,
 * action clients, or action servers.
 *
 * After all tests finish, the ROS 2 context is shut down.
 *
 * @param argc Number of command-line arguments.
 * @param argv Command-line arguments.
 * @return Zero if all tests pass; nonzero otherwise.
 */
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    rclcpp::init(argc, argv);

    const int result = RUN_ALL_TESTS();

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }

    return result;
}