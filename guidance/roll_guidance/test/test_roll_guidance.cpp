#include <gtest/gtest.h>

#include <cmath>
#include <Eigen/Geometry>

#include <vortex/utils/math.hpp>
#include <vortex/utils/types.hpp>

#include "roll_guidance/roll_guidance.hpp"

namespace vortex::guidance::test {

class ROLLTests : public ::testing::Test {
   protected:
    ROLLTests() : roll_guidance_{get_roll_params()} {}

    ROLL::Params get_roll_params() {
        ROLL::Params params;
        params.roll_angle = 2.0 * M_PI;
        params.number_of_steps = 8;
        params.trigger_distance = 3.0;
        return params;
    }

    vortex::utils::types::Pose get_initial_pose() {
        vortex::utils::types::Pose pose{};
        pose.set_pos(Eigen::Vector3d::Zero());
        pose.set_ori(Eigen::Quaterniond::Identity());
        return pose;
    }

    RollManeuverGenerator roll_guidance_;

    const double tol = 1e-6;
};


/**
 * @brief Verifies that the ROLL generator produces eight waypoints.
 *
 * Generates a complete 360-degree roll using the default
 * configuration of eight steps.
 *
 * Verifies that:
 * - Exactly eight waypoints are generated.
 */
TEST_F(ROLLTests, T01_test_number_of_waypoints) {
    auto waypoints =
        roll_guidance_.generate_waypoints(get_initial_pose());

    EXPECT_EQ(waypoints.size(), 8u);
}

/**
 * @brief Verifies the orientation of the first roll waypoint.
 *
 * Generates a complete 360-degree roll divided into eight steps.
 *
 * Verifies that:
 * - The first waypoint represents a 45-degree rotation
 *   around the roll axis.
 * - The orientation matches the expected quaternion
 *   within the configured tolerance.
 */
TEST_F(ROLLTests, T02_test_first_waypoint) {
    auto waypoints =
        roll_guidance_.generate_waypoints(get_initial_pose());

    Eigen::Quaterniond expected =
        vortex::utils::math::euler_to_quat(
            M_PI / 4.0, 0.0, 0.0);

    EXPECT_NEAR(
        waypoints[0].ori_quaternion().angularDistance(expected),
        0.0,
        tol
    );
}

/**
 * @brief Verifies the orientation of the second roll waypoint.
 *
 * Generates a complete 360-degree roll divided into eight steps.
 *
 * Verifies that:
 * - The second waypoint represents a 90-degree rotation
 *   around the roll axis.
 * - The orientation matches the expected quaternion
 *   within the configured tolerance.
 */
TEST_F(ROLLTests, T03_test_second_waypoint) {
    auto waypoints =
        roll_guidance_.generate_waypoints(get_initial_pose());

    Eigen::Quaterniond expected =
        vortex::utils::math::euler_to_quat(
            M_PI / 2.0, 0.0, 0.0);

    EXPECT_NEAR(
        waypoints[1].ori_quaternion().angularDistance(expected),
        0.0,
        tol
    );
}

/**
 * @brief Verifies the orientation halfway through the roll.
 *
 * Generates a complete 360-degree roll divided into eight steps.
 *
 * Verifies that:
 * - The fourth waypoint represents a 180-degree rotation
 *   around the roll axis.
 * - The orientation matches the expected quaternion
 *   within the configured tolerance.
 */
TEST_F(ROLLTests, T04_test_half_roll) {
    auto waypoints =
        roll_guidance_.generate_waypoints(get_initial_pose());

    Eigen::Quaterniond expected =
        vortex::utils::math::euler_to_quat(
            M_PI, 0.0, 0.0);

    EXPECT_NEAR(
        waypoints[3].ori_quaternion().angularDistance(expected),
        0.0,
        tol
    );
}

/**
 * @brief Verifies that a complete roll returns to the initial orientation.
 *
 * Generates a complete 360-degree roll from an identity orientation.
 *
 * Verifies that:
 * - The eighth waypoint represents a full rotation.
 * - The final orientation is equivalent to the identity quaternion
 *   within the configured tolerance.
 */
TEST_F(ROLLTests, T05_test_full_roll) {
    auto waypoints =
        roll_guidance_.generate_waypoints(get_initial_pose());

    EXPECT_NEAR(
        waypoints[7].ori_quaternion().angularDistance(
            Eigen::Quaterniond::Identity()),
        0.0,
        tol
    );
}

/**
 * @brief Verifies that roll waypoints preserve the initial position.
 *
 * Configures the initial AUV position to (1.0, 2.0, 3.0)
 * and generates a complete roll maneuver.
 *
 * Verifies that:
 * - Every waypoint retains the initial X-coordinate.
 * - Every waypoint retains the initial Y-coordinate.
 * - Every waypoint retains the initial Z-coordinate.
 *
 * @note The roll generator changes orientation without
 *       introducing translational movement.
 */
TEST_F(ROLLTests, T06_test_position_unchanged) {
    auto initial_pose = get_initial_pose();

    initial_pose.set_pos(
        Eigen::Vector3d(1.0, 2.0, 3.0));

    auto waypoints =
        roll_guidance_.generate_waypoints(initial_pose);

    for (const auto& waypoint : waypoints) {
        EXPECT_NEAR(waypoint.pos_vector().x(), 1.0, tol);
        EXPECT_NEAR(waypoint.pos_vector().y(), 2.0, tol);
        EXPECT_NEAR(waypoint.pos_vector().z(), 3.0, tol);
    }
}

/**
 * @brief Verifies that roll rotations are relative to the initial orientation.
 *
 * Configures a non-identity initial orientation with
 * pitch and yaw offsets before generating the roll waypoints.
 *
 * Verifies that:
 * - The first roll increment is applied relative to the
 *   initial orientation.
 * - The resulting quaternion matches the expected
 *   quaternion multiplication.
 *
 * @note This test ensures that the generator does not
 *       overwrite the initial AUV orientation.
 */
TEST_F(ROLLTests, T07_test_nonidentity_initial_orientation) {
    auto initial_pose = get_initial_pose();

    Eigen::Quaterniond initial_orientation =
        vortex::utils::math::euler_to_quat(
            0.0, 0.2, 0.5);

    initial_pose.set_ori(initial_orientation);

    auto waypoints =
        roll_guidance_.generate_waypoints(initial_pose);

    Eigen::Quaterniond expected =
        initial_orientation *
        vortex::utils::math::euler_to_quat(
            M_PI / 4.0, 0.0, 0.0);

    EXPECT_NEAR(
        waypoints[0].ori_quaternion().angularDistance(expected),
        0.0,
        tol
    );
}


/**
 * @brief Verifies that zero roll steps are rejected.
 *
 * Configures the roll generator with an invalid
 * number of steps equal to zero.
 *
 * Verifies that:
 * - The RollManeuverGenerator constructor throws
 *   std::invalid_argument.
 *
 * @note At least one step is required to generate
 *       a valid roll maneuver.
 */
TEST_F(ROLLTests, T08_test_invalid_step_count) {
    ROLL::Params params;
    params.number_of_steps = 0;

    EXPECT_THROW(
        RollManeuverGenerator generator(params),
        std::invalid_argument
    );
}

    
/**
 * @brief Verifies that a custom number of steps generates
 * the expected number of waypoints.
 */
TEST_F(ROLLTests, T09_test_custom_step_count) {
    ROLL::Params params = get_roll_params();
    params.number_of_steps = 4;

    RollManeuverGenerator generator(params);

    const auto waypoints = generator.generate_waypoints(get_initial_pose());

    ASSERT_EQ(waypoints.size(), 4u);

    const Eigen::Quaterniond expected =
        vortex::utils::math::euler_to_quat(
            M_PI / 2.0, 0.0, 0.0);

    EXPECT_NEAR(
        waypoints[0].ori_quaternion().angularDistance(expected),
        0.0,
        tol);
}

/**
 * @brief Verifies that a single-step roll produces
 * the final orientation.
 */
TEST_F(ROLLTests, T10_test_single_step_roll) {
    ROLL::Params params = get_roll_params();
    params.number_of_steps = 1;

    RollManeuverGenerator generator(params);

    const auto waypoints = generator.generate_waypoints(get_initial_pose());

    ASSERT_EQ(waypoints.size(), 1u);

    EXPECT_NEAR(
        waypoints[0].ori_quaternion().angularDistance(
            Eigen::Quaterniond::Identity()),
        0.0,
        tol);
}

/**
 * @brief Verifies that a partial roll produces
 * the configured final orientation.
 */
TEST_F(ROLLTests, T11_test_partial_roll) {
    ROLL::Params params = get_roll_params();
    params.roll_angle = M_PI;
    params.number_of_steps = 4;

    RollManeuverGenerator generator(params);

    const auto waypoints = generator.generate_waypoints(get_initial_pose());

    ASSERT_EQ(waypoints.size(), 4u);

    const Eigen::Quaterniond expected =
        vortex::utils::math::euler_to_quat(M_PI, 0.0, 0.0);

    EXPECT_NEAR(
        waypoints.back().ori_quaternion().angularDistance(expected),
        0.0,
        tol);
}

/**
 * @brief Verifies that a negative number of steps
 * is rejected by the generator.
 */
TEST_F(ROLLTests, T12_test_negative_step_count) {
    ROLL::Params params = get_roll_params();
    params.number_of_steps = -1;

    EXPECT_THROW(
        RollManeuverGenerator generator(params),
        std::invalid_argument);
}
}  // namespace vortex::guidance::test


/**
 * @brief Initialises ROS 2 and executes the unit test suite.
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
    return RUN_ALL_TESTS();
}