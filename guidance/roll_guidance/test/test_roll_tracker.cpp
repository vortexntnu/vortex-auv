#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Geometry>

#include "roll_guidance/roll_tracker.hpp"

namespace vortex::guidance::test {

namespace {

constexpr double kTolerance = 1e-6;

/**
 * @brief Creates a quaternion representing rotation around body X.
 *
 * @param radians Signed rotation angle in radians.
 * @return Unit quaternion representing the rotation.
 */
Eigen::Quaterniond roll_quaternion(double radians) {
    return Eigen::Quaterniond(
        Eigen::AngleAxisd(
            radians,
            Eigen::Vector3d::UnitX()));
}

/**
 * @brief Publishes a simulated sequence of roll measurements
 *        directly to a RollTracker.
 *
 * Each measurement is separated by less than 90 degrees,
 * satisfying the tracker's unwrapping requirement.
 *
 * @param tracker Tracker to update.
 * @param initial Initial orientation of the AUV.
 * @param total_angle Total signed roll in radians.
 * @param steps Number of measurement increments.
 */
void observe_roll_sequence(
    RollTracker& tracker,
    const Eigen::Quaterniond& initial,
    double total_angle,
    int steps = 16) {

    for (int i = 1; i <= steps; ++i) {
        const double angle =
            total_angle * static_cast<double>(i) /
            static_cast<double>(steps);

        tracker.observe(
            initial * roll_quaternion(angle));
    }
}

}  // namespace

/**
 * @brief Verifies that a positive 360-degree roll is tracked.
 *
 * Resets the tracker at the identity orientation and supplies
 * sixteen successive positive roll measurements.
 *
 * @test The accumulated rotation equals positive 2*pi.
 * @test The tracker reports successful completion.
 */
TEST(RollTrackerTests, T28_positive_360_degree_roll) {
    RollTracker tracker;

    tracker.reset(Eigen::Quaterniond::Identity());

    observe_roll_sequence(
        tracker,
        Eigen::Quaterniond::Identity(),
        2.0 * M_PI);

    EXPECT_TRUE(tracker.initialised());

    EXPECT_NEAR(
        tracker.completed_roll(),
        2.0 * M_PI,
        kTolerance);

    EXPECT_TRUE(
        tracker.completed(2.0 * M_PI, kTolerance));
}

/**
 * @brief Verifies that a negative 360-degree roll is tracked.
 *
 * Supplies a full revolution around the negative body X direction.
 *
 * @test Accumulated rotation equals negative 2*pi.
 * @test Positive roll completion is rejected.
 */
TEST(RollTrackerTests, T29_negative_360_degree_roll) {
    RollTracker tracker;

    tracker.reset(Eigen::Quaterniond::Identity());

    observe_roll_sequence(
        tracker,
        Eigen::Quaterniond::Identity(),
        -2.0 * M_PI);

    EXPECT_NEAR(
        tracker.completed_roll(),
        -2.0 * M_PI,
        kTolerance);

    EXPECT_TRUE(
        tracker.completed(-2.0 * M_PI, kTolerance));

    EXPECT_FALSE(
        tracker.completed(2.0 * M_PI, kTolerance));
}

/**
 * @brief Verifies that no rotation does not complete a roll.
 *
 * Resets the tracker and repeatedly observes the same orientation.
 *
 * @test Accumulated rotation remains zero.
 * @test A requested 360-degree roll is not completed.
 */
TEST(RollTrackerTests, T30_no_rotation) {
    RollTracker tracker;

    tracker.reset(Eigen::Quaterniond::Identity());

    for (int i = 0; i < 16; ++i) {
        tracker.observe(
            Eigen::Quaterniond::Identity());
    }

    EXPECT_NEAR(
        tracker.completed_roll(),
        0.0,
        kTolerance);

    EXPECT_FALSE(
        tracker.completed(2.0 * M_PI, 0.05));
}

/**
 * @brief Verifies rejection of invalid quaternions.
 *
 * Tests zero-norm and non-finite quaternion inputs.
 *
 * @test Reset rejects an invalid initial orientation.
 * @test Observe rejects an invalid measured orientation.
 * @test Observe before reset throws a logic error.
 */
TEST(RollTrackerTests, T31_invalid_quaternion) {
    RollTracker tracker;

    const Eigen::Quaterniond zero(0.0, 0.0, 0.0, 0.0);

    const double nan = std::numeric_limits<double>::quiet_NaN();

    const Eigen::Quaterniond invalid(nan, 0.0, 0.0, 0.0);

    EXPECT_THROW(
        tracker.observe(Eigen::Quaterniond::Identity()),
        std::logic_error);

    EXPECT_THROW(
        tracker.reset(zero),
        std::invalid_argument);

    EXPECT_THROW(
        tracker.reset(invalid),
        std::invalid_argument);

    tracker.reset(Eigen::Quaterniond::Identity());

    EXPECT_THROW(
        tracker.observe(zero),
        std::invalid_argument);

    EXPECT_THROW(
        tracker.observe(invalid),
        std::invalid_argument);
}

/**
 * @brief Verifies rejection of ambiguous roll increments.
 *
 * A roll increment of 90 degrees or greater cannot be
 * reliably unwrapped by the current tracker.
 *
 * @test A 120-degree jump is rejected.
 * @test The previous accumulated value remains unchanged.
 */
TEST(RollTrackerTests, T32_large_roll_increment) {
    RollTracker tracker;

    tracker.reset(Eigen::Quaterniond::Identity());

    EXPECT_THROW(
        tracker.observe(roll_quaternion(2.0 * M_PI / 3.0)),
        std::invalid_argument);

    EXPECT_NEAR(
        tracker.completed_roll(),
        0.0,
        kTolerance);
}

/**
 * @brief Verifies tracking from a non-identity orientation.
 *
 * The initial AUV attitude includes pitch and yaw offsets.
 * Roll measurements are applied around the initial body X-axis.
 *
 * @test The tracker accumulates the correct relative rotation.
 */
TEST(RollTrackerTests, T33_nonidentity_initial_orientation) {
    RollTracker tracker;

    const Eigen::Quaterniond initial =
        Eigen::Quaterniond(
            Eigen::AngleAxisd(
                0.5,
                Eigen::Vector3d::UnitZ())) *
        Eigen::Quaterniond(
            Eigen::AngleAxisd(
                0.2,
                Eigen::Vector3d::UnitY()));

    tracker.reset(initial);

    observe_roll_sequence(
        tracker,
        initial,
        2.0 * M_PI);

    EXPECT_NEAR(
        tracker.completed_roll(),
        2.0 * M_PI,
        kTolerance);

    EXPECT_TRUE(
        tracker.completed(2.0 * M_PI, kTolerance));
}

/**
 * @brief Verifies completion tolerance boundaries.
 *
 * Simulates a roll that stops two degrees short of 360 degrees.
 *
 * @test Completion succeeds with a five-degree tolerance.
 * @test Completion fails with a one-degree tolerance.
 */
TEST(RollTrackerTests, T34_completion_tolerance) {
    RollTracker tracker;

    tracker.reset(Eigen::Quaterniond::Identity());

    const double measured_angle =
        358.0 * M_PI / 180.0;

    observe_roll_sequence(
        tracker,
        Eigen::Quaterniond::Identity(),
        measured_angle);

    const double five_degrees = 5.0 * M_PI / 180.0;

    const double one_degree = 1.0 * M_PI / 180.0;

    EXPECT_TRUE(tracker.completed(2.0 * M_PI, five_degrees));

    EXPECT_FALSE(tracker.completed(2.0 * M_PI, one_degree));
}

}  // namespace vortex::guidance::test
