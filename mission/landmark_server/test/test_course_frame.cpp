#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "landmark_server/course_frame.hpp"
#include "test_map_utils.hpp"

namespace vortex::mission {

using namespace test;
using vortex::utils::types::Pose;

namespace {

Pose start_pose(double x, double y, double yaw) {
    return Pose::from_eigen(
        Eigen::Vector3d(x, y, 0.0),
        Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())));
}

}  // namespace

TEST(CourseFrame, StartsUnsetWithNoBounds) {
    CourseFrameTracker course(example_config().course_frame);
    EXPECT_EQ(course.status(), CourseFrameStatus::UNSET);
    EXPECT_TRUE(course.position_allowed({1000.0, -1000.0, 0.0}));
}

TEST(CourseFrame, SetCoarseGivesTheFrameFromStartPoseAndCoinFlip) {
    CourseFrameTracker course(example_config().course_frame);
    // Vehicle at (2, 3) facing odom +y; the course runs 90 deg to its right
    // (coin flip), i.e. towards odom +x... heading_offset +pi/2 is yaw 180.
    const auto res = course.set_coarse(start_pose(2.0, 3.0, M_PI_2), M_PI_2);
    ASSERT_TRUE(res.success) << res.message;
    EXPECT_EQ(course.status(), CourseFrameStatus::COARSE);
    EXPECT_NEAR(std::abs(course.through_yaw()), M_PI, 1e-12);

    // 5 m through the gate is 5 m towards odom -x (yaw pi).
    const Eigen::Vector2d odom = course.from_course({5.0, 0.0});
    EXPECT_NEAR(odom.x(), -3.0, 1e-9);
    EXPECT_NEAR(odom.y(), 3.0, 1e-9);
}

TEST(CourseFrame, RejectsNanAndIllegalAngles) {
    CourseFrameTracker course(example_config().course_frame);
    const double nan = std::numeric_limits<double>::quiet_NaN();

    EXPECT_FALSE(course.set_coarse(start_pose(nan, 0.0, 0.0), 0.0).success);
    EXPECT_FALSE(course.set_coarse(start_pose(0.0, 0.0, 0.0), nan).success);
    EXPECT_FALSE(course.set_coarse(start_pose(0.0, 0.0, 0.0), 0.4).success);
    EXPECT_FALSE(course.set_coarse(start_pose(0.0, 0.0, 0.0), 1.0).success);
    EXPECT_EQ(course.status(), CourseFrameStatus::UNSET);

    for (const double ok : {0.0, M_PI_2, -M_PI_2, M_PI, -M_PI}) {
        EXPECT_TRUE(course.set_coarse(start_pose(0.0, 0.0, 0.0), ok).success)
            << ok;
    }
}

TEST(CourseFrame, LocksToTheGateAfterConsistentEstimates) {
    CourseFrameTracker course(example_config().course_frame);
    // Start facing odom +x, course straight ahead.
    ASSERT_TRUE(course.set_coarse(start_pose(0, 0, 0), 0.0).success);

    // The gate at (10, 1) faces the start: yaw pi. Through direction = 0.
    for (int i = 0; i < 9; ++i) {
        course.add_gate_estimate({10.0, 1.0}, M_PI + 0.01 * ((i % 3) - 1));
        EXPECT_EQ(course.status(), CourseFrameStatus::COARSE);
        EXPECT_EQ(course.consistent_estimates(), i + 1);
    }
    course.add_gate_estimate({10.0, 1.0}, M_PI);
    EXPECT_EQ(course.status(), CourseFrameStatus::GATE_LOCKED);

    EXPECT_NEAR(course.origin().x(), 10.0, 1e-9);
    EXPECT_NEAR(course.origin().y(), 1.0, 1e-9);
    EXPECT_NEAR(course.through_yaw(), 0.0, 0.02);
    EXPECT_FALSE(course.take_deviation_warning());
    EXPECT_LT(course.start_vs_gate_deviation_deg(), 2.0);

    // Bounds tighten: 8 m to the side of the gate is now outside.
    EXPECT_FALSE(course.position_allowed({10.0, 9.0, 2.0}));
    EXPECT_TRUE(course.position_allowed({15.0, 1.0, 2.0}));
}

TEST(CourseFrame, ALockedFrameMovesWithTheGraphCorrection) {
    CourseFrameTracker course(example_config().course_frame);
    ASSERT_TRUE(course.set_coarse(start_pose(0, 0, 0), 0.0).success);
    for (int i = 0; i < 10; ++i) {
        course.add_gate_estimate({10.0, 1.0}, M_PI);
    }
    ASSERT_EQ(course.status(), CourseFrameStatus::GATE_LOCKED);

    // The graph turns the map 15 deg about the vehicle at (2, 0).
    const double a = 15.0 * M_PI / 180.0;
    Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
    delta.linear() = Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ()).matrix();
    const Eigen::Vector3d pivot(2.0, 0.0, 0.0);
    delta.translation() = pivot - delta.linear() * pivot;
    course.apply_correction(delta);

    const Eigen::Vector3d origin = delta * Eigen::Vector3d(10.0, 1.0, 0.0);
    EXPECT_NEAR(course.origin().x(), origin.x(), 1e-9);
    EXPECT_NEAR(course.origin().y(), origin.y(), 1e-9);
    EXPECT_NEAR(course.through_yaw(), a, 1e-9);
    // 5 m through the gate follows the turned direction.
    const Eigen::Vector2d ahead = course.from_course({5.0, 0.0});
    EXPECT_NEAR((ahead - course.origin()).norm(), 5.0, 1e-9);
    EXPECT_NEAR(std::atan2(ahead.y() - course.origin().y(),
                           ahead.x() - course.origin().x()),
                a, 1e-9);
}

TEST(CourseFrame, UnsetFrameIgnoresTheCorrection) {
    CourseFrameTracker course(example_config().course_frame);
    Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
    delta.translation() = Eigen::Vector3d(3.0, 0.0, 0.0);
    course.apply_correction(delta);
    EXPECT_EQ(course.status(), CourseFrameStatus::UNSET);
    EXPECT_TRUE(course.position_allowed({1000.0, -1000.0, 0.0}));
}

TEST(CourseFrame, InconsistentEstimatesDoNotLock) {
    CourseFrameTracker course(example_config().course_frame);
    ASSERT_TRUE(course.set_coarse(start_pose(0, 0, 0), 0.0).success);
    for (int i = 0; i < 30; ++i) {
        course.add_gate_estimate({10.0, 0.0}, M_PI + (i % 2 ? 0.4 : -0.4));
    }
    EXPECT_EQ(course.status(), CourseFrameStatus::COARSE);
    EXPECT_GT(course.yaw_std(), 0.3);
}

TEST(CourseFrame, WrongStartValueGivesAWarningAndTheGateWins) {
    CourseFrameTracker course(example_config().course_frame);
    // The start value is 40 degrees off the real gate direction.
    const double wrong = 40.0 * M_PI / 180.0;
    ASSERT_TRUE(course.set_coarse(start_pose(0, 0, wrong), 0.0).success);

    for (int i = 0; i < 10; ++i) {
        course.add_gate_estimate({10.0, 0.0}, M_PI);  // real direction: 0
    }
    ASSERT_EQ(course.status(), CourseFrameStatus::GATE_LOCKED);
    EXPECT_NEAR(course.start_vs_gate_deviation_deg(), 40.0, 1e-6);
    EXPECT_TRUE(course.take_deviation_warning());
    EXPECT_FALSE(course.take_deviation_warning());  // only once
    // The gate wins.
    EXPECT_NEAR(course.through_yaw(), 0.0, 1e-9);
}

TEST(CourseFrame, ResetGoesBackToUnset) {
    CourseFrameTracker course(example_config().course_frame);
    ASSERT_TRUE(course.set_coarse(start_pose(0, 0, 0), 0.0).success);
    course.reset();
    EXPECT_EQ(course.status(), CourseFrameStatus::UNSET);
    EXPECT_TRUE(course.position_allowed({500.0, 500.0, 0.0}));
}

TEST(CourseFrame, LaneBoundsAreInTheCourseFrameNotInOdom) {
    CourseFrameTracker course(example_config().course_frame);
    // The vehicle faces odom +y: the course runs along +y, so odom x is the
    // sideways direction. Fixed odom limits would get this wrong.
    ASSERT_TRUE(course.set_coarse(start_pose(0, 0, M_PI_2), 0.0).success);

    EXPECT_TRUE(course.position_allowed({0.0, 40.0, 2.0}));    // far ahead: ok
    EXPECT_FALSE(course.position_allowed({40.0, 0.0, 2.0}));   // far to the side
    EXPECT_FALSE(course.position_allowed({0.0, -20.0, 2.0}));  // far behind
}

}  // namespace vortex::mission
