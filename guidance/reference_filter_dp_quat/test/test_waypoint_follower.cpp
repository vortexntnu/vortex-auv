#include <gtest/gtest.h>
#include <vortex/utils/waypoint_utils.hpp>
#include "reference_filter_dp_quat/lib/waypoint_follower.hpp"

namespace vortex::guidance {

class WaypointFollowerTests : public ::testing::Test {
   protected:
    ReferenceFilterParams get_params() {
        ReferenceFilterParams params;
        params.omega = Eigen::Vector6d::Ones();
        params.zeta = Eigen::Vector6d::Ones();
        return params;
    }

    Pose zero_pose() { return Pose{}; }
    Twist zero_twist() { return Twist{}; }
};

TEST_F(WaypointFollowerTests, StartAndStepConverges) {
    WaypointFollower follower(get_params(), 0.01);

    Waypoint wp;
    wp.pose = Pose{1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    wp.mode = WaypointMode::FULL_POSE;

    follower.start(zero_pose(), zero_twist(), wp, 0.1);

    follower.step();

    // Simulate the measured pose being at the reference
    Pose measured_at_ref{1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

    EXPECT_TRUE(follower.within_convergance(measured_at_ref));
}

TEST_F(WaypointFollowerTests, StepDoesNotConvergeWhenFar) {
    WaypointFollower follower(get_params(), 0.01);

    Waypoint wp;
    wp.pose = Pose{10.0, 10.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    wp.mode = WaypointMode::FULL_POSE;

    follower.start(zero_pose(), zero_twist(), wp, 0.1);

    follower.step();

    EXPECT_FALSE(follower.within_convergance(zero_pose()));
}

TEST_F(WaypointFollowerTests, SetReferenceUpdatesMidSequence) {
    WaypointFollower follower(get_params(), 0.01);

    Waypoint wp;
    wp.pose = Pose{1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    wp.mode = WaypointMode::FULL_POSE;

    follower.start(zero_pose(), zero_twist(), wp, 0.1);

    Pose new_ref{5.0, 5.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    follower.set_reference(new_ref);

    EXPECT_DOUBLE_EQ(follower.waypoint_goal().x, 5.0);
    EXPECT_DOUBLE_EQ(follower.waypoint_goal().y, 5.0);
}

TEST_F(WaypointFollowerTests, SnapStateToReference) {
    WaypointFollower follower(get_params(), 0.01);

    Waypoint wp;
    wp.pose = Pose{3.0, 4.0, 5.0, 1.0, 0.0, 0.0, 0.0};
    wp.mode = WaypointMode::FULL_POSE;

    follower.start(zero_pose(), zero_twist(), wp, 0.1);
    follower.snap_state_to_reference();

    Pose pose = follower.pose();
    Pose goal = follower.waypoint_goal();

    EXPECT_DOUBLE_EQ(pose.x, goal.x);
    EXPECT_DOUBLE_EQ(pose.y, goal.y);
    EXPECT_DOUBLE_EQ(pose.z, goal.z);
    EXPECT_DOUBLE_EQ(pose.qw, goal.qw);
    EXPECT_DOUBLE_EQ(pose.qx, goal.qx);
    EXPECT_DOUBLE_EQ(pose.qy, goal.qy);
    EXPECT_DOUBLE_EQ(pose.qz, goal.qz);
}

TEST_F(WaypointFollowerTests, StateEvolvesWithStep) {
    WaypointFollower follower(get_params(), 0.01);

    Waypoint wp;
    wp.pose = Pose{1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    wp.mode = WaypointMode::FULL_POSE;

    follower.start(zero_pose(), zero_twist(), wp, 0.1);

    // Run several steps — state should move toward reference
    for (int i = 0; i < 100; ++i) {
        follower.step();
    }

    // x position should have moved toward 1.0
    EXPECT_GT(follower.pose().x, 0.0);
}

TEST_F(WaypointFollowerTests, SeparateTolerancesReplaceCombinedThreshold) {
    WaypointFollower follower(get_params(), 0.01);

    Waypoint wp;
    wp.pose = Pose{1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    wp.mode = WaypointMode::FULL_POSE;
    wp.position_tolerance = 0.05;
    wp.orientation_tolerance = 0.5;

    // The combined threshold of 1.0 alone would accept a 0.1 m error.
    follower.start(zero_pose(), zero_twist(), wp, 1.0);

    EXPECT_FALSE(follower.within_convergance(Pose{1.1, 0.0, 0.0, 1, 0, 0, 0}));
    EXPECT_TRUE(follower.within_convergance(Pose{1.04, 0.0, 0.0, 1, 0, 0, 0}));
}

TEST_F(WaypointFollowerTests, UnsetToleranceFallsBackToThreshold) {
    WaypointFollower follower(get_params(), 0.01);

    Waypoint wp;
    wp.pose = Pose{1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    wp.mode = WaypointMode::FULL_POSE;
    wp.position_tolerance = 0.5;  // orientation_tolerance unset -> threshold

    follower.start(zero_pose(), zero_twist(), wp, 0.1);

    const Eigen::Quaterniond yawed(
        Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
    EXPECT_FALSE(follower.within_convergance(
        Pose::from_eigen(Eigen::Vector3d(1.0, 0.0, 0.0), yawed)));
    EXPECT_TRUE(follower.within_convergance(Pose{1.4, 0.0, 0.0, 1, 0, 0, 0}));
}

TEST_F(WaypointFollowerTests, HoldNeedsContinuousTimeInside) {
    WaypointFollower follower(get_params(), 0.01);

    Waypoint wp;
    wp.pose = Pose{1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    wp.mode = WaypointMode::FULL_POSE;
    follower.start(zero_pose(), zero_twist(), wp, 0.1);

    const Pose at_goal{1.0, 0.0, 0.0, 1, 0, 0, 0};
    const Pose outside{2.0, 0.0, 0.0, 1, 0, 0, 0};
    const double hold = 1.0;

    // Inside for 0.9 s: not enough.
    for (int i = 0; i <= 90; ++i) {
        EXPECT_FALSE(follower.update_convergence(at_goal, i * 0.01, false, hold));
    }
    // Leaving the tolerance restarts the hold.
    EXPECT_FALSE(follower.update_convergence(outside, 0.95, false, hold));
    // Inside again from t = 1.0: succeeds after 1.0 s inside.
    for (int i = 100; i < 200; ++i) {
        EXPECT_FALSE(follower.update_convergence(at_goal, i * 0.01, false, hold));
    }
    EXPECT_TRUE(follower.update_convergence(at_goal, 2.0, false, hold));
}

TEST_F(WaypointFollowerTests, ZeroHoldConvergesImmediately) {
    WaypointFollower follower(get_params(), 0.01);

    Waypoint wp;
    wp.pose = Pose{1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    wp.mode = WaypointMode::FULL_POSE;
    follower.start(zero_pose(), zero_twist(), wp, 0.1);

    EXPECT_TRUE(follower.update_convergence(Pose{1.0, 0.0, 0.0, 1, 0, 0, 0},
                                            123.0, false, 0.0));
    EXPECT_FALSE(follower.update_convergence(Pose{3.0, 0.0, 0.0, 1, 0, 0, 0},
                                             123.01, false, 0.0));
}

TEST_F(WaypointFollowerTests, HoldNotSatisfiedWhenPassingThroughTarget) {
    WaypointFollower follower(get_params(), 0.01);

    Waypoint wp;
    wp.pose = Pose{1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    wp.mode = WaypointMode::FULL_POSE;
    follower.start(zero_pose(), zero_twist(), wp, 0.1);

    // Vehicle moves through the goal at 1 m/s: inside the 0.1 m tolerance
    // for only ~0.2 s, far less than the 2 s hold.
    for (int i = 0; i <= 200; ++i) {
        const double t = i * 0.01;
        EXPECT_FALSE(follower.update_convergence(
            Pose{t, 0.0, 0.0, 1, 0, 0, 0}, t, false, 2.0))
            << "t = " << t;
    }
}

TEST_F(WaypointFollowerTests, RetargetRestartsHold) {
    WaypointFollower follower(get_params(), 0.01);

    Waypoint wp;
    wp.pose = Pose{1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    wp.mode = WaypointMode::FULL_POSE;
    follower.start(zero_pose(), zero_twist(), wp, 0.1);

    const Pose at_goal{1.0, 0.0, 0.0, 1, 0, 0, 0};
    EXPECT_FALSE(follower.update_convergence(at_goal, 0.0, false, 1.0));

    follower.retarget(wp, 0.1);  // same target, new goal
    // The 1 s of hold started before the retarget must not count.
    EXPECT_FALSE(follower.update_convergence(at_goal, 1.0, false, 1.0));
    EXPECT_TRUE(follower.update_convergence(at_goal, 2.0, false, 1.0));
}

}  // namespace vortex::guidance

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
