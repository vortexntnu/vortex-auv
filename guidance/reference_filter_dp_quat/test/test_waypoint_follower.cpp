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

    follower.start(zero_pose(), zero_twist(), wp, {0.1, 0.1});

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

    follower.start(zero_pose(), zero_twist(), wp, {0.1, 0.1});

    follower.step();

    EXPECT_FALSE(follower.within_convergance(zero_pose()));
}

TEST_F(WaypointFollowerTests, SetReferenceUpdatesMidSequence) {
    WaypointFollower follower(get_params(), 0.01);

    Waypoint wp;
    wp.pose = Pose{1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    wp.mode = WaypointMode::FULL_POSE;

    follower.start(zero_pose(), zero_twist(), wp, {0.1, 0.1});

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

    follower.start(zero_pose(), zero_twist(), wp, {0.1, 0.1});
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

    follower.start(zero_pose(), zero_twist(), wp, {0.1, 0.1});

    // Run several steps — state should move toward reference
    for (int i = 0; i < 100; ++i) {
        follower.step();
    }

    // x position should have moved toward 1.0
    EXPECT_GT(follower.pose().x, 0.0);
}

TEST_F(WaypointFollowerTests, HoldTimeRequiresToleranceToHold) {
    WaypointFollower follower(get_params(), 0.01);
    Waypoint wp;
    wp.pose = Pose{1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    follower.start(zero_pose(), zero_twist(), wp, {0.1, 0.1});
    const Pose at_goal = Pose{1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    constexpr double hold_time_sec = 1.0;

    EXPECT_FALSE(follower.update_convergence(at_goal, 0.0, hold_time_sec));
    EXPECT_FALSE(follower.update_convergence(at_goal, 0.5, hold_time_sec));
    EXPECT_TRUE(follower.update_convergence(at_goal, 1.0, hold_time_sec));
}

TEST_F(WaypointFollowerTests, HoldTimeResetsWhenLeavingTolerance) {
    WaypointFollower follower(get_params(), 0.01);
    Waypoint wp;
    wp.pose = Pose{1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    follower.start(zero_pose(), zero_twist(), wp, {0.1, 0.1});
    const Pose at_goal = Pose{1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    const Pose away = Pose{0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    constexpr double hold_time_sec = 1.0;

    EXPECT_FALSE(follower.update_convergence(at_goal, 0.0, hold_time_sec));
    EXPECT_FALSE(follower.update_convergence(away, 0.5, hold_time_sec));
    EXPECT_FALSE(follower.update_convergence(at_goal, 0.6, hold_time_sec));
    EXPECT_FALSE(follower.update_convergence(at_goal, 1.2, hold_time_sec));
    EXPECT_TRUE(follower.update_convergence(at_goal, 1.6, hold_time_sec));
}

TEST_F(WaypointFollowerTests, ZeroHoldTimeConvergesOnFirstSample) {
    WaypointFollower follower(get_params(), 0.01);
    Waypoint wp;
    wp.pose = Pose{1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    follower.start(zero_pose(), zero_twist(), wp, {0.1, 0.1});
    const Pose at_goal = Pose{1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    const Pose away = Pose{0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};

    EXPECT_FALSE(follower.update_convergence(away, 0.0, 0.0));
    EXPECT_TRUE(follower.update_convergence(at_goal, 1.0, 0.0));
}

TEST_F(WaypointFollowerTests, RetargetKeepsFilterStateAndChangesTarget) {
    WaypointFollower follower(get_params(), 0.01);
    Waypoint wp;
    wp.pose = Pose{1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    follower.start(zero_pose(), zero_twist(), wp, {0.1, 0.1});
    for (int i = 0; i < 100; ++i) {
        follower.step();
    }
    const Pose pose_before = follower.pose();
    const Eigen::Vector6d velocity_before = follower.velocity();

    Waypoint new_wp;
    new_wp.pose = Pose{3.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    follower.retarget(new_wp, {0.5, 0.5});

    EXPECT_DOUBLE_EQ(follower.pose().x, pose_before.x);
    EXPECT_TRUE(follower.velocity().isApprox(velocity_before));
    EXPECT_DOUBLE_EQ(follower.waypoint_goal().x, new_wp.pose.x);
    const Pose at_new_goal{3.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    const Pose at_old_goal{1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    EXPECT_TRUE(follower.within_convergance(at_new_goal));
    EXPECT_FALSE(follower.within_convergance(at_old_goal));
}

TEST_F(WaypointFollowerTests, StartResetsFilterState) {
    WaypointFollower follower(get_params(), 0.01);
    Waypoint wp;
    wp.pose = Pose{1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    follower.start(zero_pose(), zero_twist(), wp, {0.1, 0.1});
    for (int i = 0; i < 100; ++i) {
        follower.step();
    }

    follower.start(zero_pose(), zero_twist(), wp, {0.1, 0.1});

    EXPECT_DOUBLE_EQ(follower.pose().x, 0.0);
    EXPECT_TRUE(follower.velocity().isZero());
}

TEST_F(WaypointFollowerTests, StartAndRetargetRestartHoldTimer) {
    WaypointFollower follower(get_params(), 0.01);
    Waypoint wp;
    wp.pose = Pose{1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    const Pose at_goal = Pose{1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    constexpr double hold_time_sec = 1.0;

    follower.start(zero_pose(), zero_twist(), wp, {0.1, 0.1});
    EXPECT_FALSE(follower.update_convergence(at_goal, 0.0, hold_time_sec));
    follower.retarget(wp, {0.1, 0.1});
    EXPECT_FALSE(follower.update_convergence(at_goal, 0.5, hold_time_sec));
    EXPECT_FALSE(follower.update_convergence(at_goal, 1.2, hold_time_sec));
    EXPECT_TRUE(follower.update_convergence(at_goal, 1.5, hold_time_sec));

    follower.start(zero_pose(), zero_twist(), wp, {0.1, 0.1});
    EXPECT_FALSE(follower.update_convergence(at_goal, 2.0, hold_time_sec));
    EXPECT_FALSE(follower.update_convergence(at_goal, 2.5, hold_time_sec));
    EXPECT_TRUE(follower.update_convergence(at_goal, 3.0, hold_time_sec));
}

}  // namespace vortex::guidance

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
