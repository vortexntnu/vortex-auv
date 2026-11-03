#include <gtest/gtest.h>
#include "reference_filter_dp/lib/waypoint_follower.hpp"
#include "reference_filter_dp/lib/waypoint_utils.hpp"

namespace vortex::guidance {

class WaypointFollowerTests : public ::testing::Test {
   protected:
    ReferenceFilterParams get_params() {
        ReferenceFilterParams params;
        params.omega = Eigen::Vector6d::Ones();
        params.zeta = Eigen::Vector6d::Ones();
        return params;
    }

    PoseEuler zero_pose() { return PoseEuler{}; }
    Twist zero_twist() { return Twist{}; }
};

TEST_F(WaypointFollowerTests, StartAndStepConverges) {
    WaypointFollower follower(get_params(), 0.01);

    Waypoint wp;
    wp.pose = PoseEuler{1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    wp.mode = WaypointMode::FULL_POSE;

    follower.start(zero_pose(), zero_twist(), wp, {0.1, 0.1});

    Eigen::Vector18d state = follower.step();

    // Simulate the measured pose being at the reference
    Eigen::Vector6d measured_at_ref;
    measured_at_ref << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    EXPECT_TRUE(follower.within_convergance(measured_at_ref));
    EXPECT_EQ(state.size(), 18);
}

TEST_F(WaypointFollowerTests, StepDoesNotConvergeWhenFar) {
    WaypointFollower follower(get_params(), 0.01);

    Waypoint wp;
    wp.pose = PoseEuler{10.0, 10.0, 0.0, 0.0, 0.0, 0.0};
    wp.mode = WaypointMode::FULL_POSE;

    follower.start(zero_pose(), zero_twist(), wp, {0.1, 0.1});

    follower.step();

    Eigen::Vector6d measured_far = Eigen::Vector6d::Zero();
    EXPECT_FALSE(follower.within_convergance(measured_far));
}

TEST_F(WaypointFollowerTests, SetReferenceUpdatesMidSequence) {
    WaypointFollower follower(get_params(), 0.01);

    Waypoint wp;
    wp.pose = PoseEuler{1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    wp.mode = WaypointMode::FULL_POSE;

    follower.start(zero_pose(), zero_twist(), wp, {0.1, 0.1});

    PoseEuler new_ref{5.0, 5.0, 0.0, 0.0, 0.0, 0.0};
    follower.set_reference(new_ref);

    EXPECT_DOUBLE_EQ(follower.reference()(0), 5.0);
    EXPECT_DOUBLE_EQ(follower.reference()(1), 5.0);
}

TEST_F(WaypointFollowerTests, SnapStateToReference) {
    WaypointFollower follower(get_params(), 0.01);

    Waypoint wp;
    wp.pose = PoseEuler{3.0, 4.0, 5.0, 0.1, 0.2, 0.3};
    wp.mode = WaypointMode::FULL_POSE;

    follower.start(zero_pose(), zero_twist(), wp, {0.1, 0.1});
    follower.snap_state_to_reference();

    Eigen::Vector18d state = follower.state();
    Eigen::Vector6d ref = follower.reference();

    for (int i = 0; i < 6; ++i) {
        EXPECT_DOUBLE_EQ(state(i), ref(i));
    }
}

TEST_F(WaypointFollowerTests, StateEvolvesWithStep) {
    WaypointFollower follower(get_params(), 0.01);

    Waypoint wp;
    wp.pose = PoseEuler{1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    wp.mode = WaypointMode::FULL_POSE;

    follower.start(zero_pose(), zero_twist(), wp, {0.1, 0.1});

    // Run several steps — state should move toward reference
    for (int i = 0; i < 100; ++i) {
        follower.step();
    }

    // x position should have moved toward 1.0
    EXPECT_GT(follower.state()(0), 0.0);
}

TEST_F(WaypointFollowerTests, HoldTimeRequiresToleranceToHold) {
    WaypointFollower follower(get_params(), 0.01);
    Waypoint wp;
    wp.pose = PoseEuler{1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    follower.start(zero_pose(), zero_twist(), wp, {0.1, 0.1});
    Eigen::Vector6d at_goal = Eigen::Vector6d::Zero();
    at_goal(0) = 1.0;
    constexpr double hold_time_sec = 1.0;

    EXPECT_FALSE(follower.update_convergence(at_goal, 0.0, hold_time_sec));
    EXPECT_FALSE(follower.update_convergence(at_goal, 0.5, hold_time_sec));
    EXPECT_TRUE(follower.update_convergence(at_goal, 1.0, hold_time_sec));
}

TEST_F(WaypointFollowerTests, HoldTimeResetsWhenLeavingTolerance) {
    WaypointFollower follower(get_params(), 0.01);
    Waypoint wp;
    wp.pose = PoseEuler{1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    follower.start(zero_pose(), zero_twist(), wp, {0.1, 0.1});
    Eigen::Vector6d at_goal = Eigen::Vector6d::Zero();
    at_goal(0) = 1.0;
    const Eigen::Vector6d away = Eigen::Vector6d::Zero();
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
    wp.pose = PoseEuler{1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    follower.start(zero_pose(), zero_twist(), wp, {0.1, 0.1});
    Eigen::Vector6d at_goal = Eigen::Vector6d::Zero();
    at_goal(0) = 1.0;
    const Eigen::Vector6d away = Eigen::Vector6d::Zero();

    EXPECT_FALSE(follower.update_convergence(away, 0.0, 0.0));
    EXPECT_TRUE(follower.update_convergence(at_goal, 1.0, 0.0));
}

TEST_F(WaypointFollowerTests, RetargetKeepsFilterStateAndChangesTarget) {
    WaypointFollower follower(get_params(), 0.01);
    Waypoint wp;
    wp.pose = PoseEuler{1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    follower.start(zero_pose(), zero_twist(), wp, {0.1, 0.1});
    for (int i = 0; i < 100; ++i) {
        follower.step();
    }
    const Eigen::Vector18d state_before = follower.state();

    Waypoint new_wp;
    new_wp.pose = PoseEuler{3.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    follower.retarget(new_wp, {0.5, 0.5});

    EXPECT_TRUE(follower.state().isApprox(state_before));
    Eigen::Vector6d at_new_goal = Eigen::Vector6d::Zero();
    at_new_goal(0) = 3.0;
    Eigen::Vector6d at_old_goal = Eigen::Vector6d::Zero();
    at_old_goal(0) = 1.0;
    EXPECT_TRUE(follower.within_convergance(at_new_goal));
    EXPECT_FALSE(follower.within_convergance(at_old_goal));
}

TEST_F(WaypointFollowerTests, StartResetsFilterState) {
    WaypointFollower follower(get_params(), 0.01);
    Waypoint wp;
    wp.pose = PoseEuler{1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    follower.start(zero_pose(), zero_twist(), wp, {0.1, 0.1});
    for (int i = 0; i < 100; ++i) {
        follower.step();
    }

    follower.start(zero_pose(), zero_twist(), wp, {0.1, 0.1});

    EXPECT_TRUE(follower.state().isApprox(Eigen::Vector18d::Zero()));
}

TEST_F(WaypointFollowerTests, StartAndRetargetRestartHoldTimer) {
    WaypointFollower follower(get_params(), 0.01);
    Waypoint wp;
    wp.pose = PoseEuler{1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    Eigen::Vector6d at_goal = Eigen::Vector6d::Zero();
    at_goal(0) = 1.0;
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
