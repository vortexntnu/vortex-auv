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

}  // namespace vortex::guidance

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
