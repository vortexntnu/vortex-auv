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

    follower.start(zero_pose(), zero_twist(), wp, 0.1);

    // Simulate the measured pose being at the reference
    Eigen::Vector6d measured_at_ref;
    measured_at_ref << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    StepResult result = follower.step(measured_at_ref);

    EXPECT_TRUE(result.target_reached);
    EXPECT_EQ(result.reference_state.size(), 18);
}

TEST_F(WaypointFollowerTests, StepDoesNotConvergeWhenFar) {
    WaypointFollower follower(get_params(), 0.01);

    Waypoint wp;
    wp.pose = PoseEuler{10.0, 10.0, 0.0, 0.0, 0.0, 0.0};
    wp.mode = WaypointMode::FULL_POSE;

    follower.start(zero_pose(), zero_twist(), wp, 0.1);

    Eigen::Vector6d measured_far = Eigen::Vector6d::Zero();
    StepResult result = follower.step(measured_far);

    EXPECT_FALSE(result.target_reached);
}

TEST_F(WaypointFollowerTests, SetReferenceUpdatesMidSequence) {
    WaypointFollower follower(get_params(), 0.01);

    Waypoint wp;
    wp.pose = PoseEuler{1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    wp.mode = WaypointMode::FULL_POSE;

    follower.start(zero_pose(), zero_twist(), wp, 0.1);

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

    follower.start(zero_pose(), zero_twist(), wp, 0.1);
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

    follower.start(zero_pose(), zero_twist(), wp, 0.1);

    Eigen::Vector6d measured = Eigen::Vector6d::Zero();

    // Run several steps — state should move toward reference
    for (int i = 0; i < 100; ++i) {
        follower.step(measured);
    }

    // x position should have moved toward 1.0
    EXPECT_GT(follower.state()(0), 0.0);
}

}  // namespace vortex::guidance

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
