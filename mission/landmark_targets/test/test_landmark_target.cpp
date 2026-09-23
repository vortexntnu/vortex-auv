#include <gtest/gtest.h>
#include "landmark_targets/landmark_target.hpp"
#include "test_utils.hpp"

namespace vortex::mission {

using test::make_pose;

namespace {

MapLandmark landmark_at(double x, double y, double z, double last_measurement) {
    MapLandmark lm;
    lm.id = 7;
    lm.pose = make_pose(x, y, z);
    lm.has_orientation = false;
    lm.last_measurement = last_measurement;
    return lm;
}

TargetSpec odom_axes_spec() {
    TargetSpec spec;
    spec.offset = make_pose(0.0, 0.0, -1.0);  // 1 m above
    spec.frame = OffsetFrame::LANDMARK_ODOM_AXES;
    spec.dead_reckoning_distance = 0.5;
    spec.track_loss_timeout_sec = 5.0;
    spec.resend_distance = 0.05;
    spec.min_resend_interval_sec = 0.0;
    return spec;
}

const Pose kFar = make_pose(-20.0, 0.0, 0.0);

}  // namespace

TEST(LandmarkTarget, FirstStepSendsTheInitialGoal) {
    LandmarkTarget target(odom_axes_spec(), 7);

    const auto step = target.step(landmark_at(5.0, 0.0, 2.0, 0.0), kFar, 0.0);

    ASSERT_TRUE(step.send_goal.has_value());
    EXPECT_NEAR(step.send_goal->x, 5.0, 1e-12);
    EXPECT_NEAR(step.send_goal->z, 1.0, 1e-12);
    EXPECT_EQ(step.phase, Phase::TRACKING);
}

TEST(LandmarkTarget, NewGoalOnlyWhenTargetMovedMoreThanThreshold) {
    LandmarkTarget target(odom_axes_spec(), 7);
    target.step(landmark_at(5.0, 0.0, 2.0, 0.0), kFar, 0.0);

    // 3 cm: below the 5 cm threshold.
    auto step = target.step(landmark_at(5.03, 0.0, 2.0, 1.0), kFar, 1.0);
    EXPECT_FALSE(step.send_goal.has_value());

    // 8 cm from the last sent goal.
    step = target.step(landmark_at(5.08, 0.0, 2.0, 2.0), kFar, 2.0);
    ASSERT_TRUE(step.send_goal.has_value());
    EXPECT_NEAR(step.send_goal->x, 5.08, 1e-12);

    // Compared against the last SENT goal, not the last estimate.
    step = target.step(landmark_at(5.10, 0.0, 2.0, 3.0), kFar, 3.0);
    EXPECT_FALSE(step.send_goal.has_value());
}

TEST(LandmarkTarget, RateLimitsNewGoals) {
    TargetSpec spec = odom_axes_spec();
    spec.min_resend_interval_sec = 0.5;
    LandmarkTarget target(spec, 7);
    target.step(landmark_at(5.0, 0.0, 2.0, 0.0), kFar, 0.0);

    EXPECT_FALSE(
        target.step(landmark_at(6.0, 0.0, 2.0, 0.2), kFar, 0.2).send_goal);
    EXPECT_TRUE(
        target.step(landmark_at(6.0, 0.0, 2.0, 0.6), kFar, 0.6).send_goal);
}

TEST(LandmarkTarget, FreezeComputesTheTargetOnce) {
    TargetSpec spec = odom_axes_spec();
    spec.freeze = true;
    LandmarkTarget target(spec, 7);

    const auto first = target.step(landmark_at(5.0, 0.0, 2.0, 0.0), kFar, 0.0);
    ASSERT_TRUE(first.send_goal.has_value());

    // The landmark moves a lot afterwards: no update, nothing to do.
    const auto later = target.step(landmark_at(9.0, 3.0, 2.0, 1.0), kFar, 1.0);
    EXPECT_FALSE(later.send_goal.has_value());
    EXPECT_EQ(later.phase, Phase::DEAD_RECKONING);
    ASSERT_TRUE(target.last_goal().has_value());
    EXPECT_NEAR(target.last_goal()->x, 5.0, 1e-12);
}

TEST(LandmarkTarget, DeadReckoningWithinDistanceStopsUpdating) {
    LandmarkTarget target(odom_axes_spec(), 7);
    target.step(landmark_at(5.0, 0.0, 2.0, 0.0), kFar, 0.0);

    // Vehicle 0.3 m from the target (5, 0, 1): inside the 0.5 m limit.
    const Pose close = make_pose(5.0, 0.0, 1.3);
    auto step = target.step(landmark_at(5.0, 0.0, 2.0, 1.0), close, 1.0);
    EXPECT_EQ(step.phase, Phase::DEAD_RECKONING);

    // Afterwards the landmark and its movement are ignored.
    step = target.step(landmark_at(8.0, 0.0, 2.0, 2.0), close, 2.0);
    EXPECT_FALSE(step.send_goal.has_value());
    EXPECT_EQ(step.phase, Phase::DEAD_RECKONING);
    step = target.step(std::nullopt, close, 100.0);
    EXPECT_EQ(step.phase, Phase::DEAD_RECKONING);
}

TEST(LandmarkTarget, TrackLossAfterTimeout) {
    LandmarkTarget target(odom_axes_spec(), 7);
    target.step(landmark_at(5.0, 0.0, 2.0, 0.0), kFar, 0.0);

    // Landmark gone from the map: holds the last goal until the timeout.
    auto step = target.step(std::nullopt, kFar, 1.0);
    EXPECT_EQ(step.phase, Phase::TRACKING);
    EXPECT_FALSE(step.send_goal.has_value());
    step = target.step(std::nullopt, kFar, 5.5);  // 4.5 s after the first miss
    EXPECT_EQ(step.phase, Phase::TRACKING);
    step = target.step(std::nullopt, kFar, 6.5);  // 5.5 s
    EXPECT_EQ(step.phase, Phase::LOST);

    // LOST is terminal.
    step = target.step(landmark_at(5.0, 0.0, 2.0, 7.0), kFar, 7.0);
    EXPECT_EQ(step.phase, Phase::LOST);
    EXPECT_FALSE(step.send_goal.has_value());
}

TEST(LandmarkTarget, ReacquiredLandmarkResetsTheTimer) {
    LandmarkTarget target(odom_axes_spec(), 7);
    target.step(landmark_at(5.0, 0.0, 2.0, 0.0), kFar, 0.0);
    target.step(std::nullopt, kFar, 1.0);
    target.step(std::nullopt, kFar, 4.0);
    // Seen again just before the timeout.
    target.step(landmark_at(5.0, 0.0, 2.0, 5.0), kFar, 5.0);
    // A new miss starts a new timeout.
    EXPECT_EQ(target.step(std::nullopt, kFar, 8.0).phase, Phase::TRACKING);
    EXPECT_EQ(target.step(std::nullopt, kFar, 13.5).phase, Phase::LOST);
}

TEST(LandmarkTarget, StaleRetainedLandmarkCountsAsLost) {
    LandmarkTarget target(odom_axes_spec(), 7);
    // The map keeps the landmark, but it was last measured 6 s ago.
    const auto step = target.step(landmark_at(5.0, 0.0, 2.0, 0.0), kFar, 6.0);
    EXPECT_EQ(step.phase, Phase::LOST);
    EXPECT_FALSE(step.send_goal.has_value());
}

TEST(LandmarkTarget, IgnoresOtherLandmarkIds) {
    LandmarkTarget target(odom_axes_spec(), 7);
    MapLandmark other = landmark_at(1.0, 1.0, 1.0, 0.0);
    other.id = 8;

    const auto step = target.step(other, kFar, 0.0);
    EXPECT_FALSE(step.send_goal.has_value());
    EXPECT_EQ(step.phase, Phase::TRACKING);
}

TEST(LandmarkTarget, LandmarkFrameWaitsForOrientation) {
    TargetSpec spec = odom_axes_spec();
    spec.frame = OffsetFrame::LANDMARK;
    spec.offset = make_pose(2.0, 0.0, 0.0);
    LandmarkTarget target(spec, 7);

    // No orientation yet: no goal, no exception.
    auto step = target.step(landmark_at(5.0, 0.0, 2.0, 0.0), kFar, 0.0);
    EXPECT_FALSE(step.send_goal.has_value());

    MapLandmark oriented = landmark_at(5.0, 0.0, 2.0, 1.0);
    oriented.has_orientation = true;
    step = target.step(oriented, kFar, 1.0);
    ASSERT_TRUE(step.send_goal.has_value());
    EXPECT_NEAR(step.send_goal->x, 7.0, 1e-12);
}

}  // namespace vortex::mission
