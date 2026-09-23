#include <gtest/gtest.h>
#include "landmark_server/course_frame.hpp"
#include "landmark_server/retained_landmarks.hpp"
#include "test_map_utils.hpp"

namespace vortex::mission {

using namespace test;

namespace {

Eigen::Vector3d v(double x, double y, double z = 2.0) { return {x, y, z}; }

}  // namespace

TEST(RetainedLandmarks, GateDeletedByTrackerIsRememberedAndAges) {
    RetainedLandmarks map(example_config());

    map.update({make_track(1, LT::GATE, LS::GATE_WHOLE, v(10, 0))}, 100.0);
    ASSERT_EQ(map.landmarks().size(), 1u);
    const int id = map.landmarks()[0].id;
    EXPECT_GE(map.landmarks()[0].live_track_id, 0);

    // The tracker deleted the track: no live tracks any more.
    for (double now : {101.0, 130.0, 220.0}) {
        map.update({}, now);
        ASSERT_EQ(map.landmarks().size(), 1u) << "at t=" << now;
        const auto& lm = map.landmarks()[0];
        EXPECT_EQ(lm.id, id);
        EXPECT_EQ(lm.live_track_id, -1);
        // Age grows: nothing updates last_measurement.
        EXPECT_DOUBLE_EQ(lm.last_measurement, 100.0);
    }
}

TEST(RetainedLandmarks, NewDetectionNearTheGateKeepsItsId) {
    RetainedLandmarks map(example_config());
    map.update({make_track(1, LT::GATE, LS::GATE_WHOLE, v(10, 0))}, 0.0);
    const int id = map.landmarks()[0].id;
    map.update({}, 1.0);

    // A new track (new tracker id) 0.4 m away: same object.
    map.update({make_track(7, LT::GATE, LS::GATE_WHOLE, v(10.3, 0.3))}, 2.0);

    ASSERT_EQ(map.landmarks().size(), 1u);
    EXPECT_EQ(map.landmarks()[0].id, id);
    EXPECT_EQ(map.landmarks()[0].live_track_id, 7);
    EXPECT_NEAR(map.landmarks()[0].position.x(), 10.3, 1e-12);
    EXPECT_DOUBLE_EQ(map.landmarks()[0].last_measurement, 2.0);
}

TEST(RetainedLandmarks, FalseGate8mAwayDoesNotTakeOver) {
    RetainedLandmarks map(example_config());
    map.update({make_track(1, LT::GATE, LS::GATE_WHOLE, v(10, 0))}, 0.0);
    const int id = map.landmarks()[0].id;
    map.update({}, 1.0);

    map.update({make_track(9, LT::GATE, LS::GATE_WHOLE, v(18, 0))}, 2.0);

    ASSERT_EQ(map.landmarks().size(), 1u);
    EXPECT_EQ(map.landmarks()[0].id, id);
    EXPECT_NEAR(map.landmarks()[0].position.x(), 10.0, 1e-12);
    EXPECT_EQ(map.landmarks()[0].live_track_id, -1);
    EXPECT_EQ(map.rejected_count(), 1);
}

TEST(RetainedLandmarks, TwoBinsDoNotSwapIds) {
    RetainedLandmarks map(example_config());
    map.update({make_track(1, LT::BIN, LS::BIN_UNCLASSIFIED, v(20, 1)),
                make_track(2, LT::BIN, LS::BIN_UNCLASSIFIED, v(20, 2))},
               0.0);
    ASSERT_EQ(map.landmarks().size(), 2u);
    const int id_a = map.landmarks()[0].id;
    const int id_b = map.landmarks()[1].id;
    EXPECT_NE(id_a, id_b);

    // Both tracks are recreated in the opposite order with new track ids.
    map.update({}, 1.0);
    map.update({make_track(12, LT::BIN, LS::BIN_UNCLASSIFIED, v(20.05, 2.0)),
                make_track(11, LT::BIN, LS::BIN_UNCLASSIFIED, v(20.05, 1.0))},
               2.0);

    ASSERT_EQ(map.landmarks().size(), 2u);
    const auto* a = map.find(id_a);
    const auto* b = map.find(id_b);
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    EXPECT_NEAR(a->position.y(), 1.0, 1e-12);
    EXPECT_NEAR(b->position.y(), 2.0, 1e-12);
}

TEST(RetainedLandmarks, ClassIsFullAtMaxInstances) {
    RetainedLandmarks map(example_config());
    std::vector<Track> tracks;
    for (int i = 0; i < 7; ++i) {
        tracks.push_back(make_track(i, LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED,
                                    v(10.0 + 2.0 * i, 0.0)));
    }
    map.update(tracks, 0.0);
    // max 5 red pipes.
    EXPECT_EQ(map.landmarks().size(), 5u);
    EXPECT_EQ(map.rejected_count(), 2);
}

TEST(RetainedLandmarks, PipeNextToTheGateIsRejected) {
    RetainedLandmarks map(example_config());
    map.update({make_track(1, LT::GATE, LS::GATE_WHOLE, v(10, 0))}, 0.0);

    // 1 m from the gate: a gate leg, not a pipe.
    map.update({make_track(1, LT::GATE, LS::GATE_WHOLE, v(10, 0)),
                make_track(2, LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE, v(10, 1))},
               1.0);
    EXPECT_EQ(map.landmarks().size(), 1u);
    EXPECT_EQ(map.rejected_count(), 1);

    // 2 m from the gate is fine.
    map.update({make_track(1, LT::GATE, LS::GATE_WHOLE, v(10, 0)),
                make_track(3, LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE, v(10, 2))},
               2.0);
    EXPECT_EQ(map.landmarks().size(), 2u);
}

TEST(RetainedLandmarks, PipesAreForgottenAfterRetainSecUnlessWellObserved) {
    RetainedLandmarks map(example_config());

    // Pipe A: seen once. Pipe B: seen 30 times.
    map.update({make_track(1, LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED, v(10, 0)),
                make_track(2, LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED, v(10, 3))},
               0.0);
    for (int i = 1; i < 30; ++i) {
        map.update({make_track(2, LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED, v(10, 3))},
                   0.1 * i);
    }
    ASSERT_EQ(map.landmarks().size(), 2u);
    const int id_b = map.landmarks()[1].id;
    EXPECT_GE(map.find(id_b)->observations, 30);

    // The tracker forgets both. After 15 s only the well observed one stays.
    map.update({}, 5.0);
    EXPECT_EQ(map.landmarks().size(), 2u);
    map.update({}, 30.0);
    ASSERT_EQ(map.landmarks().size(), 1u);
    EXPECT_EQ(map.landmarks()[0].id, id_b);
    map.update({}, 3600.0);
    EXPECT_EQ(map.landmarks().size(), 1u);
}

TEST(RetainedLandmarks, GateInTheNeighbourLaneIsRejected) {
    const auto cfg = example_config();
    RetainedLandmarks map(cfg);

    CourseFrameTracker course(cfg.course_frame);
    // Start at the origin facing odom +y (yaw 90 deg), course straight ahead.
    ASSERT_TRUE(course.set_coarse(vortex::utils::types::Pose::from_eigen(
                                      Eigen::Vector3d(0, 0, 0),
                                      Eigen::Quaterniond(Eigen::AngleAxisd(
                                          M_PI_2, Eigen::Vector3d::UnitZ()))),
                                  0.0)
                    .success);
    const auto allowed = [&](const Eigen::Vector3d& p) {
        return course.position_allowed(p);
    };

    // In the course frame x is odom +y and y (left) is odom +x. The gate is
    // 8 m ahead of the start; the neighbour's gate is 20 m to the side
    // (course y = 20, beyond the +-12 m limit).
    map.update({make_track(1, LT::GATE, LS::GATE_WHOLE, v(0, 8)),
                make_track(2, LT::GATE, LS::GATE_SURVEY_REPAIR, v(20, 8))},
               0.0, allowed);

    ASSERT_EQ(map.landmarks().size(), 1u);
    EXPECT_EQ(map.landmarks()[0].key.subtype, LS::GATE_WHOLE);
    EXPECT_EQ(map.rejected_count(), 1);
}

TEST(RetainedLandmarks, TighteningBoundsDropsWhatIsNowOutside) {
    const auto cfg = example_config();
    RetainedLandmarks map(cfg);
    map.update({make_track(1, LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED, v(10, 0)),
                make_track(2, LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED, v(10, 9))},
               0.0);
    ASSERT_EQ(map.landmarks().size(), 2u);

    // Only |y| <= 6 m is allowed now.
    map.update({}, 1.0, [](const Eigen::Vector3d& p) { return std::abs(p.y()) <= 6.0; });
    EXPECT_EQ(map.landmarks().size(), 1u);
}

TEST(RetainedLandmarks, DetectionWithoutOrientationDoesNotChangeYaw) {
    RetainedLandmarks map(example_config());
    const Eigen::Quaterniond yawed(Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ()));

    map.update({make_track(1, LT::GATE, LS::GATE_WHOLE, v(10, 0), true, true, yawed)},
               0.0);
    ASSERT_TRUE(map.landmarks()[0].has_orientation);

    // The tracker now reports a track without orientation (identity is a
    // placeholder): the map keeps the yaw it has.
    map.update({make_track(1, LT::GATE, LS::GATE_WHOLE, v(10, 0), true, false)}, 1.0);
    EXPECT_TRUE(map.landmarks()[0].has_orientation);
    EXPECT_NEAR(map.landmarks()[0].yaw(), 1.0, 1e-12);

    // A landmark that never had an orientation reports none.
    map.update({make_track(5, LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED, v(12, 4), true, false)},
               2.0);
    EXPECT_FALSE(map.landmarks()[1].has_orientation);
}

TEST(RetainedLandmarks, LockedYawIsNotOverwritten) {
    RetainedLandmarks map(example_config());
    const Eigen::Quaterniond a(Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ()));
    const Eigen::Quaterniond b(Eigen::AngleAxisd(-2.0, Eigen::Vector3d::UnitZ()));
    map.update({make_track(1, LT::GATE, LS::GATE_WHOLE, v(10, 0), true, true, a)}, 0.0);
    map.landmarks()[0].yaw_locked = true;

    map.update({make_track(1, LT::GATE, LS::GATE_WHOLE, v(10, 0), true, true, b)}, 1.0);
    EXPECT_NEAR(map.landmarks()[0].yaw(), 1.0, 1e-12);
}

TEST(RetainedLandmarks, ClearForgetsEverythingButNeverReusesIds) {
    RetainedLandmarks map(example_config());
    map.update({make_track(1, LT::GATE, LS::GATE_WHOLE, v(10, 0))}, 0.0);
    const int id = map.landmarks()[0].id;
    map.clear();
    EXPECT_TRUE(map.landmarks().empty());
    map.update({make_track(1, LT::GATE, LS::GATE_WHOLE, v(10, 0))}, 1.0);
    ASSERT_EQ(map.landmarks().size(), 1u);
    EXPECT_GT(map.landmarks()[0].id, id);
}

}  // namespace vortex::mission
