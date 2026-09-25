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

namespace {

/// Two remembered white pipes 0.8 m apart (A at y = 0, B at y = 0.8), no live
/// tracks. Returns their ids.
std::pair<int, int> two_remembered_pipes(RetainedLandmarks& map) {
    map.update({make_track(1, LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE, v(10, 0)),
                make_track(2, LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE, v(10, 0.8))},
               0.0);
    map.update({}, 1.0);
    return {map.landmarks()[0].id, map.landmarks()[1].id};
}

}  // namespace

TEST(RetainedLandmarks, AmbiguousTakeOverWaitsThenCountsAsNew) {
    RetainedLandmarks map(example_config());
    const auto [a, b] = two_remembered_pipes(map);

    // 0.35 m from A, 0.45 m from B: either could be it.
    const auto track = make_track(5, LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE,
                                  v(10, 0.35));
    map.update({track}, 2.0);
    EXPECT_EQ(map.find(a)->live_track_id, -1);
    EXPECT_EQ(map.find(b)->live_track_id, -1);
    EXPECT_EQ(map.landmarks().size(), 2u) << "no new landmark while waiting";
    EXPECT_EQ(map.ambiguous_count(), 1);

    // Still ambiguous after wait_sec (2 s): a new object, the old ones keep
    // their ids and positions.
    map.update({track}, 3.0);
    EXPECT_EQ(map.landmarks().size(), 2u);
    map.update({track}, 4.1);
    ASSERT_EQ(map.landmarks().size(), 3u);
    EXPECT_EQ(map.find(a)->live_track_id, -1);
    EXPECT_EQ(map.find(b)->live_track_id, -1);
    EXPECT_NEAR(map.find(a)->position.y(), 0.0, 1e-12);
    EXPECT_NEAR(map.find(b)->position.y(), 0.8, 1e-12);
}

TEST(RetainedLandmarks, AmbiguityResolvedByACloserLookTakesOver) {
    RetainedLandmarks map(example_config());
    const auto [a, b] = two_remembered_pipes(map);

    map.update({make_track(5, LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE,
                           v(10, 0.35))},
               2.0);
    EXPECT_EQ(map.find(a)->live_track_id, -1);

    // The track settles 0.1 m from A (0.7 m from B): clearly A.
    map.update({make_track(5, LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE,
                           v(10, 0.1))},
               2.5);
    EXPECT_EQ(map.find(a)->live_track_id, 5);
    EXPECT_EQ(map.find(b)->live_track_id, -1);
    EXPECT_EQ(map.landmarks().size(), 2u);
}

TEST(RetainedLandmarks, ClearlyNearestTakesOverAtOnce) {
    RetainedLandmarks map(example_config());
    const auto [a, b] = two_remembered_pipes(map);
    // 0.2 m from A, 0.6 m from B: ratio 3.
    map.update({make_track(5, LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE,
                           v(10, 0.2))},
               2.0);
    EXPECT_EQ(map.find(a)->live_track_id, 5);
    EXPECT_EQ(map.ambiguous_count(), 0);
    (void)b;
}

TEST(RetainedLandmarks, AFollowedLandmarkDoesNotMakeATakeOverAmbiguous) {
    RetainedLandmarks map(example_config());
    const auto [a, b] = two_remembered_pipes(map);
    // Both pipes are seen again at once, shifted by drift (0.3 m for A, 0.2 m for B). The track
    // on B is clear; once B is followed, the one near A is clear too.
    const auto near_a = make_track(5, LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE,
                                   v(10, 0.3));
    const auto near_b = make_track(6, LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE,
                                   v(10, 1.0));
    map.update({near_a, near_b}, 2.0);
    EXPECT_EQ(map.find(b)->live_track_id, 6);
    map.update({near_a, near_b}, 2.2);
    EXPECT_EQ(map.find(a)->live_track_id, 5);
    EXPECT_EQ(map.landmarks().size(), 2u);
}

TEST(RetainedLandmarks, AmbiguousTrackInAFullClassIsRejectedAfterTheWait) {
    auto cfg = example_config();
    auto rule = cfg.rule_for({LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE});
    rule.max_instances = 2;
    cfg.class_rules[{LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE}] = rule;
    RetainedLandmarks map(cfg);
    two_remembered_pipes(map);
    const auto track = make_track(5, LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE,
                                  v(10, 0.35));
    map.update({track}, 2.0);
    map.update({track}, 4.5);
    EXPECT_EQ(map.landmarks().size(), 2u);
    EXPECT_GE(map.rejected_count(), 1);
    for (const auto& lm : map.landmarks()) {
        EXPECT_EQ(lm.live_track_id, -1);
    }
}

TEST(RetainedLandmarks, RatioOneTurnsTheCheckOff) {
    auto cfg = example_config();
    cfg.adoption_ambiguity_ratio = 1.0;
    RetainedLandmarks map(cfg);
    const auto [a, b] = two_remembered_pipes(map);
    map.update({make_track(5, LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE,
                           v(10, 0.35))},
               2.0);
    EXPECT_EQ(map.find(a)->live_track_id, 5) << "nearest, as before";
    (void)b;
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

TEST(RetainedLandmarks, SplitTrackOnAFollowedObjectIsNotANewLandmark) {
    RetainedLandmarks map(example_config());
    map.update({make_track(1, LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED, v(10, 0))}, 0.0);

    // The tracker opens a second track 0.3 m away while the first still lives.
    map.update({make_track(1, LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED, v(10, 0)),
                make_track(2, LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED, v(10.3, 0))},
               0.1);
    EXPECT_EQ(map.landmarks().size(), 1u);
    EXPECT_EQ(map.rejected_count(), 1);

    // A pipe 2 m away is a different pipe.
    map.update({make_track(1, LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED, v(10, 0)),
                make_track(3, LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED, v(10, 2))},
               0.2);
    EXPECT_EQ(map.landmarks().size(), 2u);
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

    // In the course frame x is odom +y and y (right) is odom -x. The gate is
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

TEST(RetainedLandmarks, ZLockPutsFloorAndSurfaceObjectsAtTheirDepth) {
    auto cfg = example_config();
    cfg.map_rules.z_lock.enable = true;
    cfg.map_rules.z_lock.floor_z = 3.4;
    cfg.map_rules.z_lock.surface_z = 0.1;
    cfg.map_rules.z_lock.floor_classes = {{LT::TABLE, 0}};
    cfg.map_rules.z_lock.surface_classes = {{LT::OCTAGON, 0}};
    RetainedLandmarks map(cfg);

    // Measured depths are off; the gate is not depth locked.
    map.update({make_track(1, LT::TABLE, LS::TABLE_WHOLE, v(30, 2, 2.9)),
                make_track(2, LT::OCTAGON, LS::OCTAGON_WHOLE, v(30, 2, 0.9)),
                make_track(3, LT::GATE, LS::GATE_WHOLE, v(10, 0, 1.7))},
               0.0);

    for (const auto& lm : map.landmarks()) {
        if (lm.key.type == LT::TABLE) {
            EXPECT_DOUBLE_EQ(lm.position.z(), 3.4);
        } else if (lm.key.type == LT::OCTAGON) {
            EXPECT_DOUBLE_EQ(lm.position.z(), 0.1);
        } else {
            EXPECT_DOUBLE_EQ(lm.position.z(), 1.7);
        }
    }
}

TEST(RetainedLandmarks, ZLockOffKeepsMeasuredDepth) {
    RetainedLandmarks map(example_config());
    map.update({make_track(1, LT::TABLE, LS::TABLE_WHOLE, v(30, 2, 2.9))}, 0.0);
    EXPECT_DOUBLE_EQ(map.landmarks()[0].position.z(), 2.9);
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
