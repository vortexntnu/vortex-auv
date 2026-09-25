#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <vortex/utils/types.hpp>
#include "pose_filtering/lib/hungarian.hpp"
#include "pose_filtering/lib/pose_track_manager.hpp"

namespace vortex::filtering {

class PoseTrackManagerTests : public ::testing::Test {
   protected:
    TrackManagerConfig make_default_config() {
        TrackManagerConfig cfg{};
        cfg.default_class_config.dyn_std_dev = 0.1;
        cfg.default_class_config.sens_std_dev = 0.1;
        cfg.default_class_config.nm.confirm_n = 3;
        cfg.default_class_config.nm.confirm_m = 5;
        cfg.default_class_config.nm.delete_n = 5;
        cfg.default_class_config.nm.delete_m = 7;
        return cfg;
    }

    Landmark make_landmark(
        const Eigen::Vector3d& pos,
        const Eigen::Quaterniond& q = Eigen::Quaterniond::Identity()) {
        return Landmark{Pose::from_eigen(pos, q), LandmarkClassKey{0, 0}};
    }
};

TEST_F(PoseTrackManagerTests, creates_tracks_from_measurements) {
    PoseTrackManager mgr(make_default_config());

    std::vector<Landmark> measurements{
        make_landmark({0.0, 0.0, 0.0}),
        make_landmark({1.0, 0.0, 0.0}),
        make_landmark({0.0, 1.0, 0.0}),
    };

    mgr.step(measurements, 0.1);

    const auto& tracks = mgr.get_tracks();
    ASSERT_EQ(tracks.size(), 3);

    for (const auto& t : tracks) {
        EXPECT_FALSE(t.confirmed);
        EXPECT_EQ(t.hits(), 1);
        EXPECT_NEAR(t.nominal_state.ori.w(), 1.0, 1e-12);
    }
}

TEST_F(PoseTrackManagerTests, track_confirms_after_n_hits) {
    auto cfg = make_default_config();
    cfg.default_class_config.nm.confirm_n = 3;
    cfg.default_class_config.nm.confirm_m = 5;
    cfg.default_class_config.nm.delete_n = 5;
    cfg.default_class_config.nm.delete_m = 7;
    PoseTrackManager mgr(cfg);

    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

    for (int i = 0; i < 5; ++i) {
        std::vector<Landmark> z = {make_landmark({0, 0, 0}, q)};
        mgr.step(z, 0.1);
    }

    ASSERT_EQ(mgr.get_tracks().size(), 1);
    EXPECT_TRUE(mgr.get_tracks().front().confirmed);
}

TEST_F(PoseTrackManagerTests, track_does_not_confirm_without_enough_hits) {
    auto cfg = make_default_config();
    cfg.default_class_config.nm.confirm_n = 3;
    cfg.default_class_config.nm.confirm_m = 5;
    PoseTrackManager mgr(cfg);

    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

    // First step creates the track with 1 hit
    std::vector<Landmark> z = {make_landmark({0, 0, 0}, q)};
    mgr.step(z, 0.1);

    // Next step: no measurement (miss)
    z = {};
    mgr.step(z, 0.1);

    ASSERT_EQ(mgr.get_tracks().size(), 1);
    EXPECT_FALSE(mgr.get_tracks().front().confirmed);
}

TEST_F(PoseTrackManagerTests, track_deleted_after_n_misses) {
    auto cfg = make_default_config();
    cfg.default_class_config.nm.confirm_n = 1;
    cfg.default_class_config.nm.confirm_m = 1;
    cfg.default_class_config.nm.delete_n = 3;
    cfg.default_class_config.nm.delete_m = 3;
    PoseTrackManager mgr(cfg);

    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

    // Create track
    std::vector<Landmark> z = {make_landmark({0, 0, 0}, q)};
    mgr.step(z, 0.1);
    ASSERT_EQ(mgr.get_tracks().size(), 1);

    // 3 consecutive misses
    for (int i = 0; i < 3; ++i) {
        z = {};
        mgr.step(z, 0.1);
    }

    EXPECT_TRUE(mgr.get_tracks().empty());
}

TEST_F(PoseTrackManagerTests, hits_increase_with_measurements) {
    auto cfg = make_default_config();
    PoseTrackManager mgr(cfg);

    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

    std::vector<Landmark> z = {make_landmark({0, 0, 0}, q)};
    mgr.step(z, 0.1);

    int hits1 = mgr.get_tracks().front().hits();
    ASSERT_EQ(hits1, 1);

    z = {make_landmark({0, 0, 0}, q)};
    mgr.step(z, 0.1);

    int hits2 = mgr.get_tracks().front().hits();
    ASSERT_EQ(hits2, 2);
    ASSERT_GT(hits2, hits1);
}

TEST_F(PoseTrackManagerTests, noisy_measurements_move_the_track_less) {
    const auto shift_after = [&](double extra_variance) {
        PoseTrackManager mgr(make_default_config());
        std::vector<Landmark> first = {make_landmark({0.0, 0.0, 0.0})};
        mgr.step(first, 0.1);
        // A measurement 0.3 m away, inside the gate.
        Landmark m = make_landmark({0.3, 0.0, 0.0});
        m.extra_variance = extra_variance;
        std::vector<Landmark> second = {m};
        mgr.step(second, 0.1);
        return mgr.get_tracks().front().nominal_state.pos.x();
    };

    const double trusting = shift_after(0.0);
    const double distrusting = shift_after(4.0);
    EXPECT_GT(trusting, 0.0);
    EXPECT_LT(distrusting, trusting);
}

TEST_F(PoseTrackManagerTests, extra_variance_does_not_slow_orientation) {
    const Eigen::Quaterniond yawed(
        Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
    const auto yaw_after = [&](double extra_variance) {
        PoseTrackManager mgr(make_default_config());
        std::vector<Landmark> first = {make_landmark({0.0, 0.0, 0.0})};
        mgr.step(first, 0.1);
        Landmark m = make_landmark({0.0, 0.0, 0.0}, yawed);
        m.extra_variance = extra_variance;
        std::vector<Landmark> second = {m};
        mgr.step(second, 0.1);
        return mgr.get_tracks().front().nominal_state.ori.angularDistance(
            Eigen::Quaterniond::Identity());
    };
    EXPECT_NEAR(yaw_after(4.0), yaw_after(0.0), 1e-9);
}

TEST_F(PoseTrackManagerTests, far_detection_of_another_object_does_not_add_noise) {
    const auto shift_after = [&](double other_extra_variance) {
        PoseTrackManager mgr(make_default_config());
        std::vector<Landmark> first = {make_landmark({0.0, 0.0, 0.0})};
        mgr.step(first, 0.1);
        Landmark near = make_landmark({0.3, 0.0, 0.0});
        Landmark far = make_landmark({20.0, 0.0, 0.0});
        far.extra_variance = other_extra_variance;
        std::vector<Landmark> second = {near, far};
        mgr.step(second, 0.1);
        return mgr.get_tracks().front().nominal_state.pos.x();
    };
    EXPECT_NEAR(shift_after(4.0), shift_after(0.0), 1e-9);
}

TEST_F(PoseTrackManagerTests, measurement_without_orientation_keeps_yaw) {
    PoseTrackManager mgr(make_default_config());
    const Eigen::Quaterniond yawed(
        Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ()));

    // Track created from an oriented measurement.
    std::vector<Landmark> z = {make_landmark({1, 2, 3}, yawed)};
    mgr.step(z, 0.1);
    ASSERT_EQ(mgr.get_tracks().size(), 1);
    ASSERT_TRUE(mgr.get_tracks().front().has_orientation);

    // Position-only detections with a placeholder (identity) orientation.
    for (int i = 0; i < 20; ++i) {
        Landmark m = make_landmark({1.0, 2.0, 3.0});
        m.has_orientation = false;
        std::vector<Landmark> zz = {m};
        mgr.step(zz, 0.1);
    }

    ASSERT_EQ(mgr.get_tracks().size(), 1);
    const auto& t = mgr.get_tracks().front();
    EXPECT_TRUE(t.has_orientation);
    EXPECT_NEAR(t.nominal_state.ori.angularDistance(yawed), 0.0, 1e-9);
}

TEST_F(PoseTrackManagerTests, track_from_position_only_learns_orientation) {
    PoseTrackManager mgr(make_default_config());

    Landmark first = make_landmark({0, 0, 0});
    first.has_orientation = false;
    std::vector<Landmark> z = {first};
    mgr.step(z, 0.1);
    ASSERT_EQ(mgr.get_tracks().size(), 1);
    EXPECT_FALSE(mgr.get_tracks().front().has_orientation);

    // A later measurement with a large yaw is associated and adopted.
    const Eigen::Quaterniond yawed(
        Eigen::AngleAxisd(2.0, Eigen::Vector3d::UnitZ()));
    std::vector<Landmark> z2 = {make_landmark({0, 0, 0}, yawed)};
    mgr.step(z2, 0.1);

    ASSERT_EQ(mgr.get_tracks().size(), 1);
    const auto& t = mgr.get_tracks().front();
    EXPECT_TRUE(t.has_orientation);
    EXPECT_NEAR(t.nominal_state.ori.angularDistance(yawed), 0.0, 1e-9);
}

TEST(Hungarian, finds_the_optimal_assignment) {
    // Greedy (row by row) takes (0,0)=1 and then (1,1)=10: 11.
    // The optimum is (0,1)=2 and (1,0)=2: 4.
    Eigen::MatrixXd c(2, 2);
    c << 1.0, 2.0, 2.0, 10.0;
    const auto a = solve_assignment(c);
    EXPECT_EQ(a[0], 1);
    EXPECT_EQ(a[1], 0);
}

TEST(Hungarian, rectangular) {
    Eigen::MatrixXd c(2, 3);
    c << 5.0, 1.0, 9.0, 1.0, 5.0, 9.0;
    const auto a = solve_assignment(c);
    EXPECT_EQ(a[0], 1);
    EXPECT_EQ(a[1], 0);
}

TEST(Hungarian, gnn_leaves_forbidden_and_expensive_pairs_unpaired) {
    Eigen::MatrixXd cost(2, 2);
    cost << 1.0, 0.0, 0.0, 20.0;
    Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic> allowed(2, 2);
    allowed << true, false, false, true;
    // Pair (1,1) costs more than leaving both unpaired (9).
    const auto a = associate_gnn(cost, allowed, 9.0);
    EXPECT_EQ(a[0], 0);
    EXPECT_EQ(a[1], -1);
}

TEST_F(PoseTrackManagerTests, close_objects_keep_their_own_measurements) {
    // Two objects of the same class 0.4 m apart, both inside each other's
    // gate. The track seen most (processed first) used to take both
    // measurements and starve its neighbour.
    auto cfg = make_default_config();
    cfg.default_class_config.max_pos_error = 1.0;
    cfg.default_class_config.mahalanobis_threshold = 10.0;
    PoseTrackManager mgr(cfg);

    const Eigen::Vector3d a{0.0, 0.0, 0.0};
    const Eigen::Vector3d b{0.4, 0.0, 0.0};
    for (int i = 0; i < 8; ++i) {
        std::vector<Landmark> z = {make_landmark(a), make_landmark(b)};
        mgr.step(z, 0.1);
    }

    ASSERT_EQ(mgr.get_tracks().size(), 2);
    for (const auto& t : mgr.get_tracks()) {
        EXPECT_TRUE(t.confirmed);
        EXPECT_EQ(t.misses(), 0);
        const double to_a = (t.nominal_state.pos - a).norm();
        const double to_b = (t.nominal_state.pos - b).norm();
        EXPECT_LT(std::min(to_a, to_b), 1e-6);
    }
}

TEST_F(PoseTrackManagerTests, several_frames_in_one_cycle_are_one_hit) {
    PoseTrackManager mgr(make_default_config());
    for (int cycle = 0; cycle < 3; ++cycle) {
        for (int frame = 0; frame < 3; ++frame) {
            std::vector<Landmark> z = {make_landmark({1.0, 0.0, 0.0})};
            mgr.update(z, 0.03);
        }
        mgr.end_cycle();
    }
    // One track (the second frame did not start a new one) and one hit per
    // cycle.
    ASSERT_EQ(mgr.get_tracks().size(), 1);
    EXPECT_EQ(mgr.get_tracks().front().hits(), 3);
    EXPECT_EQ(mgr.get_tracks().front().misses(), 0);
}

TEST_F(PoseTrackManagerTests, position_only_does_not_shrink_orientation_cov) {
    PoseTrackManager mgr(make_default_config());
    std::vector<Landmark> z = {make_landmark({0, 0, 0})};
    mgr.step(z, 0.1);
    const double before =
        mgr.get_tracks().front().error_state.cov()(5, 5);

    Landmark m = make_landmark({0, 0, 0});
    m.has_orientation = false;
    std::vector<Landmark> zz = {m};
    mgr.step(zz, 0.1);
    // Only the prediction (process noise) acts on the orientation.
    EXPECT_GT(mgr.get_tracks().front().error_state.cov()(5, 5), before);
}

TEST_F(PoseTrackManagerTests, line_of_sight_noise_moves_less_along_the_ray) {
    // Depth (x) is uncertain, the side (y) is not: the same 0.3 m offset
    // moves the track less along x than along y.
    const auto moved = [&](const Eigen::Vector3d& offset) {
        PoseTrackManager mgr(make_default_config());
        std::vector<Landmark> z = {make_landmark({0, 0, 0})};
        mgr.step(z, 0.1);
        Landmark m = make_landmark(offset);
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        cov.diagonal() << 1.0, 0.001, 0.001;
        m.extra_position_cov = cov;
        std::vector<Landmark> zz = {m};
        mgr.step(zz, 0.1);
        return (mgr.get_tracks().front().nominal_state.pos).norm();
    };
    EXPECT_LT(moved({0.3, 0.0, 0.0}), moved({0.0, 0.3, 0.0}));
}

TEST_F(PoseTrackManagerTests, last_associations_name_the_track_of_each_hit) {
    auto cfg = make_default_config();
    cfg.default_class_config.mahalanobis_threshold = 10.0;
    PoseTrackManager mgr(cfg);
    std::vector<Landmark> z = {make_landmark({0, 0, 0}),
                               make_landmark({3, 0, 0})};
    mgr.update(z, 0.1);
    // Both started a track.
    ASSERT_EQ(mgr.last_associations().size(), 2);
    mgr.end_cycle();

    const auto id_at = [&](const Eigen::Vector3d& p) {
        for (const auto& t : mgr.get_tracks()) {
            if ((t.nominal_state.pos - p).norm() < 0.5) {
                return t.id;
            }
        }
        return -1;
    };
    // One hit on the far track, one clutter far from both: a new track.
    std::vector<Landmark> zz = {make_landmark({3.05, 0, 0}),
                                make_landmark({0, 8, 0})};
    mgr.update(zz, 0.1);
    const auto& assoc = mgr.last_associations();
    ASSERT_EQ(assoc.size(), 2);
    EXPECT_EQ(assoc[0].track_id, id_at({3, 0, 0}));
    EXPECT_NEAR(assoc[0].measurement.pose.x, 3.05, 1e-12);
    EXPECT_EQ(assoc[1].track_id, id_at({0, 8, 0}));
    EXPECT_NE(assoc[1].track_id, id_at({0, 0, 0}));
}

}  // namespace vortex::filtering
