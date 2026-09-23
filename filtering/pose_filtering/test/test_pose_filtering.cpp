#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <vortex/utils/types.hpp>
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

}  // namespace vortex::filtering
