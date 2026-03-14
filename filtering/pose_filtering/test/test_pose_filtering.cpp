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
        cfg.nm.confirm_n = 3;
        cfg.nm.confirm_m = 5;
        cfg.nm.delete_n = 5;
        cfg.nm.delete_m = 7;
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
    cfg.nm.confirm_n = 3;
    cfg.nm.confirm_m = 5;
    cfg.nm.delete_n = 5;
    cfg.nm.delete_m = 7;
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
    cfg.nm.confirm_n = 3;
    cfg.nm.confirm_m = 5;
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
    cfg.nm.confirm_n = 1;
    cfg.nm.confirm_m = 1;
    cfg.nm.delete_n = 3;
    cfg.nm.delete_m = 3;
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

}  // namespace vortex::filtering
