#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "pose_filtering/lib/pose_track_manager.hpp"
#include "vortex/utils/types.hpp"

namespace vortex::filtering {
using vortex::utils::types::Pose;

class PoseTrackManagerTests : public ::testing::Test {
   protected:
    TrackManagerConfig make_default_config() {
        TrackManagerConfig cfg{};
        cfg.dyn_mod.std_dev = 0.1;
        cfg.sensor_mod.std_dev = 0.1;

        cfg.existence.initial_existence_probability = 0.5;
        cfg.existence.confirmation_threshold = 0.6;
        cfg.existence.deletion_threshold = 0.2;

        cfg.max_angle_gate_threshold = 0.5;  // radians

        cfg.ipda.pdaf.prob_of_detection = 0.5;
        cfg.ipda.pdaf.clutter_intensity = 0.01;
        cfg.ipda.pdaf.mahalanobis_threshold = 1.0;
        cfg.ipda.pdaf.min_gate_threshold = 0.0;
        cfg.ipda.pdaf.max_gate_threshold = 1.0;
        cfg.ipda.ipda.estimate_clutter = false;

        return cfg;
    }

    Pose make_pose(
        const Eigen::Vector3d& pos,
        const Eigen::Quaterniond& q = Eigen::Quaterniond::Identity()) {
        return Pose::from_eigen(pos, q);
    }
};

TEST_F(PoseTrackManagerTests, creates_tracks_from_measurements) {
    PoseTrackManager mgr(make_default_config());

    std::vector<Pose> measurements{
        make_pose({0.0, 0.0, 0.0}),
        make_pose({1.0, 0.0, 0.0}),
        make_pose({0.0, 1.0, 0.0}),
    };

    mgr.step(measurements, 0.1);

    const auto& tracks = mgr.get_tracks();
    ASSERT_EQ(tracks.size(), 3);

    for (const auto& t : tracks) {
        EXPECT_FALSE(t.confirmed);
        EXPECT_NEAR(t.existence_probability, 0.5, 1e-12);
        EXPECT_NEAR(t.current_orientation.w(), 1.0, 1e-12);
    }
}

TEST_F(PoseTrackManagerTests, angular_gate_filters_measurements) {
    auto cfg = make_default_config();
    cfg.max_angle_gate_threshold = 0.2;  // strict gate
    PoseTrackManager mgr(cfg);

    Eigen::Quaterniond q_ref = Eigen::Quaterniond::Identity();

    Eigen::Quaterniond q_far(Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ()));

    std::vector<Pose> measurements{
        make_pose({0, 0, 1}, q_ref),
        make_pose({0, 0, 1}, q_far),
    };

    mgr.step(measurements, 0.1);

    ASSERT_EQ(mgr.get_tracks().size(), 2);

    double t1_exist_prob_step1 = mgr.get_tracks().at(0).existence_probability;
    double t2_exist_prob_step1 = mgr.get_tracks().at(1).existence_probability;

    measurements = {
        make_pose({0, 0.5, 1}, q_ref),
        make_pose({0, 0.5, 1}, q_far),
    };

    mgr.step(measurements, 0.1);

    ASSERT_EQ(mgr.get_tracks().size(), 2);

    double t1_exist_prob_step2 = mgr.get_tracks().at(0).existence_probability;
    double t2_exist_prob_step2 = mgr.get_tracks().at(1).existence_probability;

    ASSERT_GT(t1_exist_prob_step2, t1_exist_prob_step1);
    ASSERT_GT(t2_exist_prob_step2, t2_exist_prob_step1);
}

TEST_F(PoseTrackManagerTests, single_target_existance_increase) {
    auto cfg = make_default_config();

    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

    std::vector<Pose> z = {make_pose({0, 0, 0}, q)};

    PoseTrackManager mgr(cfg);

    mgr.step(z, 0.1);

    double exist_prob1 = mgr.get_tracks().front().existence_probability;

    ASSERT_EQ(mgr.get_tracks().size(), 1);
    ASSERT_EQ(exist_prob1, 0.5);

    z = {make_pose({0, 0, 0}, q)};

    mgr.step(z, 0.1);

    double exist_prob2 = mgr.get_tracks().front().existence_probability;

    ASSERT_EQ(mgr.get_tracks().size(), 1);
    ASSERT_GT(exist_prob2, exist_prob1);

    z = {make_pose({0, 0, 0}, q)};

    mgr.step(z, 0.1);

    double exist_prob3 = mgr.get_tracks().front().existence_probability;

    ASSERT_EQ(mgr.get_tracks().size(), 1);
    ASSERT_GT(exist_prob3, exist_prob2);
}

TEST_F(PoseTrackManagerTests, existence_decrease_on_no_detection) {
    auto cfg = make_default_config();

    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

    std::vector<Pose> z = {make_pose({0, 0, 0}, q)};

    PoseTrackManager mgr(cfg);

    mgr.step(z, 0.1);

    double exist_prob1 = mgr.get_tracks().front().existence_probability;
    ASSERT_EQ(mgr.get_tracks().size(), 1);

    z = {};

    mgr.step(z, 0.1);

    const auto& tracks_after = mgr.get_tracks();
    if (tracks_after.empty()) {
        SUCCEED();
    } else {
        double exist_prob2 = tracks_after.front().existence_probability;
        ASSERT_LT(exist_prob2, exist_prob1);
    }
}

}  // namespace vortex::filtering
