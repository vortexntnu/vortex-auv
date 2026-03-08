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
        EXPECT_NEAR(t.existence_probability, 0.5, 1e-12);
        EXPECT_NEAR(t.nominal_state.ori.w(), 1.0, 1e-12);
    }
}

TEST_F(PoseTrackManagerTests, angular_gate_filters_measurements) {
    auto cfg = make_default_config();
    cfg.default_class_config.max_ori_error = 0.2;  // strict gate
    PoseTrackManager mgr(cfg);

    Eigen::Quaterniond q_ref = Eigen::Quaterniond::Identity();

    Eigen::Quaterniond q_far(Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ()));

    std::vector<Landmark> measurements{
        make_landmark({0, 0, 1}, q_ref),
        make_landmark({0, 0, 1}, q_far),
    };

    mgr.step(measurements, 0.1);

    ASSERT_EQ(mgr.get_tracks().size(), 2);

    double t1_exist_prob_step1 = mgr.get_tracks().at(0).existence_probability;
    double t2_exist_prob_step1 = mgr.get_tracks().at(1).existence_probability;

    measurements = {
        make_landmark({0, 0.25, 1}, q_ref),
        make_landmark({0, 0.25, 1}, q_far),
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

    std::vector<Landmark> z = {make_landmark({0, 0, 0}, q)};

    PoseTrackManager mgr(cfg);

    mgr.step(z, 0.1);

    double exist_prob1 = mgr.get_tracks().front().existence_probability;

    ASSERT_EQ(mgr.get_tracks().size(), 1);
    ASSERT_EQ(exist_prob1, 0.5);

    z = {make_landmark({0, 0, 0}, q)};

    mgr.step(z, 0.1);

    double exist_prob2 = mgr.get_tracks().front().existence_probability;

    ASSERT_EQ(mgr.get_tracks().size(), 1);
    ASSERT_GT(exist_prob2, exist_prob1);

    z = {make_landmark({0, 0, 0}, q)};

    mgr.step(z, 0.1);

    double exist_prob3 = mgr.get_tracks().front().existence_probability;

    ASSERT_EQ(mgr.get_tracks().size(), 1);
    ASSERT_GT(exist_prob3, exist_prob2);
}

TEST_F(PoseTrackManagerTests, existence_decrease_on_no_detection) {
    auto cfg = make_default_config();

    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

    std::vector<Landmark> z = {make_landmark({0, 0, 0}, q)};

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
