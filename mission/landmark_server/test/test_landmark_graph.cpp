#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <numbers>
#include <vector>

#include "landmark_server/landmark_graph.hpp"

namespace vortex::mission {

namespace {

constexpr double kDeg = std::numbers::pi / 180.0;

Eigen::Isometry3d pose(double x, double y, double z, double yaw) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(x, y, z);
    T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).matrix();
    return T;
}

Eigen::Isometry3d yaw_only(double yaw) {
    return pose(0.0, 0.0, 0.0, yaw);
}

/**
 * A vehicle drives out along x and back. The odometry drifts in yaw
 * (drift_deg_per_m on every metre). A forward camera sees landmarks up to
 * 5 m ahead, within 60 degrees, and measures them exactly (in the odom frame,
 * as landmark_server gives them).
 */
struct Voyage {
    std::map<int, Eigen::Vector3d> world_landmarks;
    double drift_deg_per_m{0.0};
    double step_m{0.1};
    double length_m{20.0};
    /// Back to x = return_to_x (x = 4: the start landmark is still in view,
    /// so the drift after the last sighting does not count).
    double return_to_x{4.0};

    Eigen::Isometry3d world_T_body{pose(-3.0, 0.0, 2.0, 0.0)};
    Eigen::Isometry3d odom_T_body{pose(-3.0, 0.0, 2.0, 0.0)};
    double stamp{0.0};
    /// Last raw measurement per landmark (odom frame).
    std::map<int, Eigen::Vector3d> last_raw;

    void move(LandmarkGraph& graph, const Eigen::Isometry3d& rel) {
        world_T_body = world_T_body * rel;
        const double dist = rel.translation().norm();
        odom_T_body =
            odom_T_body * rel * yaw_only(drift_deg_per_m * dist * kDeg);
        stamp += 0.2;
        graph.add_odometry(stamp, odom_T_body);
        observe(graph);
        graph.optimize();
    }

    void observe(LandmarkGraph& graph) {
        for (const auto& [id, p_world] : world_landmarks) {
            const Eigen::Vector3d in_body = world_T_body.inverse() * p_world;
            const double range = in_body.norm();
            const double bearing = std::atan2(in_body.y(), in_body.x());
            if (in_body.x() <= 0.0 || range > 5.0 ||
                std::abs(bearing) > 60.0 * kDeg) {
                continue;
            }
            const Eigen::Vector3d p_odom = odom_T_body * in_body;
            graph.add_measurement(id, stamp, p_odom,
                                  Eigen::Matrix3d::Identity() * 0.01);
            last_raw[id] = p_odom;
        }
    }

    void out_and_back(LandmarkGraph& graph) {
        graph.add_odometry(stamp, odom_T_body);
        observe(graph);
        graph.optimize();
        const int steps = static_cast<int>((length_m + 3.0) / step_m);
        for (int i = 0; i < steps; ++i) {
            move(graph, pose(step_m, 0.0, 0.0, 0.0));
        }
        for (int i = 0; i < 18; ++i) {
            move(graph, yaw_only(10.0 * kDeg));
        }
        const int back = static_cast<int>((length_m - return_to_x) / step_m);
        for (int i = 0; i < back; ++i) {
            move(graph, pose(step_m, 0.0, 0.0, 0.0));
        }
    }

    /// Where the landmark should be in the current odom frame: at its true
    /// place relative to the vehicle, expressed with the drifted odometry.
    Eigen::Vector3d expected_in_odom(int id) const {
        return odom_T_body *
               (world_T_body.inverse() * world_landmarks.at(id));
    }
};

LandmarkGraphConfig test_config() {
    LandmarkGraphConfig c;
    c.enable = true;
    c.keyframe_interval_sec = 0.0;
    // The drift below is a bias (0.5 deg on every metre); a random-walk
    // model needs a larger std per step to allow it.
    c.odom_yaw_std_deg_per_m = 2.0;
    c.odom_yaw_std_deg_per_sec = 0.0;
    c.huber_k = 100.0;
    c.max_measurements_per_keyframe = 100;
    c.min_observations = 1;
    return c;
}

}  // namespace

TEST(LandmarkGraph, without_drift_the_graph_agrees_with_odometry) {
    LandmarkGraph graph(test_config());
    Voyage run;
    run.world_landmarks = {{0, {2.0, 2.0, 2.5}}, {1, {18.0, -2.0, 1.5}}};
    run.out_and_back(graph);

    for (const auto& [id, p] : run.world_landmarks) {
        const auto est = graph.landmark_in_odom(id);
        ASSERT_TRUE(est.has_value()) << id;
        EXPECT_LT((*est - run.expected_in_odom(id)).norm(), 1e-3) << id;
    }
    EXPECT_LT((graph.correction().translation()).norm(), 1e-3);
    EXPECT_GT(graph.keyframe_count(), 60u);
    // Without drift the smoothed trajectory is the odometry.
    const auto smoothed = graph.keyframes_in_odom();
    const auto raw = graph.keyframes_raw();
    ASSERT_EQ(smoothed.size(), graph.keyframe_count());
    ASSERT_EQ(raw.size(), smoothed.size());
    for (std::size_t i = 0; i < raw.size(); ++i) {
        EXPECT_LT((smoothed[i].translation() - raw[i].translation()).norm(),
                  1e-3);
    }
}

TEST(LandmarkGraph, seeing_the_start_again_corrects_the_far_end) {
    LandmarkGraph graph(test_config());
    Voyage run;
    run.drift_deg_per_m = 0.5;
    // A near the start (seen going out and coming back), B at the far end
    // (seen last on the way back, ~12 m of drift ago).
    run.world_landmarks = {{0, {2.0, 2.0, 2.5}}, {1, {19.0, -2.0, 1.5}}};
    run.out_and_back(graph);

    const double raw_err =
        (run.last_raw.at(1) - run.expected_in_odom(1)).norm();
    const auto est = graph.landmark_in_odom(1);
    ASSERT_TRUE(est.has_value());
    const double graph_err = (*est - run.expected_in_odom(1)).norm();

    // The drift is real (the test means something) and the graph removes
    // most of it.
    EXPECT_GT(raw_err, 0.5);
    EXPECT_LT(graph_err, 0.3 * raw_err)
        << "raw " << raw_err << " m, graph " << graph_err << " m";

    // The smoothed trajectory differs from the drifted odometry back in
    // time, and agrees with it at the newest keyframe.
    const auto smoothed = graph.keyframes_in_odom();
    const auto raw = graph.keyframes_raw();
    EXPECT_LT((smoothed.back().translation() - raw.back().translation()).norm(),
              1e-6);
    double max_diff = 0.0;
    for (std::size_t i = 0; i < raw.size(); ++i) {
        max_diff = std::max(
            max_diff, (smoothed[i].translation() - raw[i].translation()).norm());
    }
    EXPECT_GT(max_diff, 0.5);

    // A is being seen now: both are right.
    const auto a = graph.landmark_in_odom(0);
    ASSERT_TRUE(a.has_value());
    EXPECT_LT((*a - run.expected_in_odom(0)).norm(), 0.2);
}

TEST(LandmarkGraph, depth_and_attitude_stay_with_odometry) {
    LandmarkGraph graph(test_config());
    Voyage run;
    run.drift_deg_per_m = 0.5;
    run.world_landmarks = {{0, {2.0, 2.0, 2.5}}, {1, {19.0, -2.0, 1.5}}};
    run.out_and_back(graph);

    const auto kf = graph.latest_keyframe_estimate();
    ASSERT_TRUE(kf.has_value());
    EXPECT_NEAR(kf->translation().z(), 2.0, 0.05);
    const Eigen::Vector3d up = kf->rotation() * Eigen::Vector3d::UnitZ();
    EXPECT_GT(up.z(), std::cos(1.0 * kDeg));
}

TEST(LandmarkGraph, a_landmark_waits_for_min_observations) {
    auto cfg = test_config();
    cfg.min_observations = 3;
    LandmarkGraph graph(cfg);
    graph.add_odometry(0.0, pose(0, 0, 1, 0));
    const Eigen::Vector3d p{3.0, 0.0, 1.0};
    const Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * 0.01;

    EXPECT_TRUE(graph.add_measurement(7, 0.0, p, cov));
    EXPECT_TRUE(graph.add_measurement(7, 0.1, p, cov));
    graph.optimize();
    EXPECT_FALSE(graph.landmark_in_odom(7).has_value());

    EXPECT_TRUE(graph.add_measurement(7, 0.2, p, cov));
    graph.optimize();
    const auto est = graph.landmark_in_odom(7);
    ASSERT_TRUE(est.has_value());
    EXPECT_LT((*est - p).norm(), 1e-3);
    EXPECT_EQ(graph.observations(7), 3);
    EXPECT_EQ(graph.landmark_count(), 1u);
}

TEST(LandmarkGraph, measurements_per_keyframe_are_capped) {
    auto cfg = test_config();
    cfg.max_measurements_per_keyframe = 2;
    LandmarkGraph graph(cfg);
    const Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * 0.01;

    EXPECT_FALSE(graph.add_measurement(1, 0.0, {1, 0, 0}, cov))
        << "no keyframe yet";
    graph.add_odometry(0.0, pose(0, 0, 1, 0));
    EXPECT_TRUE(graph.add_measurement(1, 0.0, {1, 0, 1}, cov));
    EXPECT_TRUE(graph.add_measurement(1, 0.1, {1, 0, 1}, cov));
    EXPECT_FALSE(graph.add_measurement(1, 0.2, {1, 0, 1}, cov));
    // Another landmark has its own count.
    EXPECT_TRUE(graph.add_measurement(2, 0.2, {1, 1, 1}, cov));
    graph.optimize();
    EXPECT_EQ(graph.observations(1), 2);
}

TEST(LandmarkGraph, clear_starts_a_new_graph) {
    LandmarkGraph graph(test_config());
    graph.add_odometry(0.0, pose(0, 0, 1, 0));
    graph.add_measurement(1, 0.0, {1, 0, 1}, Eigen::Matrix3d::Identity());
    graph.optimize();
    graph.clear();
    EXPECT_EQ(graph.keyframe_count(), 0u);
    EXPECT_EQ(graph.landmark_count(), 0u);
    EXPECT_FALSE(graph.landmark_in_odom(1).has_value());

    // Works again, from a new start pose.
    graph.add_odometry(10.0, pose(5, 5, 1, 1.0));
    EXPECT_TRUE(graph.add_measurement(1, 10.0, {6, 5, 1},
                                      Eigen::Matrix3d::Identity() * 0.01));
    graph.optimize();
    const auto est = graph.landmark_in_odom(1);
    ASSERT_TRUE(est.has_value());
    EXPECT_LT((*est - Eigen::Vector3d(6, 5, 1)).norm(), 1e-3);
}

TEST(LandmarkGraph, config_is_read_from_yaml) {
    const YAML::Node node = YAML::Load(R"(
enable: true
keyframe: {distance_m: 1.0, angle_deg: 5.0, interval_sec: 0.0}
odom_noise: {pos_std_per_m: 0.05, yaw_std_deg_per_m: 1.0}
absolute: {depth_std_m: 0.1}
measurements: {huber_k: 3.0, max_per_keyframe: 4, min_observations: 2}
)");
    const auto c = parse_graph_config(node);
    EXPECT_TRUE(c.enable);
    EXPECT_DOUBLE_EQ(c.keyframe_distance_m, 1.0);
    EXPECT_DOUBLE_EQ(c.keyframe_angle_deg, 5.0);
    EXPECT_DOUBLE_EQ(c.keyframe_interval_sec, 0.0);
    EXPECT_DOUBLE_EQ(c.odom_pos_std_per_m, 0.05);
    EXPECT_DOUBLE_EQ(c.odom_yaw_std_deg_per_m, 1.0);
    EXPECT_DOUBLE_EQ(c.depth_std_m, 0.1);
    EXPECT_DOUBLE_EQ(c.huber_k, 3.0);
    EXPECT_EQ(c.max_measurements_per_keyframe, 4);
    EXPECT_EQ(c.min_observations, 2);
    // Untouched keys keep their defaults.
    EXPECT_DOUBLE_EQ(c.roll_pitch_std_deg, LandmarkGraphConfig{}.roll_pitch_std_deg);
    EXPECT_FALSE(parse_graph_config(YAML::Node()).enable);
}

}  // namespace vortex::mission
