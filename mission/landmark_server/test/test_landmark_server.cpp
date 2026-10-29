#include <gtest/gtest.h>

#include <cmath>

#include "landmark_server/detection_check.hpp"
#include "landmark_server/map.hpp"
#include "landmark_server/measure.hpp"

#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>
#include "landmark_server/targets.hpp"

namespace vortex::landmark_server {

namespace {

ClassConfig pipe_class(const std::string& name = "pipe") {
    ClassConfig c;
    c.name = name;
    c.group = "pipe";
    c.symmetry_deg = 360.0;
    return c;
}

Hit hit(const ClassConfig& cls, double x, double y, double t = 0.0) {
    Hit h;
    h.cls = &cls;
    h.position = Eigen::Vector3d(x, y, 1.0);
    h.t = t;
    return h;
}

Landmark panel(const std::string& name, double x, double y) {
    Landmark l;
    l.cls.name = name;
    l.position = Eigen::Vector3d(x, y, 1.0);
    return l;
}

}  // namespace

TEST(Cluster, NeighbouringObjectsAreNotChained) {
    // Two pipes 0.6 m apart with hits in between: single linkage would join
    // them, densest-first keeps two.
    std::vector<Eigen::Vector3d> pts;
    for (int i = 0; i < 10; ++i) {
        pts.emplace_back(0.0, 0.01 * i, 0.0);
        pts.emplace_back(0.6, 0.01 * i, 0.0);
    }
    pts.emplace_back(0.3, 0.0, 0.0);
    const auto clusters = cluster(pts, 0.25, 5);
    ASSERT_EQ(clusters.size(), 2u);
    EXPECT_EQ(clusters[0].size(), 10u);
}

TEST(Cluster, SmallClustersAreDropped) {
    std::vector<Eigen::Vector3d> pts(4, Eigen::Vector3d::Zero());
    EXPECT_TRUE(cluster(pts, 0.5, 5).empty());
}

TEST(SymmetricMeanYaw, WrapsAroundTheSymmetry) {
    // A 180 deg symmetric object seen at 89 and -89 deg is at 90 deg.
    const auto yaw = symmetric_mean_yaw({89.0 * M_PI / 180, -89.0 * M_PI / 180}, 180.0);
    ASSERT_TRUE(yaw);
    EXPECT_NEAR(std::abs(*yaw), M_PI / 2, 1e-9);
    EXPECT_FALSE(symmetric_mean_yaw({0.1}, 360.0));
    EXPECT_FALSE(symmetric_mean_yaw({}, 0.0));
}

TEST(Summarise, MedianIgnoresAnOutlierAndMajorityGivesTheClass) {
    const ClassConfig white = pipe_class("white");
    const ClassConfig red = pipe_class("red");
    std::vector<Hit> hits;
    for (int i = 0; i < 6; ++i) {
        hits.push_back(hit(white, 2.0, 0.0));
    }
    hits.push_back(hit(red, 2.4, 0.0));
    std::vector<const Hit*> in;
    for (const Hit& h : hits) {
        in.push_back(&h);
    }
    const Landmark l = summarise(in);
    EXPECT_EQ(l.cls.name, "white");
    EXPECT_DOUBLE_EQ(l.position.x(), 2.0);
    EXPECT_EQ(l.n_obs, 7);
    EXPECT_NEAR(l.spread_m, std::sqrt(0.16 / 7.0), 1e-9);
}

TEST(DetectionMap, IdsStayWhenTheClusterMovesALittle) {
    const ClassConfig c = pipe_class();
    DetectionMap map;
    for (int i = 0; i < 5; ++i) {
        map.add(hit(c, 3.0, 0.0), 100);
    }
    map.update(0.5, 5, 0.0, 0.0);
    ASSERT_EQ(map.landmarks().size(), 1u);
    const int id = map.landmarks()[0].id;
    for (int i = 0; i < 10; ++i) {
        map.add(hit(c, 3.2, 0.0), 100);
    }
    map.update(0.5, 5, 0.0, 0.0);
    ASSERT_EQ(map.landmarks().size(), 1u);
    EXPECT_EQ(map.landmarks()[0].id, id);
}

TEST(DetectionMap, OldHitsAreForgottenAndClassesCleared) {
    const ClassConfig c = pipe_class();
    DetectionMap map;
    for (int i = 0; i < 5; ++i) {
        map.add(hit(c, 3.0, 0.0, 0.0), 100);
    }
    map.update(0.5, 5, 10.0, 5.0);
    EXPECT_EQ(map.landmarks().size(), 1u);
    map.update(0.5, 5, 10.0, 20.0);
    EXPECT_TRUE(map.landmarks().empty());

    for (int i = 0; i < 5; ++i) {
        map.add(hit(c, 3.0, 0.0, 20.0), 100);
    }
    map.update(0.5, 5, 0.0, 20.0);
    map.clear_class("pipe");
    EXPECT_TRUE(map.landmarks().empty());
    map.update(0.5, 5, 0.0, 20.0);
    EXPECT_TRUE(map.landmarks().empty());
}

ClassConfig slalom_class(std::uint16_t subtype) {
    ClassConfig c = pipe_class(subtype == 2 ? "red" : "white");
    c.type = vortex_msgs::msg::LandmarkType::SLALOM_PIPE;
    c.subtype = subtype;
    return c;
}

void add_hits(std::vector<Hit>& out, const ClassConfig& c, double x, double y) {
    for (int i = 0; i < 6; ++i) {
        out.push_back(hit(c, x + 0.01 * i, y));
    }
}

TEST(Measure, ObjectWithoutYawFacesFromTheVehicle) {
    const ClassConfig c = pipe_class();
    std::vector<Hit> hits;
    add_hits(hits, c, 0.0, 3.0);
    const auto m = measure_object(hits, Pose::Identity(), MeasureSettings{});
    ASSERT_TRUE(m.ok);
    EXPECT_NEAR(yaw_of(m.pose.rotation()), M_PI / 2, 0.02);
    EXPECT_EQ(m.hits, 6);
}

TEST(Measure, TooFewHitsOrTooMuchSpreadFails) {
    const ClassConfig c = pipe_class();
    std::vector<Hit> hits{hit(c, 3.0, 0.0)};
    EXPECT_FALSE(measure_object(hits, Pose::Identity(), MeasureSettings{}).ok);
    hits.clear();
    add_hits(hits, c, 3.0, 0.0);
    hits.push_back(hit(c, 3.0, 0.45));
    MeasureSettings s;
    s.max_spread_m = 0.1;
    const auto m = measure_object(hits, Pose::Identity(), s);
    EXPECT_FALSE(m.ok);
    EXPECT_NEAR(m.spread_m, 0.17, 0.01);
}

TEST(Measure, SlalomRowIsTheNearestInFrontAcrossItsPipes) {
    using LS = vortex_msgs::msg::LandmarkSubtype;
    const ClassConfig red = slalom_class(LS::SLALOM_PIPE_RED);
    const ClassConfig white = slalom_class(LS::SLALOM_PIPE_WHITE);
    std::vector<Hit> hits;
    // Row 1 at x 3, slightly turned; row 2 at x 5; a red pipe behind.
    add_hits(hits, red, 3.0, 0.0);
    add_hits(hits, white, 3.1, -1.5);
    add_hits(hits, white, 2.9, 1.5);
    add_hits(hits, red, 5.0, 0.5);
    add_hits(hits, red, -2.0, 0.0);
    const auto m = measure_slalom_row(hits, Pose::Identity(), MeasureSettings{});
    ASSERT_TRUE(m.ok) << m.message;
    EXPECT_NEAR(m.pose.translation().x(), 3.025, 0.01);
    EXPECT_NEAR(yaw_of(m.pose.rotation()), std::atan2(0.2, 3.0), 0.01);
    EXPECT_EQ(m.hits, 18);
}

TEST(Measure, SlalomRowWithOnlyRedPointsFromTheVehicle) {
    using LS = vortex_msgs::msg::LandmarkSubtype;
    const ClassConfig red = slalom_class(LS::SLALOM_PIPE_RED);
    std::vector<Hit> hits;
    add_hits(hits, red, 3.0, 3.0);
    const auto m = measure_slalom_row(hits, Pose::Identity(), MeasureSettings{});
    ASSERT_TRUE(m.ok);
    EXPECT_NEAR(yaw_of(m.pose.rotation()), M_PI / 4, 0.02);
    hits.clear();
    add_hits(hits, red, -3.0, 0.0);
    EXPECT_FALSE(measure_slalom_row(hits, Pose::Identity(), MeasureSettings{}).ok);
}

TEST(GateFrames, TwoPanelsGiveTheGateDirection) {
    GateParams gate;
    gate.panel_classes = {"left", "right"};
    const auto frames = gate_frames(
        {panel("left", 4.0, -0.8), panel("right", 4.0, 0.8)}, gate,
        Eigen::Vector3d::Zero());
    ASSERT_EQ(frames.size(), 5u);
    EXPECT_EQ(frames[0].name, "gate_middle");
    EXPECT_NEAR(yaw_of(frames[0].pose.rotation()), 0.0, 1e-9);
    EXPECT_EQ(frames[1].name, "left_entrance");
    EXPECT_NEAR(frames[1].pose.translation().x(), 3.0, 1e-9);
    EXPECT_NEAR(frames[1].pose.translation().y(), -0.8, 1e-9);
}

TEST(GateFrames, OnePanelUsesTheLineFromTheStart) {
    GateParams gate;
    gate.panel_classes = {"left", "right"};
    const auto frames =
        gate_frames({panel("right", 3.0, 3.0)}, gate, Eigen::Vector3d::Zero());
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0].name, "right_entrance");
    EXPECT_NEAR(yaw_of(frames[0].pose.rotation()), M_PI / 4, 1e-9);
    EXPECT_NEAR(frames[0].pose.translation().x(), 3.0 - std::sqrt(0.5), 1e-9);
    EXPECT_EQ(frames[1].name, "right_exit");
    EXPECT_NEAR(frames[1].pose.translation().z(), 1.0 + gate.depth_below_panel_m,
                1e-9);
}

TEST(GateFrames, PanelsTooFarApartGiveNoGateMiddle) {
    GateParams gate;
    gate.panel_classes = {"left", "right"};
    const auto frames = gate_frames(
        {panel("left", 4.0, -3.0), panel("right", 4.0, 3.0)}, gate,
        Eigen::Vector3d::Zero());
    ASSERT_EQ(frames.size(), 4u);
    EXPECT_EQ(frames[0].name, "left_entrance");
}

geometry_msgs::msg::PoseWithCovariance detection(double qw, double qz) {
    geometry_msgs::msg::PoseWithCovariance p;
    p.pose.position.x = 2.0;
    p.pose.orientation.w = qw;
    p.pose.orientation.z = qz;
    return p;
}

TEST(DetectionCheck, AllZeroCovarianceMeansNoOrientation) {
    const auto d = check_detection(detection(0.0, 0.0));
    ASSERT_TRUE(d);
    EXPECT_FALSE(d->rotation);
    EXPECT_DOUBLE_EQ(d->position.x(), 2.0);
}

TEST(DetectionCheck, LargeOrientationVarianceMeansNoOrientation) {
    auto p = detection(1.0, 0.0);
    p.covariance[0] = 0.01;
    p.covariance[21] = p.covariance[28] = p.covariance[35] = 1000.0;
    EXPECT_FALSE(check_detection(p)->rotation);
}

TEST(DetectionCheck, NearUnitQuaternionIsNormalised) {
    auto p = detection(1.02, 0.0);
    p.covariance[35] = 0.01;
    const auto d = check_detection(p);
    ASSERT_TRUE(d && d->rotation);
    EXPECT_NEAR(d->rotation->norm(), 1.0, 1e-12);
}

TEST(DetectionCheck, BadQuaternionOrNanPositionIsDropped) {
    auto p = detection(0.0, 0.0);
    p.covariance[35] = 0.01;
    EXPECT_FALSE(check_detection(p));
    p = detection(1.0, 0.0);
    p.pose.position.y = std::nan("");
    EXPECT_FALSE(check_detection(p));
}

}  // namespace vortex::landmark_server
