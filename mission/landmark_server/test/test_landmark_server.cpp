#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <random>

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

// Recount every round: what cluster() must still give.
std::vector<std::vector<std::size_t>> cluster_by_recounting(
    const std::vector<Eigen::Vector3d>& p,
    double r,
    std::size_t min_size) {
    std::vector<bool> alive(p.size(), true);
    std::vector<std::vector<std::size_t>> out;
    while (true) {
        std::size_t best = 0, best_count = 0;
        for (std::size_t i = 0; i < p.size(); ++i) {
            std::size_t c = 0;
            for (std::size_t j = 0; j < p.size(); ++j) {
                c += alive[i] && alive[j] && (p[i] - p[j]).norm() <= r;
            }
            if (c > best_count) {
                best = i;
                best_count = c;
            }
        }
        if (best_count < min_size || best_count == 0) {
            return out;
        }
        std::vector<std::size_t> m;
        for (std::size_t j = 0; j < p.size(); ++j) {
            if (alive[j] && (p[best] - p[j]).norm() <= r) {
                m.push_back(j);
                alive[j] = false;
            }
        }
        out.push_back(m);
    }
}

TEST(Cluster, SameAsRecountingAndFast) {
    std::mt19937 rng(3);
    std::normal_distribution<double> noise(0.0, 0.3);
    std::uniform_real_distribution<double> spread(0.0, 6.0);
    std::vector<Eigen::Vector3d> pts;
    for (int k = 0; k < 9; ++k) {  // 9 objects and scattered false hits
        const Eigen::Vector3d c(spread(rng), spread(rng), 1.0);
        for (int i = 0; i < 80; ++i) {
            pts.emplace_back(c + Eigen::Vector3d(noise(rng), noise(rng), 0.0));
        }
    }
    for (int i = 0; i < 300; ++i) {
        pts.emplace_back(spread(rng), spread(rng), 1.0);
    }
    EXPECT_EQ(cluster(pts, 0.5, 5), cluster_by_recounting(pts, 0.5, 5));
    const auto t0 = std::chrono::steady_clock::now();
    cluster(pts, 0.5, 5);
    const double ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - t0)
                          .count();
    EXPECT_LT(ms, 50.0);  // 1020 hits, twice a second
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

TEST(Measure, WhitePipeOfTheNextRowDoesNotTurnTheRow) {
    using LS = vortex_msgs::msg::LandmarkSubtype;
    const ClassConfig red = slalom_class(LS::SLALOM_PIPE_RED);
    const ClassConfig white = slalom_class(LS::SLALOM_PIPE_WHITE);
    std::vector<Hit> hits;
    // Sim course row 0 seen from (6.6, -0.45): its red and left white, and
    // the left white of row 1 whose red is not seen.
    add_hits(hits, red, 8.10, 0.345);
    add_hits(hits, white, 8.14, -1.24);
    add_hits(hits, white, 10.15, -0.645);
    const Pose T = yaw_pose(0.0, Eigen::Vector3d(6.6, -0.45, 1.0));
    const auto m = measure_slalom_row(hits, T, MeasureSettings{});
    ASSERT_TRUE(m.ok) << m.message;
    EXPECT_NEAR(yaw_of(m.pose.rotation()), 0.0, 0.05);
}

TEST(Measure, SlalomRowWithOnlyRedKeepsTheVehiclesHeading) {
    using LS = vortex_msgs::msg::LandmarkSubtype;
    const ClassConfig red = slalom_class(LS::SLALOM_PIPE_RED);
    std::vector<Hit> hits;
    add_hits(hits, red, 1.0, 0.75);  // 1 m before the left gap
    const auto m = measure_slalom_row(hits, Pose::Identity(), MeasureSettings{});
    ASSERT_TRUE(m.ok);
    EXPECT_NEAR(yaw_of(m.pose.rotation()), 0.0, 0.02);
    hits.clear();
    add_hits(hits, red, -3.0, 0.0);
    EXPECT_FALSE(measure_slalom_row(hits, Pose::Identity(), MeasureSettings{}).ok);
}

// Three rows 2 m apart turned 20 deg, red pipes shifted sideways, whites
// 1.5 m each side, like the sim course. Row k's red at (along, lateral).
std::vector<Eigen::Vector2d> slalom_reds(double th) {
    const Eigen::Vector2d n(std::cos(th), std::sin(th)), u(-std::sin(th), std::cos(th));
    const Eigen::Vector2d c0(6.0, 0.7);
    return {c0, c0 + 2.0 * n + 0.7 * u, c0 + 4.0 * n - 0.3 * u};
}

void add_row(std::vector<Hit>& hits, const ClassConfig& red, const ClassConfig& white,
             const Eigen::Vector2d& r, double th, bool whites = true) {
    const Eigen::Vector2d u(-std::sin(th), std::cos(th));
    add_hits(hits, red, r.x(), r.y());
    if (whites) {
        add_hits(hits, white, (r - 1.5 * u).x(), (r - 1.5 * u).y());
        add_hits(hits, white, (r + 1.5 * u).x(), (r + 1.5 * u).y());
    }
}

TEST(Measure, SlalomGivesEveryRowFromTheFirstInFront) {
    using LS = vortex_msgs::msg::LandmarkSubtype;
    const ClassConfig red = slalom_class(LS::SLALOM_PIPE_RED);
    const ClassConfig white = slalom_class(LS::SLALOM_PIPE_WHITE);
    const double th = 20.0 * M_PI / 180.0;
    const auto reds = slalom_reds(th);
    std::vector<Hit> hits;
    for (const auto& r : reds) {
        add_row(hits, red, white, r, th);
    }
    const Pose T = yaw_pose(0.0, Eigen::Vector3d(3.0, 0.5, 1.0));
    const auto m = measure_slalom(hits, T, MeasureSettings{});
    ASSERT_TRUE(m.ok) << m.message;
    ASSERT_EQ(m.rows.size(), 3u);
    EXPECT_EQ(m.row_seen, std::vector<bool>({true, true, true}));
    for (int k = 0; k < 3; ++k) {
        EXPECT_LT((m.rows[k].translation().head<2>() - reds[k]).norm(), 0.05) << k;
        EXPECT_NEAR(yaw_of(m.rows[k].rotation()), th, 0.03) << k;
    }
    // Past the first row, the second is the first in front.
    const auto past = measure_slalom(hits, yaw_pose(th, Eigen::Vector3d(7.0, 1.4, 1.0)),
                                     MeasureSettings{});
    ASSERT_TRUE(past.ok);
    EXPECT_LT((past.rows[0].translation().head<2>() - reds[1]).norm(), 0.05);
}

TEST(Measure, SlalomRowNotSeenFollowsTheOthers) {
    using LS = vortex_msgs::msg::LandmarkSubtype;
    const ClassConfig red = slalom_class(LS::SLALOM_PIPE_RED);
    const ClassConfig white = slalom_class(LS::SLALOM_PIPE_WHITE);
    const double th = 20.0 * M_PI / 180.0;
    const auto reds = slalom_reds(th);
    std::vector<Hit> hits;
    add_row(hits, red, white, reds[0], th);
    add_row(hits, red, white, reds[1], th, false);  // only its red
    const auto m = measure_slalom(hits, yaw_pose(0.0, Eigen::Vector3d(3.0, 0.5, 1.0)),
                                  MeasureSettings{});
    ASSERT_TRUE(m.ok) << m.message;
    EXPECT_LT((m.rows[1].translation().head<2>() - reds[1]).norm(), 0.05);
    EXPECT_EQ(m.row_seen, std::vector<bool>({true, true, false}));
    // Row 2 unseen: 2 m on, at row 1's side shift; the real one is 1 m off.
    EXPECT_LT((m.rows[2].translation().head<2>() - reds[2]).norm(), 1.05);
}

TEST(Measure, SlalomFailsOnlyWhenItIsNotEvenRows) {
    using LS = vortex_msgs::msg::LandmarkSubtype;
    const ClassConfig red = slalom_class(LS::SLALOM_PIPE_RED);
    const ClassConfig white = slalom_class(LS::SLALOM_PIPE_WHITE);
    const double th = 20.0 * M_PI / 180.0;
    const Eigen::Vector2d n(std::cos(th), std::sin(th));
    const Pose T = yaw_pose(0.0, Eigen::Vector3d(3.0, 0.5, 1.0));
    auto reds = slalom_reds(th);
    auto measure = [&](const std::vector<Eigen::Vector2d>& r, double turn_last) {
        std::vector<Hit> hits;
        for (std::size_t k = 0; k < r.size(); ++k) {
            add_row(hits, red, white, r[k], th + (k + 1 == r.size() ? turn_last : 0.0));
        }
        return measure_slalom(hits, T, MeasureSettings{});
    };
    std::mt19937 rng(5);
    std::normal_distribution<double> noise(0.0, 0.1);
    auto noisy = reds;
    for (auto& r : noisy) {
        r += Eigen::Vector2d(noise(rng), noise(rng));
    }
    EXPECT_TRUE(measure(noisy, 0.0).ok) << measure(noisy, 0.0).message;
    std::vector<Hit> with_false;  // one false pipe between the rows
    for (const auto& r : reds) {
        add_row(with_false, red, white, r, th);
    }
    const Eigen::Vector2d between = reds[0] + 1.0 * n;
    add_hits(with_false, white, between.x(), between.y());
    for (int i = 0; i < 6; ++i) {  // and a far pipe too noisy to use
        with_false.push_back(hit(white, 14.0 + 0.4 * (i % 2), 3.0 + 0.4 * (i / 2 % 2)));
    }
    const auto f = measure_slalom(with_false, T, MeasureSettings{});
    ASSERT_TRUE(f.ok) << f.message;
    EXPECT_LT((f.rows[1].translation().head<2>() - reds[1]).norm(), 0.05);
    auto uneven = reds;
    uneven[2] += 1.2 * n;  // 2 m, then 3.2 m
    EXPECT_FALSE(measure(uneven, 0.0).ok);
    // A turned last row still gives every row near its red pipe; measured
    // again in front of it, that row's own pipes give its heading.
    const auto turned = measure(reds, 25.0 * M_PI / 180.0);
    ASSERT_TRUE(turned.ok) << turned.message;
    for (int k = 0; k < 3; ++k) {
        EXPECT_LT((turned.rows[k].translation().head<2>() - reds[k]).norm(), 0.4) << k;
    }
    std::vector<Hit> two_reds;
    add_row(two_reds, red, white, reds[0], th);
    add_hits(two_reds, red, reds[0].x() - 0.8 * std::sin(th), reds[0].y() + 0.8 * std::cos(th));
    const auto m = measure_slalom(two_reds, T, MeasureSettings{});
    EXPECT_FALSE(m.ok);
    EXPECT_NE(m.message.find("red pipes in one row"), std::string::npos) << m.message;
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
