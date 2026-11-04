#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <utility>
#include <utility>

#include "landmark_server/detection_check.hpp"
#include "landmark_server/graph.hpp"
#include "landmark_server/landmark.hpp"
#include "landmark_server/measure.hpp"

#include "landmark_server/association.hpp"

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

struct MapRun {
    Config cfg;
    Associator tracker;
    DriftGraph drift;
    // One object can not tell a heading error from a position error, two can.
    std::vector<Eigen::Vector3d> objects{{5.0, 2.0, 1.0}, {5.0, -2.0, 1.0}};
    double t{0.0};

    MapRun() {
        ClassConfig c = pipe_class();
        c.type = 1;
        c.subtype = 1;
        cfg.classes.push_back(c);
        tracker.reset(cfg);
        drift.reset(cfg.params.graph, Pose::Identity(), Pose::Identity(), t);
    }

    /// One step: odom yaw drifts with the distance driven. Detections land in
    /// odom through the same drift, as the real server gets them by TF.
    void step(const Eigen::Vector3d& truth,
              double travelled,
              double drift_per_m,
              Pose& T_odom_true) {
        T_odom_true = Pose::Identity();
        T_odom_true.linear() =
            Eigen::AngleAxisd(drift_per_m * travelled, Eigen::Vector3d::UnitZ())
                .matrix();
        Pose T_odom_base = T_odom_true;
        T_odom_base.translation() = T_odom_true * truth;
        std::vector<Hit> hits;
        for (const Eigen::Vector3d& object : objects) {
            if ((object - truth).norm() < 6.0) {
                Hit h = hit(cfg.classes[0], 0.0, 0.0, t);
                h.position = T_odom_true * object;
                hits.push_back(h);
            }
        }
        drift.update(T_odom_base, t, tracker.update(hits, t, drift));
        t += 0.5;
    }
};

TEST(Stack, ObjectSeenAgainIsTheSameTrackAndCorrectsTheDrift) {
    MapRun run;
    const double drift = 0.006;
    Pose T_odom_true = Pose::Identity();
    double travelled = 0.0;
    Eigen::Vector3d truth(0.0, 0.0, 1.0);
    const auto go = [&](const Eigen::Vector3d& to) {
        while ((to - truth).norm() > 0.1) {
            truth += 0.25 * (to - truth).normalized();
            travelled += 0.25;
            run.step(truth, travelled, drift, T_odom_true);
        }
        for (int i = 0; i < 6; ++i) {
            run.step(truth, travelled, drift, T_odom_true);
        }
    };
    go({1.0, 0.0, 1.0});
    ASSERT_EQ(run.tracker.landmarks().size(), 2u);
    const int id = run.tracker.landmarks()[0].id;

    // Out of view for long: the object stays.
    go({-15.0, 0.0, 1.0});
    ASSERT_EQ(run.tracker.landmarks().size(), 2u);

    go({1.0, 0.0, 1.0});
    ASSERT_EQ(run.tracker.landmarks().size(), 2u);
    EXPECT_EQ(run.tracker.landmarks()[0].id, id);

    for (int i = 0; i < 30; ++i) {
        run.step(truth, travelled, drift, T_odom_true);
    }
    // map -> odom takes the heading drift back out.
    const double odom_yaw = drift * travelled;
    const double map_yaw = yaw_of(run.drift.T_map_odom().rotation());
    EXPECT_GT(odom_yaw, 0.15);
    EXPECT_LT(std::abs(map_yaw + odom_yaw), 0.5 * odom_yaw);
    // The landmark is where odom sees it, not the graph's estimate.
    const Eigen::Vector3d in_odom = T_odom_true * run.objects[0];
    EXPECT_LT((run.tracker.landmarks()[0].position - in_odom).norm(), 0.1);
}

TEST(Stack, ClearedClassComesBackWhenSeenAgain) {
    MapRun run;
    Pose T = Pose::Identity();
    const Eigen::Vector3d truth(1.0, 0.0, 1.0);
    for (int i = 0; i < 6; ++i) {
        run.step(truth, 0.0, 0.0, T);
    }
    ASSERT_EQ(run.tracker.landmarks().size(), 2u);
    run.tracker.clear("pipe");
    EXPECT_TRUE(run.tracker.landmarks().empty());
    run.step(truth, 0.0, 0.0, T);
    EXPECT_EQ(run.tracker.landmarks().size(), 2u);
}

TEST(Stack, ClassesOfAGroupShareOneObjectAndTheMajorityWins) {
    MapRun run;
    run.objects = {{3.0, 0.0, 1.0}};
    ClassConfig red = run.cfg.classes[0];
    red.name = "red";
    red.subtype = 2;
    run.cfg.classes.push_back(red);
    Associator tracker;
    tracker.reset(run.cfg);
    for (int i = 0; i < 10; ++i) {
        Hit h = hit(i == 3 ? run.cfg.classes[0] : run.cfg.classes[1], 3.0, 0.0, 0.5 * i);
        run.drift.update(Pose::Identity(), 0.5 * i,
                         tracker.update({h}, 0.5 * i, run.drift));
    }
    ASSERT_EQ(tracker.landmarks().size(), 1u);
    EXPECT_EQ(tracker.landmarks()[0].cls.name, "red");
}

TEST(Association, AssignmentTakesTheLowestTotal) {
    Eigen::MatrixXd cost(2, 3);
    cost << 1.0, 2.0, 9.0,  //
        1.5, 9.0, 9.0;
    EXPECT_EQ(solve_assignment(cost), (std::vector<int>{1, 0}));
}

TEST(Association, TwoCloseObjectsStayTwo) {
    MapRun run;
    run.objects = {{3.0, 0.4, 1.0}, {3.0, -0.4, 1.0}};
    Pose T = Pose::Identity();
    const Eigen::Vector3d truth(0.0, 0.0, 1.0);
    for (int i = 0; i < 30; ++i) {
        run.step(truth, 0.0, 0.0, T);
    }
    ASSERT_EQ(run.tracker.landmarks().size(), 2u);
    EXPECT_EQ(run.tracker.landmarks()[0].n_obs, run.tracker.landmarks()[1].n_obs);
}

TEST(Association, AFewFalseDetectionsMakeNoObject) {
    MapRun run;
    run.tracker.update({hit(run.cfg.classes[0], 3.0, 3.0, 0.0)}, 0.0, run.drift);
    run.tracker.update({hit(run.cfg.classes[0], 3.0, 3.0, 2.0)}, 2.0, run.drift);
    run.tracker.update({hit(run.cfg.classes[0], 3.0, 3.0, 9.0)}, 9.0, run.drift);
    EXPECT_TRUE(run.tracker.landmarks().empty());
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
