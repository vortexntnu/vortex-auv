#include <gtest/gtest.h>

#include "landmark_server/association.hpp"
#include "landmark_server/targets.hpp"

namespace vortex::landmark_server {

namespace {

ClassConfig pipe_class() {
    ClassConfig c;
    c.name = "pipe";
    c.group = "pipe";
    c.symmetry_deg = 360.0;
    return c;
}

Measurement at(double x, double y) {
    Measurement z;
    z.position = gtsam::Point3(x, y, 0.0);
    return z;
}

// The vehicle drives along x and sees landmark 1 at (3, 0) from every
// keyframe, and landmark 2 at (3.1, 0) from the last two.
LandmarkGraph two_landmarks_at_one_place(const ClassConfig& cls) {
    LandmarkGraph g;
    g.reset(Params{}, gtsam::Pose3(), 0.0);
    g.add_landmark(1, cls, gtsam::Pose3({}, gtsam::Point3(3.0, 0.0, 0.0)), 0.0);
    g.add_observation(0, 1, at(3.0, 0.0), 0.0);
    g.update();
    for (int k = 1; k <= 4; ++k) {
        const double x = 0.5 * k;
        const int kf = g.add_keyframe(
            gtsam::Pose3({}, gtsam::Point3(x, 0.0, 0.0)), k);
        g.add_observation(kf, 1, at(3.0 - x, 0.0), k);
        if (k == 3) {
            g.add_landmark(2, cls,
                           gtsam::Pose3({}, gtsam::Point3(3.1, 0.0, 0.0)), k);
        }
        if (k >= 3) {
            g.add_observation(kf, 2, at(3.1 - x, 0.0), k);
        }
        g.update();
    }
    return g;
}

}  // namespace

TEST(Assignment, FindsTheCheapestPairing) {
    Eigen::MatrixXd cost(3, 3);
    cost << 4, 1, 3,
            2, 0, 5,
            3, 2, 2;
    // Greedy would take (1,1)=0 first and end at 0+4+2=6; the best is 5.
    EXPECT_EQ(solve_assignment(cost), (std::vector<int>{1, 0, 2}));
}

TEST(Assignment, MoreColumnsThanRows) {
    Eigen::MatrixXd cost(2, 3);
    cost << 9, 9, 1,
            9, 2, 1;
    EXPECT_EQ(solve_assignment(cost), (std::vector<int>{2, 1}));
}

TEST(Association, DetectionOfAKnownLandmarkIsMatchedNotNew) {
    const ClassConfig cls = pipe_class();
    LandmarkGraph g = two_landmarks_at_one_place(cls);
    g.retire(2);
    const int kf = g.last_keyframe();  // vehicle at x = 2

    const std::vector<Detection> detections{{&cls, at(1.02, 0.01)},
                                            {&cls, at(1.0, 3.0)}};
    const Association a = associate(g, kf, detections);

    ASSERT_EQ(a.matches.size(), 1u);
    EXPECT_EQ(a.matches[0].detection, 0u);
    EXPECT_EQ(a.matches[0].landmark, 1);
    EXPECT_EQ(a.unmatched, (std::vector<std::size_t>{1}));
}

TEST(Graph, DuplicateLandmarkIsRetired) {
    const ClassConfig cls = pipe_class();
    LandmarkGraph g = two_landmarks_at_one_place(cls);
    ASSERT_EQ(g.landmarks().size(), 2u);

    const auto retired = g.retire_duplicates(0.5, chi2_threshold(0.999, 3));

    // The less observed one goes.
    EXPECT_EQ(retired, (std::vector<std::pair<int, int>>{{1, 2}}));
    ASSERT_EQ(g.landmarks().size(), 1u);
    EXPECT_EQ(g.landmarks()[0].id, 1);
}

LandmarkState panel(const std::string& name, double x, double y) {
    LandmarkState l;
    l.cls.name = name;
    l.pose = gtsam::Pose3({}, gtsam::Point3(x, y, 1.0));
    return l;
}

TEST(GateFrames, TwoPanelsGiveTheGateDirection) {
    GateParams gate;
    gate.panel_classes = {"left", "right"};
    const auto frames = gate_frames(
        {panel("left", 4.0, -0.8), panel("right", 4.0, 0.8)}, gate, {});
    ASSERT_EQ(frames.size(), 5u);
    EXPECT_EQ(frames[0].name, "gate_middle");
    EXPECT_NEAR(frames[0].pose.rotation().yaw(), 0.0, 1e-9);
    EXPECT_EQ(frames[1].name, "left_entrance");
    EXPECT_NEAR(frames[1].pose.x(), 3.0, 1e-9);
    EXPECT_NEAR(frames[1].pose.y(), -0.8, 1e-9);
}

TEST(GateFrames, OnePanelUsesTheLineFromTheStart) {
    GateParams gate;
    gate.panel_classes = {"left", "right"};
    const auto frames = gate_frames({panel("right", 3.0, 3.0)}, gate, {});
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0].name, "right_entrance");
    EXPECT_NEAR(frames[0].pose.rotation().yaw(), M_PI / 4, 1e-9);
    EXPECT_NEAR(frames[0].pose.x(), 3.0 - std::sqrt(0.5), 1e-9);
    EXPECT_EQ(frames[1].name, "right_exit");
    EXPECT_NEAR(frames[1].pose.z(), 1.0 + gate.depth_below_panel_m, 1e-9);
}

TEST(GateFrames, PanelsTooFarApartGiveNoGateMiddle) {
    GateParams gate;
    gate.panel_classes = {"left", "right"};
    const auto frames = gate_frames(
        {panel("left", 4.0, -3.0), panel("right", 4.0, 3.0)}, gate, {});
    ASSERT_EQ(frames.size(), 4u);
    EXPECT_EQ(frames[0].name, "left_entrance");
}

}  // namespace vortex::landmark_server
