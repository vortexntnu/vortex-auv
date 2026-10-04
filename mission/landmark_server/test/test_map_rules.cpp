#include <gtest/gtest.h>
#include <cmath>
#include "landmark_server/course_frame.hpp"
#include "landmark_server/map_rules.hpp"
#include "landmark_server/retained_landmarks.hpp"
#include "test_map_utils.hpp"

namespace vortex::mission {

using namespace test;

namespace {

constexpr double kDeg = M_PI / 180.0;

Eigen::Vector3d v(double x, double y, double z = 2.0) { return {x, y, z}; }

Eigen::Quaterniond yaw_q(double yaw) {
    return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

double wrap(double a) { return std::remainder(a, 2.0 * M_PI); }

/// Scene helper: keeps a map and feeds it the same tracks every tick.
struct Scene {
    LandmarkMapConfig config = example_config();
    MapRulesConfig rules;
    RetainedLandmarks map{config};
    Eigen::Vector3d vehicle{0.0, 0.0, 2.0};
    double now = 0.0;

    void tick(const std::vector<Track>& tracks, CourseFrameTracker* course = nullptr) {
        map.update(tracks, now);
        apply_map_rules(map, rules, vehicle, now, course);
        now += 0.1;
    }

    void run(const std::vector<Track>& tracks, int ticks,
             CourseFrameTracker* course = nullptr) {
        for (int i = 0; i < ticks; ++i) {
            tick(tracks, course);
        }
    }

    const RetainedLandmark* find(uint16_t type, uint16_t subtype) const {
        for (const auto& lm : map.landmarks()) {
            if (lm.key.type == type && lm.key.subtype == subtype) {
                return &lm;
            }
        }
        return nullptr;
    }
};

std::vector<Track> gate_panels(double x = 10.0) {
    return {make_track(1, LT::GATE, LS::GATE_SURVEY_REPAIR, v(x, -0.78), true, false),
            make_track(2, LT::GATE, LS::GATE_SEARCH_RESCUE, v(x, 0.78), true, false)};
}

}  // namespace

// --- Course frame from the gate panels ---------------------------------

TEST(MapRulesCourseFrame, ConsistentEstimatesMoveTheCourseFrameToTheGate) {
    Scene s;
    CourseFrameTracker course(s.config.course_frame);
    // Coarse start value: vehicle at the origin facing +x, course straight on.
    ASSERT_TRUE(course.set_coarse(vortex::utils::types::Pose::from_eigen(
                                      Eigen::Vector3d(0, 0, 0), yaw_q(0.0)),
                                  0.0)
                    .success);

    s.run(gate_panels(), 12, &course);

    EXPECT_EQ(course.status(), CourseFrameStatus::GATE_LOCKED);
    EXPECT_NEAR(course.origin().x(), 10.0, 1e-6);
    EXPECT_NEAR(course.origin().y(), 0.0, 1e-6);
    // Through the gate = away from the start = +x.
    EXPECT_NEAR(wrap(course.through_yaw()), 0.0, 1e-6);
    EXPECT_FALSE(course.take_deviation_warning());
}

TEST(MapRulesCourseFrame, FortyDegreesWrongStartValueGivesAWarning) {
    Scene s;
    CourseFrameTracker course(s.config.course_frame);
    ASSERT_TRUE(course.set_coarse(vortex::utils::types::Pose::from_eigen(
                                      Eigen::Vector3d(0, 0, 0), yaw_q(40.0 * kDeg)),
                                  0.0)
                    .success);
    s.run(gate_panels(), 12, &course);

    ASSERT_EQ(course.status(), CourseFrameStatus::GATE_LOCKED);
    EXPECT_NEAR(course.start_vs_gate_deviation_deg(), 40.0, 1e-4);
    EXPECT_TRUE(course.take_deviation_warning());
    // The gate wins.
    EXPECT_NEAR(wrap(course.through_yaw()), 0.0, 1e-6);
}

TEST(MapRulesCourseFrame, PanelsTooFarApartGiveNoEstimate) {
    Scene s;
    CourseFrameTracker course(s.config.course_frame);
    ASSERT_TRUE(course.set_coarse(vortex::utils::types::Pose::from_eigen(
                                      Eigen::Vector3d(0, 0, 0), yaw_q(0.0)),
                                  0.0)
                    .success);
    // A false panel 4 m from the other: not one gate.
    s.run({make_track(1, LT::GATE, LS::GATE_SURVEY_REPAIR, v(10, -2.0), true, false),
           make_track(2, LT::GATE, LS::GATE_SEARCH_RESCUE, v(10, 2.0), true, false)},
          12, &course);
    EXPECT_EQ(course.status(), CourseFrameStatus::COARSE);
}

TEST(MapRulesCourseFrame, OnePanelGivesNoEstimate) {
    Scene s;
    CourseFrameTracker course(s.config.course_frame);
    ASSERT_TRUE(course.set_coarse(vortex::utils::types::Pose::from_eigen(
                                      Eigen::Vector3d(0, 0, 0), yaw_q(0.0)),
                                  0.0)
                    .success);
    s.run({make_track(1, LT::GATE, LS::GATE_SURVEY_REPAIR, v(10, -0.78), true, false)}, 12,
          &course);
    EXPECT_EQ(course.status(), CourseFrameStatus::COARSE);
}

// --- Bins -------------------------------------------------------------------

TEST(MapRulesBins, RoleFromTheDownCameraIconGoesToTheNearestBin) {
    Scene s;
    s.run({make_track(1, LT::BIN, LS::BIN_UNCLASSIFIED, v(20.0, 1.0)),
           make_track(2, LT::BIN, LS::BIN_UNCLASSIFIED, v(20.0, 3.0)),
           make_track(3, LT::BIN, LS::BIN_SURVEY_REPAIR, v(20.05, 1.05, 2.4), true, false)},
          3);

    const RetainedLandmark* near_bin = nullptr;
    const RetainedLandmark* far_bin = nullptr;
    const RetainedLandmark* role_bin = nullptr;
    for (const auto& lm : s.map.landmarks()) {
        if (lm.key.subtype == LS::BIN_SURVEY_REPAIR) {
            role_bin = &lm;
        } else if (std::abs(lm.position.y() - 1.0) < 0.01) {
            near_bin = &lm;
        } else {
            far_bin = &lm;
        }
    }
    ASSERT_NE(role_bin, nullptr);
    ASSERT_NE(near_bin, nullptr);
    ASSERT_NE(far_bin, nullptr);
    // The bin next to the icon is the role bin; the other stays a bin.
    EXPECT_EQ(near_bin->absorbed_by, role_bin->id);
    EXPECT_EQ(far_bin->absorbed_by, -1);
}

TEST(MapRulesBins, RoleBinTooFarAwayIsNotMatched) {
    Scene s;
    s.run({make_track(1, LT::BIN, LS::BIN_UNCLASSIFIED, v(20.0, 1.0)),
           make_track(3, LT::BIN, LS::BIN_SEARCH_RESCUE, v(20.0, 3.0), true, false)},
          3);
    for (const auto& lm : s.map.landmarks()) {
        EXPECT_EQ(lm.absorbed_by, -1);
    }
}

// --- Octagon ----------------------------------------------------------------

TEST(MapRulesOctagon, OctagonFloatsAtTheSurfaceWhenZLockIsOn) {
    Scene s;
    s.rules.z_lock.enable = true;
    s.rules.z_lock.surface_z = 0.2;
    s.run({make_track(1, LT::TABLE, LS::TABLE_WHOLE, v(30.0, 2.0, 3.4), true, false)}, 3);

    const auto* octagon = s.find(LT::OCTAGON, LS::OCTAGON_WHOLE);
    ASSERT_NE(octagon, nullptr);
    EXPECT_NEAR(octagon->position.x(), 30.0, 1e-9);
    EXPECT_NEAR(octagon->position.y(), 2.0, 1e-9);
    EXPECT_NEAR(octagon->position.z(), 0.2, 1e-9);  // not the table's 3.4
    EXPECT_NEAR(s.find(LT::TABLE, LS::TABLE_WHOLE)->position.z(), 3.4, 1e-9);
}

TEST(MapRulesOctagon, OctagonAboveTheTable) {
    Scene s;
    s.run({make_track(1, LT::TABLE, LS::TABLE_WHOLE, v(30.0, 2.0, 3.0), true, false)}, 3);

    const auto* octagon = s.find(LT::OCTAGON, LS::OCTAGON_WHOLE);
    ASSERT_NE(octagon, nullptr);
    EXPECT_TRUE(octagon->derived);
    EXPECT_NEAR(octagon->position.x(), 30.0, 1e-9);
    EXPECT_NEAR(octagon->position.y(), 2.0, 1e-9);
    EXPECT_NEAR(octagon->position.z(), 3.0, 1e-9);

    // Stable id while the table is remembered, even after the tracker forgot it.
    const int id = octagon->id;
    s.run({}, 5);
    EXPECT_EQ(s.find(LT::OCTAGON, LS::OCTAGON_WHOLE)->id, id);
}

TEST(MapRulesOctagon, TableUnderTheOctagonWhenOnlyTheOctagonIsSeen) {
    Scene s;
    s.rules.z_lock.enable = true;
    s.rules.z_lock.floor_z = 3.4;
    s.rules.table_height_m = 0.7;
    s.run({make_track(1, LT::OCTAGON, LS::OCTAGON_WHOLE, v(30.0, 2.0, 0.5), true, false)}, 3);

    const auto* table = s.find(LT::TABLE, LS::TABLE_WHOLE);
    ASSERT_NE(table, nullptr);
    EXPECT_TRUE(table->derived);
    EXPECT_NEAR(table->position.x(), 30.0, 1e-9);
    EXPECT_NEAR(table->position.y(), 2.0, 1e-9);
    EXPECT_NEAR(table->position.z(), 2.7, 1e-9);
}

TEST(MapRulesOctagon, NoTableFromTheOctagonWithoutTheFloorDepth) {
    Scene s;
    s.run({make_track(1, LT::OCTAGON, LS::OCTAGON_WHOLE, v(30.0, 2.0, 0.5), true, false)}, 3);
    EXPECT_EQ(s.find(LT::TABLE, LS::TABLE_WHOLE), nullptr);
}

TEST(MapRulesOctagon, BothMeasuredShareTheXyOfThePrimary) {
    const std::vector<Track> both = {
        make_track(1, LT::TABLE, LS::TABLE_WHOLE, v(30.0, 2.0, 3.0), true, false),
        make_track(2, LT::OCTAGON, LS::OCTAGON_WHOLE, v(31.0, 3.0, 0.2), true, false)};
    struct Case {
        const char* primary;
        double x, y;
    };
    for (const Case& c : {Case{"table", 30.0, 2.0}, Case{"octagon", 31.0, 3.0},
                          Case{"midpoint", 30.5, 2.5}}) {
        Scene s;
        s.rules.table_octagon_primary = c.primary;
        s.run(both, 2);
        const auto* table = s.find(LT::TABLE, LS::TABLE_WHOLE);
        const auto* octagon = s.find(LT::OCTAGON, LS::OCTAGON_WHOLE);
        ASSERT_NE(table, nullptr);
        ASSERT_NE(octagon, nullptr);
        for (const auto* lm : {table, octagon}) {
            EXPECT_FALSE(lm->derived) << c.primary;
            EXPECT_NEAR(lm->position.x(), c.x, 1e-9) << c.primary;
            EXPECT_NEAR(lm->position.y(), c.y, 1e-9) << c.primary;
        }
        // Each keeps its own depth.
        EXPECT_NEAR(table->position.z(), 3.0, 1e-9) << c.primary;
        EXPECT_NEAR(octagon->position.z(), 0.2, 1e-9) << c.primary;
    }
}

}  // namespace vortex::mission
