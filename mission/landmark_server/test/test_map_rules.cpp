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

MapRulesConfig rules_config() {
    MapRulesConfig cfg;
    cfg.torpedo_version_1.fire = {0.0, 0.0, -0.169};
    cfg.torpedo_version_1.blood = {0.0, 0.0, -0.169};
    cfg.torpedo_version_1.firetruck = {0.0, 0.1888, 0.0};
    cfg.torpedo_version_1.ambulance = {0.0, -0.192, 0.0};
    cfg.torpedo_version_2.fire = {0.0, 0.0, -0.218};
    cfg.torpedo_version_2.blood = {0.0, 0.0, -0.168};
    cfg.torpedo_version_2.firetruck = {0.0, -0.1555, -0.15};
    cfg.torpedo_version_2.ambulance = {0.0, 0.193, 0.0};
    return cfg;
}

/// Scene helper: keeps a map and feeds it the same tracks every tick.
struct Scene {
    LandmarkMapConfig config = example_config();
    MapRulesConfig rules = rules_config();
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

// --- Gate ---------------------------------------------------------------

TEST(MapRulesGate, YawFromPanelsNormalTowardsTheVehicleAndLocked) {
    Scene s;
    s.run(gate_panels(), 5);

    const auto* gate = s.find(LT::GATE, LS::GATE_WHOLE);
    ASSERT_NE(gate, nullptr);
    EXPECT_TRUE(gate->has_orientation);
    // The vehicle is at the origin, so the front (+X) points towards -x.
    EXPECT_NEAR(std::abs(wrap(gate->yaw())), M_PI, 1e-6);
    EXPECT_TRUE(gate->yaw_locked);
    // The gate is at the midpoint of the panels.
    EXPECT_NEAR(gate->position.x(), 10.0, 1e-9);
    EXPECT_NEAR(gate->position.y(), 0.0, 1e-9);
}

TEST(MapRulesGate, LockedYawDoesNotFlipWhenSeenFromBehind) {
    Scene s;
    s.run(gate_panels(), 5);
    const auto* gate = s.find(LT::GATE, LS::GATE_WHOLE);
    ASSERT_NE(gate, nullptr);
    const double locked_yaw = gate->yaw();
    ASSERT_TRUE(gate->yaw_locked);

    // The vehicle passes through the gate and looks at it from behind.
    s.vehicle = v(20.0, 0.0);
    s.run(gate_panels(), 10);

    gate = s.find(LT::GATE, LS::GATE_WHOLE);
    EXPECT_NEAR(wrap(gate->yaw() - locked_yaw), 0.0, 1e-9);
}

TEST(MapRulesGate, YawDoesNotFlipBeforeLockEither) {
    Scene s;
    s.run(gate_panels(), 2);  // first estimates from the front
    const double first = s.find(LT::GATE, LS::GATE_WHOLE)->yaw();
    ASSERT_FALSE(s.find(LT::GATE, LS::GATE_WHOLE)->yaw_locked);

    s.vehicle = v(20.0, 0.0);  // now behind it
    s.run(gate_panels(), 6);
    EXPECT_NEAR(wrap(s.find(LT::GATE, LS::GATE_WHOLE)->yaw() - first), 0.0, 1e-9);
}

TEST(MapRulesGate, SyntheticGateWhenOnlyPanelsAreSeen) {
    Scene s;
    s.run(gate_panels(), 3);

    const auto* gate = s.find(LT::GATE, LS::GATE_WHOLE);
    ASSERT_NE(gate, nullptr);
    EXPECT_TRUE(gate->derived);
    const int id = gate->id;

    // Same landmark, same id, every tick.
    s.run(gate_panels(), 5);
    EXPECT_EQ(s.find(LT::GATE, LS::GATE_WHOLE)->id, id);
    int gates = 0;
    for (const auto& lm : s.map.landmarks()) {
        gates += (lm.key.type == LT::GATE && lm.key.subtype == LS::GATE_WHOLE);
    }
    EXPECT_EQ(gates, 1);
}

TEST(MapRulesGate, PanelsInheritTheYaw) {
    Scene s;
    s.run(gate_panels(), 6);
    const auto* gate = s.find(LT::GATE, LS::GATE_WHOLE);
    for (const uint16_t sub : {LS::GATE_SURVEY_REPAIR, LS::GATE_SEARCH_RESCUE}) {
        const auto* panel = s.find(LT::GATE, sub);
        ASSERT_NE(panel, nullptr);
        EXPECT_TRUE(panel->has_orientation);
        EXPECT_NEAR(wrap(panel->yaw() - gate->yaw()), 0.0, 1e-9);
    }
}

TEST(MapRulesGate, MeasuredGateWholeIsPulledToTheMidpointAndTakesOverFromSynthetic) {
    Scene s;
    s.run(gate_panels(), 3);
    const auto* synthetic = s.find(LT::GATE, LS::GATE_WHOLE);
    ASSERT_TRUE(synthetic->derived);
    const int synthetic_id = synthetic->id;

    // Perception now also sees the whole gate, 30 cm off (depth bias).
    auto tracks = gate_panels();
    tracks.push_back(make_track(3, LT::GATE, LS::GATE_WHOLE, v(10.3, 0.1), true, false));
    s.run(tracks, 4);

    const RetainedLandmark* measured = nullptr;
    for (const auto& lm : s.map.landmarks()) {
        if (lm.key.type == LT::GATE && lm.key.subtype == LS::GATE_WHOLE && !lm.derived) {
            measured = &lm;
        }
    }
    ASSERT_NE(measured, nullptr);
    EXPECT_NEAR(measured->position.x(), 10.0, 1e-9);  // pulled to the midpoint
    EXPECT_TRUE(measured->has_orientation);
    // The synthetic one is hidden behind it.
    for (const auto& lm : s.map.landmarks()) {
        if (lm.id == synthetic_id) {
            EXPECT_EQ(lm.absorbed_by, measured->id);
        }
    }
}

TEST(MapRulesGate, OnePanelGivesNoYaw) {
    Scene s;
    s.run({make_track(1, LT::GATE, LS::GATE_SURVEY_REPAIR, v(10, -0.78), true, false)}, 10);
    EXPECT_EQ(s.find(LT::GATE, LS::GATE_WHOLE), nullptr);
}

TEST(MapRulesGate, ConsistentEstimatesMoveTheCourseFrameToTheGate) {
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

TEST(MapRulesGate, FortyDegreesWrongStartValueGivesAWarning) {
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

// --- Torpedo board ----------------------------------------------------------

namespace {

struct Layout {
    Eigen::Vector3d fire, blood, firetruck, ambulance;  // in the board frame
};

// Version 1: fire above blood (z is down).
Layout version_1_layout() {
    return {{0.0, -0.15, -0.10}, {0.0, 0.10, 0.15}, {0.0, 0.15, -0.10}, {0.0, -0.10, 0.15}};
}

// Version 2: blood above fire.
Layout version_2_layout() {
    return {{0.0, -0.15, 0.15}, {0.0, 0.10, -0.10}, {0.0, 0.15, 0.15}, {0.0, -0.10, -0.10}};
}

/// Icon tracks for a board at @p board_pos whose front points along @p yaw.
std::vector<Track> icon_tracks(const Layout& l,
                               const Eigen::Vector3d& board_pos,
                               double yaw,
                               bool fire = true,
                               bool blood = true,
                               bool truck = true,
                               bool ambulance = true) {
    const Eigen::Quaterniond q = yaw_q(yaw);
    std::vector<Track> out;
    const auto add = [&](int id, uint16_t sub, const Eigen::Vector3d& local) {
        out.push_back(make_track(id, LT::TORPEDO_BOARD, sub, board_pos + q * local, true, false));
    };
    if (fire) add(1, LS::TORPEDO_ICON_FIRE, l.fire);
    if (blood) add(2, LS::TORPEDO_ICON_BLOOD, l.blood);
    if (truck) add(3, LS::TORPEDO_ICON_FIRETRUCK, l.firetruck);
    if (ambulance) add(4, LS::TORPEDO_ICON_AMBULANCE, l.ambulance);
    return out;
}

const RetainedLandmark* find_target(const Scene& s, uint16_t sub) {
    return s.find(LT::TORPEDO_BOARD, sub);
}

}  // namespace

TEST(MapRulesBoard, YawFromEachIconPair) {
    const Eigen::Vector3d board(20.0, 3.0, 2.0);
    // The vehicle is in front of the board (front points towards -x).
    const double yaw = M_PI;

    struct Case {
        const char* name;
        bool fire, blood, truck, ambulance;
    };
    const Case cases[] = {{"all four", true, true, true, true},
                          {"ambulance + firetruck", false, false, true, true},
                          {"fire + blood", true, true, false, false}};
    for (const auto& c : cases) {
        Scene s;
        s.vehicle = v(5.0, 3.0);
        s.run(icon_tracks(version_1_layout(), board, yaw, c.fire, c.blood, c.truck,
                          c.ambulance),
              6);
        const auto* b = s.find(LT::TORPEDO_BOARD, LS::TORPEDO_BOARD_WHOLE);
        ASSERT_NE(b, nullptr) << c.name;
        ASSERT_TRUE(b->has_orientation) << c.name;
        EXPECT_NEAR(std::abs(wrap(b->yaw())), M_PI, 1e-6) << c.name;
        EXPECT_TRUE(b->derived) << c.name;
        EXPECT_TRUE(b->yaw_locked) << c.name;
    }
}

TEST(MapRulesBoard, YawFusesBothIconPairs) {
    // The two pairs are off in opposite directions (+-6 deg): the fused yaw is
    // the average, which is closer to the truth than either pair alone.
    const Eigen::Vector3d board(20.0, 3.0, 2.0);
    const auto pair_tracks = [&](double yaw_error) {
        const Eigen::Quaterniond q = yaw_q(M_PI + yaw_error);
        return q;
    };
    const Eigen::Quaterniond q_vehicle_pair = pair_tracks(6.0 * kDeg);
    const Eigen::Quaterniond q_hazard_pair = pair_tracks(-6.0 * kDeg);
    const Layout l = version_1_layout();

    std::vector<Track> tracks = {
        make_track(1, LT::TORPEDO_BOARD, LS::TORPEDO_ICON_FIRE,
                   board + q_hazard_pair * l.fire, true, false),
        make_track(2, LT::TORPEDO_BOARD, LS::TORPEDO_ICON_BLOOD,
                   board + q_hazard_pair * l.blood, true, false),
        make_track(3, LT::TORPEDO_BOARD, LS::TORPEDO_ICON_FIRETRUCK,
                   board + q_vehicle_pair * l.firetruck, true, false),
        make_track(4, LT::TORPEDO_BOARD, LS::TORPEDO_ICON_AMBULANCE,
                   board + q_vehicle_pair * l.ambulance, true, false)};

    Scene s;
    s.vehicle = v(5.0, 3.0);
    s.tick(tracks);
    const auto* b = s.find(LT::TORPEDO_BOARD, LS::TORPEDO_BOARD_WHOLE);
    ASSERT_NE(b, nullptr);
    // Both pairs have their own error, the fused one is close to pi.
    EXPECT_LT(std::abs(wrap(b->yaw() - M_PI)), 2.0 * kDeg);
}

TEST(MapRulesBoard, BoardIsPulledToTheIconCentre) {
    Scene s;
    s.vehicle = v(5.0, 3.0);
    const auto icons = icon_tracks(version_1_layout(), {20.0, 3.0, 2.0}, M_PI);
    s.run(icons, 3);

    Eigen::Vector3d centre = Eigen::Vector3d::Zero();
    for (const auto& t : icons) {
        centre += t.nominal_state.pos;
    }
    centre /= 4.0;
    const auto* b = s.find(LT::TORPEDO_BOARD, LS::TORPEDO_BOARD_WHOLE);
    ASSERT_NE(b, nullptr);
    EXPECT_NEAR((b->position - centre).norm(), 0.0, 1e-9);
}

TEST(MapRulesBoard, VersionFromIconHeightsGivesTheRightOffsets) {
    const auto cfg = rules_config();
    const Eigen::Vector3d board(20.0, 3.0, 2.0);

    for (int version : {1, 2}) {
        Scene s;
        s.vehicle = v(5.0, 3.0);
        const Layout layout = version == 1 ? version_1_layout() : version_2_layout();
        const auto& offsets = version == 1 ? cfg.torpedo_version_1 : cfg.torpedo_version_2;
        const auto icons = icon_tracks(layout, board, M_PI);
        s.run(icons, 6);

        const Eigen::Quaterniond q = yaw_q(M_PI);
        const auto expect_target = [&](uint16_t sub, const Eigen::Vector3d& icon_pos,
                                       const Eigen::Vector3d& offset) {
            const auto* t = find_target(s, sub);
            ASSERT_NE(t, nullptr) << "version " << version << " sub " << sub;
            EXPECT_TRUE(t->derived);
            const Eigen::Vector3d expected = icon_pos + q * offset;
            EXPECT_NEAR((t->position - expected).norm(), 0.0, 1e-6)
                << "version " << version << " sub " << sub;
        };
        expect_target(LS::TORPEDO_TARGET_LARGE_SURVEY_REPAIR, icons[0].nominal_state.pos, offsets.fire);
        expect_target(LS::TORPEDO_TARGET_LARGE_SEARCH_RESCUE, icons[1].nominal_state.pos, offsets.blood);
        expect_target(LS::TORPEDO_TARGET_SMALL_SURVEY_REPAIR, icons[2].nominal_state.pos, offsets.firetruck);
        expect_target(LS::TORPEDO_TARGET_SMALL_SEARCH_RESCUE, icons[3].nominal_state.pos, offsets.ambulance);
    }
}

TEST(MapRulesBoard, OpeningIsRightWithTheBoardRotated30Degrees) {
    const auto cfg = rules_config();
    const double yaw = 30.0 * kDeg;
    const Eigen::Vector3d board(20.0, 3.0, 2.0);
    Scene s;
    // In front of the board along its normal.
    s.vehicle = board + yaw_q(yaw) * Eigen::Vector3d(4.0, 0.0, 0.0);
    const auto icons = icon_tracks(version_1_layout(), board, yaw);
    s.run(icons, 6);

    const auto* b = s.find(LT::TORPEDO_BOARD, LS::TORPEDO_BOARD_WHOLE);
    ASSERT_NE(b, nullptr);
    EXPECT_NEAR(wrap(b->yaw() - yaw), 0.0, 1e-6);

    // Small survey/repair opening = firetruck icon + offset in the board frame.
    const auto* t = find_target(s, LS::TORPEDO_TARGET_SMALL_SURVEY_REPAIR);
    ASSERT_NE(t, nullptr);
    const Eigen::Vector3d expected =
        icons[2].nominal_state.pos + yaw_q(yaw) * cfg.torpedo_version_1.firetruck;
    EXPECT_NEAR((t->position - expected).norm(), 0.0, 1e-6);
    // The openings have the orientation of the board.
    EXPECT_NEAR(wrap(t->yaw() - yaw), 0.0, 1e-6);
}

TEST(MapRulesBoard, OneIconIsNotEnough) {
    Scene s;
    s.run(icon_tracks(version_1_layout(), {20, 3, 2}, M_PI, true, false, false, false), 10);
    EXPECT_EQ(s.find(LT::TORPEDO_BOARD, LS::TORPEDO_BOARD_WHOLE), nullptr);
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

}  // namespace vortex::mission
