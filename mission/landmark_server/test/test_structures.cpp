#include <gtest/gtest.h>
#include <cmath>
#include <stdexcept>
#include <vortex_msgs/msg/landmark_subtype.hpp>
#include <vortex_msgs/msg/landmark_type.hpp>
#include "landmark_server/class_config.hpp"
#include "landmark_server/retained_landmarks.hpp"
#include "landmark_server/structures.hpp"
#include "test_map_utils.hpp"

namespace vortex::mission {

using namespace test;

namespace {

using LT = vortex_msgs::msg::LandmarkType;
using LS = vortex_msgs::msg::LandmarkSubtype;

/// The slalom part of config/landmark_server_config.yaml.
LandmarkMapConfig slalom_config(bool count_only_in_structure = true) {
    return parse_map_config(YAML::Load(std::string(R"(
classes:
  SLALOM_PIPE:
    max_instances: {WHITE: 2, RED: 1}
    instance_gate_m: 0.7
    retain: forever
    count_only_in_structure: )") +
                                       (count_only_in_structure ? "true" : "false") +
                                       R"(
rules:
  adoption: {ambiguity_ratio: 2.0, wait_sec: 2.0}
  structures:
    slalom_set:
      max_instances: 3
      min_members: 2
      sigma: [0.2, 0.2, 0.3]
      members:
        white_left: {class: SLALOM_PIPE_WHITE, offset: [0.0, -1.52, 0.0]}
        red: {class: SLALOM_PIPE_RED, offset: [0.0, 0.0, 0.0]}
        white_right: {class: SLALOM_PIPE_WHITE, offset: [0.0, 1.52, 0.0]}
)"));
}

Eigen::Vector3d v(double x, double y, double z = 2.6) { return {x, y, z}; }

int count_type(const RetainedLandmarks& map, uint16_t subtype) {
    int n = 0;
    for (const auto& l : map.landmarks()) {
        n += l.key.type == LT::SLALOM_PIPE && l.key.subtype == subtype ? 1 : 0;
    }
    return n;
}

}  // namespace

TEST(Structures, TheSlalomTemplateParses) {
    const auto cfg = slalom_config();
    ASSERT_EQ(cfg.structures.size(), 1u);
    const auto& t = cfg.structures[0];
    EXPECT_EQ(t.name, "slalom_set");
    EXPECT_EQ(t.max_instances, 3);
    ASSERT_EQ(t.variants[0].members.size(), 3u);
    EXPECT_DOUBLE_EQ(t.variants[0].members[2].offset.y(), 1.52);
    EXPECT_TRUE(t.has_member_class({LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED}));
    EXPECT_FALSE(t.has_member_class({LT::GATE, LS::GATE_WHOLE}));
    EXPECT_TRUE(cfg.rule_for({LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED})
                    .count_only_in_structure);
}

TEST(Structures, MistakesAreRejectedWithTheKey) {
    EXPECT_THROW(parse_map_config(YAML::Load(R"(
rules:
  structures:
    s: {members: {a: {class: SLALOM_PIPE_PINK, offset: [0, 0, 0]}}}
)")),
                 std::runtime_error);
    EXPECT_THROW(parse_map_config(YAML::Load(R"(
rules:
  structures:
    s: {members: {a: {class: SLALOM_PIPE_RED, offset: [0, 0]},
                  b: {class: SLALOM_PIPE_WHITE}}}
)")),
                 std::runtime_error);
}

TEST(Structures, ASetFromItsPipes) {
    const auto cfg = slalom_config();
    std::vector<FitLandmark> lms = {
        {0, {LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE}, v(8.0, -1.4), {}},
        {1, {LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED}, v(8.1, 0.1), {}},
        {2, {LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE}, v(8.0, 1.65), {}},
        // the next set's white, 2 m on
        {3, {LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE}, v(10.0, -1.0), {}}};
    for (auto& l : lms) {
        l.covariance = Eigen::Matrix3d::Identity() * 0.01;
    }
    const auto fit = fit_structure(cfg.structures[0], lms);
    ASSERT_TRUE(fit);
    EXPECT_EQ(fit->members.size(), 3u);
    for (const auto& [m, id] : fit->members) {
        EXPECT_NE(id, 3);
    }
    EXPECT_LT((fit->pose.translation() - v(8.07, 0.1)).head<2>().norm(), 0.15);
}

TEST(Structures, ABadViewOfAMemberIsThatMemberNotANewPipe) {
    // A set seen well: white left and red. Then a new track 0.75 m from the
    // white: outside the 0.7 m take-over radius, so without the structure it
    // would be a second white pipe; inside half the distance to the red
    // (0.76 m), so the drawing says it is the white.
    for (const bool with_structure : {false, true}) {
        auto cfg = slalom_config();
        if (!with_structure) {
            cfg.structures.clear();
        }
        RetainedLandmarks map(cfg);
        map.update({make_track(1, LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE,
                               v(8.0, -1.52)),
                    make_track(2, LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED,
                               v(8.0, 0.0))},
                   0.0);
        map.update({}, 0.2);  // the tracker lost them; the set is fitted
        EXPECT_EQ(map.structures().instances().size(), with_structure ? 1u : 0u);
        map.update({make_track(3, LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE,
                               v(8.0, -2.27))},
                   0.4);
        EXPECT_EQ(count_type(map, LS::SLALOM_PIPE_WHITE), with_structure ? 1 : 2)
            << (with_structure ? "with" : "without") << " the structure";
    }
}

TEST(Structures, TheThirdPipeJoinsItsSet) {
    RetainedLandmarks map(slalom_config());
    map.update({make_track(1, LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE,
                           v(8.0, -1.52)),
                make_track(2, LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED, v(8.0, 0.0))},
               0.0);
    map.update({}, 0.2);
    map.update({make_track(3, LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE,
                           v(8.1, 1.7))},
               0.4);
    map.update({}, 0.6);
    ASSERT_EQ(map.structures().instances().size(), 1u);
    const auto& s = map.structures().instances()[0];
    EXPECT_EQ(std::count_if(s.members.begin(), s.members.end(),
                            [](int m) { return m >= 0; }),
              3);
}

TEST(Structures, AFalsePipeDoesNotTakeTheRealOnesPlace) {
    // Room for one red. A false red far from any set is mapped first; the
    // real red, between two whites, must still get in.
    for (const bool count_only : {false, true}) {
        RetainedLandmarks map(slalom_config(count_only));
        map.update({make_track(1, LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED,
                               v(18.0, 5.0, 3.2))},
                   0.0);
        map.update({make_track(2, LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE,
                               v(8.0, -1.52)),
                    make_track(3, LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE,
                               v(8.0, 1.52))},
                   0.2);
        map.update({}, 0.4);
        map.update({make_track(4, LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED,
                               v(8.0, 0.0))},
                   0.6);
        EXPECT_EQ(count_type(map, LS::SLALOM_PIPE_RED), count_only ? 2 : 1)
            << (count_only ? "with" : "without") << " count_only_in_structure";
    }
}

namespace {

LandmarkMapConfig overlap_config() {
    auto cfg = slalom_config();
    cfg.class_rules[{LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED}].max_instances = 5;
    cfg.class_rules[{LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE}].max_instances = 5;
    const auto parsed = parse_map_config(YAML::Load(R"(
rules:
  exclusive_groups:
    slalom_pipes: {classes: [SLALOM_PIPE_WHITE, SLALOM_PIPE_RED], distance_m: 0.7}
)"));
    cfg.exclusive_groups = parsed.exclusive_groups;
    return cfg;
}

}  // namespace

TEST(Structures, AnExclusiveGroupNeedsTwoClassesAndADistance) {
    EXPECT_THROW(parse_map_config(YAML::Load(R"(
rules:
  exclusive_groups:
    g: {classes: [SLALOM_PIPE_WHITE], distance_m: 0.7}
)")),
                 std::runtime_error);
    EXPECT_THROW(parse_map_config(YAML::Load(R"(
rules:
  exclusive_groups:
    g: {classes: [SLALOM_PIPE_WHITE, SLALOM_PIPE_RED]}
)")),
                 std::runtime_error);
}

TEST(Structures, InASetTheDrawingDecidesTheColour) {
    // A red "pipe" on top of the left white of a set (the white pipe called
    // red), seen more often than the white: the slot is white, the red goes,
    // and its track does not bring it back.
    RetainedLandmarks map(overlap_config());
    map.update({make_track(1, LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE,
                           v(8.0, -1.52)),
                make_track(2, LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED, v(8.0, 0.0))},
               0.0);
    map.update({}, 0.2);
    ASSERT_EQ(map.structures().instances().size(), 1u);
    for (int i = 0; i < 10; ++i) {
        map.update({make_track(3, LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED,
                               v(8.0, -1.45))},
                   0.4 + 0.2 * i);
    }
    EXPECT_EQ(count_type(map, LS::SLALOM_PIPE_WHITE), 1);
    EXPECT_EQ(count_type(map, LS::SLALOM_PIPE_RED), 1);
    EXPECT_GE(map.overlap_count(), 1);
    for (const auto& l : map.landmarks()) {
        if (l.key.subtype == LS::SLALOM_PIPE_RED) {
            EXPECT_NEAR(l.position.y(), 0.0, 1e-9);  // the set's red
        }
    }
}

TEST(Structures, OutsideASetTheOneSeenMoreOftenStays) {
    RetainedLandmarks map(overlap_config());
    for (int i = 0; i < 5; ++i) {
        map.update({make_track(1, LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE,
                               v(20.0, 5.0))},
                   0.2 * i);
    }
    map.update({make_track(1, LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE,
                           v(20.0, 5.0)),
                make_track(2, LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED,
                           v(20.0, 5.3))},
               1.0);
    EXPECT_EQ(count_type(map, LS::SLALOM_PIPE_WHITE), 1);
    EXPECT_EQ(count_type(map, LS::SLALOM_PIPE_RED), 0);
}

}  // namespace vortex::mission
