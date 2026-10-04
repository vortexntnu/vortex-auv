#include <gtest/gtest.h>
#include <cmath>
#include "test_map_utils.hpp"

namespace vortex::mission {

using namespace test;

TEST(ClassConfig, ParsesExampleYaml) {
    const auto cfg = example_config();

    EXPECT_DOUBLE_EQ(cfg.plausibility_radius_m, 3.0);
    EXPECT_EQ(cfg.course_frame.gate_lock_consistent_estimates, 10);

    const auto& gate = cfg.rule_for({LT::GATE, LS::GATE_WHOLE});
    EXPECT_EQ(gate.max_instances, 1);
    EXPECT_TRUE(gate.retain_forever);

    // Per-subtype max_instances for pipes.
    EXPECT_EQ(cfg.rule_for({LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE}).max_instances, 10);
    EXPECT_EQ(cfg.rule_for({LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED}).max_instances, 5);
    const auto& pipe = cfg.rule_for({LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED});
    EXPECT_DOUBLE_EQ(pipe.instance_gate_m, 0.7);
    EXPECT_DOUBLE_EQ(pipe.retain_sec, 15.0);
    EXPECT_FALSE(pipe.retain_forever);
    EXPECT_EQ(pipe.keep_after_observations, 30);

    EXPECT_EQ(cfg.rule_for({LT::BIN, LS::BIN_UNCLASSIFIED}).max_instances, 4);
    EXPECT_DOUBLE_EQ(cfg.rule_for({LT::BIN, LS::BIN_UNCLASSIFIED}).instance_gate_m, 0.25);
}

TEST(ClassConfig, ParsesZLockAndDetectorNoise) {
    const auto cfg = parse_map_config(YAML::Load(R"(
detector_noise: {base_std_m: 0.05, along_std_per_m: 0.04, across_std_per_m: 0.002}
intake:
  measurement_covariance: {use: true, scale: 10.0, min_std_m: 0.05}
rules:
  adoption: {ambiguity_ratio: 3.0, wait_sec: 1.5}
  z_lock:
    enable: true
    floor_z: 3.4
    surface_z: 0.1
    floor_classes: [TABLE, BIN_STRUCTURE]
    surface_classes: [OCTAGON]
)"));
    EXPECT_DOUBLE_EQ(cfg.intake.noise.base_std_m, 0.05);
    EXPECT_DOUBLE_EQ(cfg.intake.noise.along_std_per_m, 0.04);
    EXPECT_DOUBLE_EQ(cfg.intake.noise.across_std_per_m, 0.002);
    EXPECT_TRUE(cfg.intake.use_measurement_covariance);
    EXPECT_DOUBLE_EQ(cfg.intake.covariance_scale, 10.0);
    EXPECT_DOUBLE_EQ(cfg.intake.covariance_min_std_m, 0.05);
    EXPECT_FALSE(parse_map_config(YAML::Node()).intake.use_measurement_covariance);
    EXPECT_DOUBLE_EQ(cfg.adoption_ambiguity_ratio, 3.0);
    EXPECT_DOUBLE_EQ(cfg.adoption_wait_sec, 1.5);
    EXPECT_DOUBLE_EQ(parse_map_config(YAML::Node()).adoption_ambiguity_ratio, 2.0);
    const auto& z = cfg.map_rules.z_lock;
    EXPECT_TRUE(z.enable);
    EXPECT_DOUBLE_EQ(z.floor_z, 3.4);
    EXPECT_DOUBLE_EQ(z.surface_z, 0.1);
    EXPECT_TRUE(z.is_floor({LT::TABLE, LS::TABLE_ITEM_PILL}));
    EXPECT_TRUE(z.is_floor({LT::BIN, LS::BIN_STRUCTURE}));
    EXPECT_FALSE(z.is_floor({LT::BIN, LS::BIN_UNCLASSIFIED}));
    EXPECT_TRUE(z.is_surface({LT::OCTAGON, LS::OCTAGON_WHOLE}));
    EXPECT_FALSE(z.is_surface({LT::GATE, LS::GATE_WHOLE}));
}

TEST(ClassConfig, ZLockCanSelectSingleSubtypes) {
    const auto cfg = parse_map_config(YAML::Load(R"(
rules:
  z_lock:
    enable: true
    surface_classes: [OCTAGON_WHOLE]
)"));
    const auto& z = cfg.map_rules.z_lock;
    EXPECT_TRUE(z.is_surface({LT::OCTAGON, LS::OCTAGON_WHOLE}));
    EXPECT_FALSE(z.is_surface({LT::OCTAGON, LS::OCTAGON_IMAGE_REPAIR}));
    EXPECT_FALSE(z.is_floor({LT::TABLE, LS::TABLE_ITEM_PILL}));
}

TEST(ClassConfig, ParsesMarkerBoxes) {
    const auto cfg = parse_map_config(YAML::Load(R"(
markers:
  boxes:
    GATE_WHOLE: {size: [0.08, 3.18, 1.36], offset: [0.0, 0.0, 0.48]}
    TABLE: {size: [0.6, 1.1, 0.8]}
)"));
    const auto* gate = cfg.marker_box_for({LT::GATE, LS::GATE_WHOLE});
    ASSERT_NE(gate, nullptr);
    EXPECT_DOUBLE_EQ(gate->size.y(), 3.18);
    EXPECT_DOUBLE_EQ(gate->offset.z(), 0.48);
    EXPECT_EQ(cfg.marker_box_for({LT::GATE, LS::GATE_POLE_EDGE}), nullptr);
    // A type entry covers every subtype; no offset means none.
    const auto* item = cfg.marker_box_for({LT::TABLE, LS::TABLE_ITEM_PILL});
    ASSERT_NE(item, nullptr);
    EXPECT_TRUE(item->offset.isZero());
    const auto pipes = parse_map_config(YAML::Load(R"(
markers:
  boxes:
    SLALOM_PIPE_RED: {size: [0.03, 0.03, 0.94], solid: true, color: [1.0, 0.0, 0.0]}
)"));
    const auto* red = pipes.marker_box_for({LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED});
    ASSERT_NE(red, nullptr);
    EXPECT_TRUE(red->solid);
    ASSERT_TRUE(red->color.has_value());
    EXPECT_DOUBLE_EQ(red->color->x(), 1.0);
    EXPECT_FALSE(gate->solid);
    EXPECT_FALSE(gate->color.has_value());
    EXPECT_THROW(parse_map_config(YAML::Load(R"(
markers:
  boxes:
    GATE_WHOLE: {size: [1.0, 2.0]}
)")),
                 std::runtime_error);
}

TEST(ClassConfig, ZLockIsOffByDefault) {
    const auto cfg = parse_map_config(YAML::Load("{}"));
    EXPECT_FALSE(cfg.map_rules.z_lock.enable);
}

TEST(ClassConfig, DetectorNoiseIsLessSureOfTheDepthThanTheBearing) {
    DetectorNoise noise;  // the defaults: 0.06 m + 0.03/m along, 0.003/m across
    ASSERT_TRUE(noise.enabled());
    const Eigen::Matrix3d cov = noise.covariance({5.0, 0.0, 0.0});
    EXPECT_NEAR(std::sqrt(cov(0, 0)), 0.06 + 0.03 * 5.0, 1e-9);   // along
    EXPECT_NEAR(std::sqrt(cov(1, 1)), 0.06 + 0.003 * 5.0, 1e-9);  // across
    EXPECT_NEAR(std::sqrt(cov(2, 2)), 0.06 + 0.003 * 5.0, 1e-9);
    EXPECT_NEAR(cov(0, 1), 0.0, 1e-12);
    EXPECT_FALSE((DetectorNoise{0.0, 0.0, 0.0}).enabled());
    EXPECT_THROW(parse_map_config(YAML::Load("detector_noise: {base_std_m: -0.1}")),
                 std::runtime_error);
}

TEST(ClassConfig, RemovedKeysAreAnErrorWithAHint) {
    // Old config files must not run with values that no longer do anything.
    for (const char* yaml :
         {"intake: {max_pipe_distance_m: 7.0}",
          "intake: {distance_noise: {variance_per_meter: 0.005}}",
          "course_frame: {lane: {before_gate: {x: [-10.0, 45.0]}}}",
          "rules: {board: {icon_radius_m: 1.0}}",
          "rules: {yaw_lock: {consistent_estimates: 5}}",
          "rules: {large_structures: [GATE]}",
          "rules: {torpedo_targets_from_icons: {}}",
          "classes: {SLALOM_PIPE: {min_distance_to_large_structures_m: 1.5}}"}) {
        EXPECT_THROW(parse_map_config(YAML::Load(yaml)), std::runtime_error) << yaml;
    }
}

TEST(ClassConfig, ClassNamesAreReadable) {
    EXPECT_EQ(class_name({LT::GATE, LS::GATE_WHOLE}), "GATE_WHOLE");
    EXPECT_EQ(class_name({LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED}), "SLALOM_PIPE_RED");
    EXPECT_EQ(class_name({LT::TORPEDO_BOARD, LS::TORPEDO_ICON_FIRE}), "TORPEDO_ICON_FIRE");
    EXPECT_EQ(class_name({LT::GATE, 0}), "GATE");
    EXPECT_EQ(class_name({LT::GATE, 99}), "GATE/99");
    EXPECT_EQ(class_name({77, 3}), "77/3");
}

TEST(ClassConfig, UnknownClassIsAnError) {
    EXPECT_THROW(parse_map_config(YAML::Load("classes: {NOT_A_CLASS: {max_instances: 1}}")),
                 std::runtime_error);
    EXPECT_THROW(parse_map_config(YAML::Load(
                     "classes: {SLALOM_PIPE: {max_instances: {PURPLE: 1}}}")),
                 std::runtime_error);
}

TEST(ClassConfig, ParsesTheRuleChecks) {
    const auto cfg = parse_map_config(YAML::Load(R"(
rules:
  max_panel_separation_m: 3.0
  table_octagon: {primary: midpoint, table_height_m: 0.67}
)"));
    EXPECT_DOUBLE_EQ(cfg.map_rules.max_panel_separation_m, 3.0);
    EXPECT_EQ(cfg.map_rules.table_octagon_primary, "midpoint");
    EXPECT_DOUBLE_EQ(cfg.map_rules.table_height_m, 0.67);

    EXPECT_THROW(parse_map_config(YAML::Load("rules: {table_octagon: {primary: floor}}")),
                 std::runtime_error);
}

TEST(ClassConfig, MissingKeysKeepDefaults) {
    const auto cfg = parse_map_config(YAML::Load("{}"));
    EXPECT_DOUBLE_EQ(cfg.map_rules.max_panel_separation_m, 2.5);
    EXPECT_DOUBLE_EQ(cfg.intake.noise.along_std_per_m, 0.03);
    EXPECT_EQ(cfg.rule_for({LT::GATE, 0}).max_instances, 20);
}

TEST(ClassConfig, PerClassTrackConfigOverridesDefault) {
    vortex::filtering::LandmarkClassConfig def;
    def.nm.confirm_n = 3;
    def.nm.confirm_m = 5;
    def.dyn_std_dev = 0.2;

    const auto out = parse_per_class_track_config(
        YAML::Load(R"(
default: {nm: {confirm_n: 3}}
SLALOM_PIPE: {nm: {confirm_n: 4, confirm_m: 6}}
BIN_STRUCTURE: {dyn_mod_std_dev: 0.5}
)"),
        def);

    const auto find = [&](uint16_t t, uint16_t s) {
        for (const auto& [k, c] : out) {
            if (k.type == t && k.subtype == s) {
                return c;
            }
        }
        ADD_FAILURE() << "no config for " << t << "/" << s;
        return def;
    };

    // A class without subtype covers all its subtypes.
    EXPECT_EQ(find(LT::SLALOM_PIPE, LS::SLALOM_PIPE_WHITE).nm.confirm_n, 4);
    EXPECT_EQ(find(LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED).nm.confirm_m, 6);
    EXPECT_DOUBLE_EQ(find(LT::SLALOM_PIPE, LS::SLALOM_PIPE_RED).dyn_std_dev, 0.2);
    // A subtype entry only covers that subtype.
    EXPECT_DOUBLE_EQ(find(LT::BIN, LS::BIN_STRUCTURE).dyn_std_dev, 0.5);
    EXPECT_EQ(find(LT::BIN, LS::BIN_STRUCTURE).nm.confirm_n, 3);
}

TEST(ClassConfig, EveryClassOfTheMessageIsKnownByName) {
    // The names come from vortex_msgs at build time: the ArUco board
    // subtypes were never written into a table here.
    const auto key = parse_class_name("ARUCO_BOARD_CAMERA");
    ASSERT_TRUE(key);
    EXPECT_EQ(key->first, vortex_msgs::msg::LandmarkType::ARUCO_BOARD);
    EXPECT_EQ(key->second, vortex_msgs::msg::LandmarkSubtype::ARUCO_BOARD_CAMERA);
    // Subtypes named after the first word of their type.
    const auto icon = parse_class_name("TORPEDO_ICON_FIRE");
    ASSERT_TRUE(icon);
    EXPECT_EQ(icon->first, vortex_msgs::msg::LandmarkType::TORPEDO_BOARD);
    EXPECT_EQ(parse_landmark_subtype(vortex_msgs::msg::LandmarkType::TORPEDO_BOARD, "ICON_FIRE"),
              vortex_msgs::msg::LandmarkSubtype::TORPEDO_ICON_FIRE);
}

}  // namespace vortex::mission
