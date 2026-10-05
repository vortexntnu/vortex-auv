#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include "landmark_server/class_config.hpp"
#include "landmark_server/landmark_graph.hpp"
#include "test_map_utils.hpp"

namespace vortex::mission {

using namespace test;

namespace {

/// Merge b into a (b wins), as ROS does with several parameter files.
void merge(YAML::Node a, const YAML::Node& b) {
    for (const auto& kv : b) {
        const auto key = kv.first.as<std::string>();
        if (kv.second.IsMap() && a[key] && a[key].IsMap()) {
            merge(a[key], kv.second);
        } else {
            a[key] = kv.second;
        }
    }
}

/// The parameters of the files the launch file loads for an environment.
YAML::Node load_env(const std::string& env) {
    const std::string dir = LANDMARK_SERVER_CONFIG_DIR;
    YAML::Node params(YAML::NodeType::Map);
    for (const std::string& file :
         {dir + "/landmark_server_config.yaml", dir + "/markers.yaml",
          dir + "/" + env + ".yaml", dir + "/course/templates.yaml",
          dir + "/course/" + env + ".yaml"}) {
        merge(params, YAML::LoadFile(file)["/**"]["ros__parameters"]);
    }
    return params;
}

}  // namespace

// The shipped files parse with the same code as at start: a removed or
// misspelt key fails here instead of on the vehicle.
TEST(ConfigFiles, SimulatorConfigParses) {
    const YAML::Node params = load_env("sim");
    LandmarkMapConfig cfg;
    ASSERT_NO_THROW(cfg = parse_map_config(params));
    LandmarkGraphConfig graph;
    ASSERT_NO_THROW(graph = parse_graph_config(params["graph"]));
    EXPECT_TRUE(graph.enable);  // off in the code: the config must turn it on
    EXPECT_TRUE(cfg.course.enable);
    EXPECT_EQ(cfg.course.tasks.size(), 8u);
    EXPECT_TRUE(cfg.map_rules.z_lock.enable);
    // Every slalom set has the slalom template's tolerances.
    for (const char* name : {"slalom_1", "slalom_2", "slalom_3"}) {
        const TaskSpec* t = cfg.course.task(name);
        ASSERT_NE(t, nullptr) << name;
        EXPECT_EQ(t->min_parts, 3) << name;
        EXPECT_TRUE(t->symmetric) << name;
        EXPECT_DOUBLE_EQ(t->max_range_m, 7.0) << name;
    }
    EXPECT_TRUE(cfg.rule_for({LT::TABLE, LS::TABLE_ITEM_PILL}).live_only);
}

TEST(ConfigFiles, PoolConfigParses) {
    const YAML::Node params = load_env("pool");
    LandmarkMapConfig cfg;
    ASSERT_NO_THROW(cfg = parse_map_config(params));
    LandmarkGraphConfig graph;
    ASSERT_NO_THROW(graph = parse_graph_config(params["graph"]));
    EXPECT_TRUE(graph.enable);
    EXPECT_EQ(cfg.course.tasks.size(), 8u);
}

TEST(ConfigFiles, CalibrationSessionTurnsTheGraphAndCourseOff) {
    YAML::Node params = load_env("pool");
    merge(params, YAML::LoadFile(std::string(LANDMARK_SERVER_CONFIG_DIR) +
                                 "/calibration.yaml")["/**"]["ros__parameters"]);
    LandmarkMapConfig cfg;
    ASSERT_NO_THROW(cfg = parse_map_config(params));
    EXPECT_FALSE(parse_graph_config(params["graph"]).enable);
    EXPECT_FALSE(cfg.course.enable);
    const auto& board = cfg.rule_for({LT::ARUCO_BOARD, LS::ARUCO_BOARD_CAMERA});
    EXPECT_EQ(board.max_instances, 1);
    EXPECT_TRUE(board.retain_forever);
    EXPECT_TRUE(cfg.rule_for({LT::ARUCO_MARKER, 28}).live_only);
}

}  // namespace vortex::mission
