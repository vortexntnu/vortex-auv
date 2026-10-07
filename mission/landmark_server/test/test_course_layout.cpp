#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include "landmark_server/course_layout.hpp"
#include "landmark_server/course_model.hpp"

namespace vortex::mission {

namespace fs = std::filesystem;

namespace {

const char* kCourse = R"(
enable: true
start: [-4.0, 0.0]
templates:
  gate:
    members:
      whole: {class: GATE_WHOLE, offset: [0.0, 0.0, 0.0]}
      pole_left: {class: GATE_POLE_EDGE, offset: [0.0, 1.55, 0.0]}
  slalom_set:
    region_radius_m: 1.2
    members:
      white_left: {class: SLALOM_PIPE_WHITE, offset: [0.0, -1.52, 0.0]}
      red: {class: SLALOM_PIPE_RED, offset: [0.0, 0.0, 0.0]}
      white_right: {class: SLALOM_PIPE_WHITE, offset: [0.0, 1.52, 0.0]}
tasks:
  gate: {template: gate, prior: [0.0, 0.0, 180.0], min_parts: 1}
  slalom_1: {template: slalom_set, prior: [4.0, 0.0, 0.0], max_range_m: 7.0}
)";

CourseLayout layout_of(const CourseConfig& cfg) {
    CourseLayout l;
    l.enable = cfg.enable;
    l.start_x = cfg.start_xy.x();
    l.start_y = cfg.start_xy.y();
    for (const auto& t : cfg.tasks) {
        l.tasks.push_back({t.name, t.template_name, t.prior_xy.x(), t.prior_xy.y(),
                           t.prior_yaw * 180.0 / M_PI, t.region_radius_m});
    }
    return l;
}

/// A fresh directory shaped like the package source: <pkg>/CMakeLists.txt
/// and <pkg>/config/course.
fs::path package_dir(const std::string& name) {
    const fs::path pkg = fs::temp_directory_path() / ("landmark_layout_test_" + name);
    fs::remove_all(pkg);
    fs::create_directories(pkg / "config" / "course");
    std::ofstream(pkg / "CMakeLists.txt") << "\n";
    return pkg;
}

std::string read_text(const fs::path& file) {
    std::ifstream in(file);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

}  // namespace

TEST(CourseLayout, LayoutNames) {
    EXPECT_TRUE(valid_layout_name("robosub_A"));
    EXPECT_TRUE(valid_layout_name("pool-2"));
    EXPECT_FALSE(valid_layout_name(""));
    EXPECT_FALSE(valid_layout_name("templates"));
    EXPECT_FALSE(valid_layout_name("../sim"));
    EXPECT_FALSE(valid_layout_name("a b"));
}

TEST(CourseLayout, MovedTasksKeepTheirOwnTolerances) {
    const YAML::Node course = YAML::Load(kCourse);
    CourseLayout l = layout_of(parse_course_config(course));
    l.tasks.at(0).x = 0.5;
    l.tasks.at(1).y = 1.25;
    l.tasks.at(1).region_radius_m = 1.2;  // the template's: not written
    const YAML::Node out = apply_layout(course, l);

    const CourseConfig cfg = parse_course_config(out);
    EXPECT_NEAR(cfg.task("gate")->prior_xy.x(), 0.5, 1e-9);
    EXPECT_EQ(cfg.task("gate")->min_parts, 1);
    EXPECT_NEAR(cfg.task("slalom_1")->prior_xy.y(), 1.25, 1e-9);
    EXPECT_DOUBLE_EQ(cfg.task("slalom_1")->max_range_m, 7.0);
    EXPECT_FALSE(out["tasks"]["slalom_1"]["region_radius_m"]);

    l.tasks.at(1).region_radius_m = 2.0;  // its own now
    EXPECT_DOUBLE_EQ(apply_layout(course, l)["tasks"]["slalom_1"]["region_radius_m"].as<double>(),
                     2.0);
}

TEST(CourseLayout, AnotherPropStartsWithoutTheOldTolerances) {
    const YAML::Node course = YAML::Load(kCourse);
    CourseLayout l = layout_of(parse_course_config(course));
    l.tasks.at(1).template_name = "gate";
    const YAML::Node out = apply_layout(course, l);
    EXPECT_FALSE(out["tasks"]["slalom_1"]["max_range_m"]);
    EXPECT_NO_THROW(parse_course_config(out));
}

TEST(CourseLayout, TasksAreAddedAndRemoved) {
    const YAML::Node course = YAML::Load(kCourse);
    CourseLayout l = layout_of(parse_course_config(course));
    l.tasks.erase(l.tasks.begin());  // no gate
    l.tasks.push_back({"slalom_2", "slalom_set", 6.0, 0.5, 0.0, 0.0});
    const CourseConfig cfg = parse_course_config(apply_layout(course, l));
    EXPECT_EQ(cfg.task("gate"), nullptr);
    ASSERT_NE(cfg.task("slalom_2"), nullptr);
    EXPECT_DOUBLE_EQ(cfg.task("slalom_2")->region_radius_m, 1.2);
}

TEST(CourseLayout, MistakesAreRefused) {
    const YAML::Node course = YAML::Load(kCourse);
    const CourseLayout good = layout_of(parse_course_config(course));
    CourseLayout l = good;
    l.tasks.push_back(l.tasks.front());
    EXPECT_THROW(apply_layout(course, l), std::runtime_error);  // a name twice
    l = good;
    l.tasks.at(0).yaw_deg = std::nan("");
    EXPECT_THROW(apply_layout(course, l), std::runtime_error);
    l = good;
    l.tasks.at(0).name = "";
    EXPECT_THROW(apply_layout(course, l), std::runtime_error);
    l = good;
    l.tasks.at(0).template_name = "nope";  // checked by the course parser
    try {
        parse_course_config(apply_layout(course, l));
        ADD_FAILURE() << "an unknown template was taken";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("unknown template 'nope'"), std::string::npos)
            << e.what();
    }
}

TEST(CourseLayout, TheShippedSimulatorLayoutSurvivesARoundTrip) {
    const std::string dir = std::string(LANDMARK_SERVER_CONFIG_DIR) + "/course";
    const YAML::Node course = with_layout_file(
        read_layout_file(dir + "/templates.yaml").course,
        read_layout_file(dir + "/sim.yaml").course);
    const CourseConfig before = parse_course_config(course);
    const CourseConfig after = parse_course_config(apply_layout(course, layout_of(before)));
    ASSERT_EQ(after.tasks.size(), before.tasks.size());
    for (std::size_t i = 0; i < before.tasks.size(); ++i) {
        const auto& a = before.tasks[i];
        const auto& b = after.tasks[i];
        EXPECT_EQ(a.name, b.name);
        EXPECT_LT((a.prior_xy - b.prior_xy).norm(), 1e-3) << a.name;
        EXPECT_NEAR(a.prior_yaw, b.prior_yaw, 1e-3) << a.name;
        EXPECT_DOUBLE_EQ(a.region_radius_m, b.region_radius_m) << a.name;
        EXPECT_EQ(a.min_parts, b.min_parts) << a.name;
    }
}

TEST(CourseLayout, ASavedLayoutReadsBackAndTheOldOneIsKept) {
    const fs::path pkg = package_dir("save");
    const fs::path dir = pkg / "config" / "course";
    const fs::path file = dir / "robosub_A.yaml";
    const YAML::Node course = YAML::Load(kCourse);
    CourseLayout l = layout_of(parse_course_config(course));

    EXPECT_TRUE(write_layout_file(file, apply_layout(course, l), "zoom: 2", "20261007_120000")
                    .empty());  // nothing to back up
    l.tasks.at(1).x = 4.5;
    const fs::path backup =
        write_layout_file(file, apply_layout(course, l), "zoom: 3", "20261007_120500");
    EXPECT_EQ(backup, dir / "backup" / "robosub_A_20261007_120500.yaml");
    EXPECT_TRUE(fs::exists(backup));

    const LayoutFile saved = read_layout_file(file);
    EXPECT_EQ(saved.gui_state, "zoom: 3");
    const CourseConfig cfg = parse_course_config(with_layout_file(course, saved.course));
    EXPECT_NEAR(cfg.task("slalom_1")->prior_xy.x(), 4.5, 1e-9);
    EXPECT_EQ(cfg.task("gate")->min_parts, 1);
    // Only the layout went in the file, not the templates.
    EXPECT_FALSE(saved.course["templates"]);
    EXPECT_NEAR(parse_course_config(with_layout_file(course, read_layout_file(backup).course))
                    .task("slalom_1")
                    ->prior_xy.x(),
                4.0, 1e-9);

    // ROS refuses arrays that mix integers and doubles: every number in a
    // prior or the start has a decimal point.
    std::string text;
    {
        std::stringstream lines(read_text(file));
        for (std::string line; std::getline(lines, line);) {
            if (line.rfind('#', 0) != 0) {  // not the header comment
                text += line + "\n";
            }
        }
    }
    const std::regex array(R"((prior|start): \[([^\]]*)\])");
    int arrays = 0;
    for (auto it = std::sregex_iterator(text.begin(), text.end(), array);
         it != std::sregex_iterator(); ++it, ++arrays) {
        std::stringstream items((*it)[2].str());
        for (std::string item; std::getline(items, item, ',');) {
            EXPECT_NE(item.find('.'), std::string::npos) << (*it)[0];
        }
    }
    EXPECT_EQ(arrays, 3);  // start and two priors

    EXPECT_EQ(list_layouts(dir), std::vector<std::string>{"robosub_A"});
    EXPECT_TRUE(in_source_tree(dir));
    fs::remove(pkg / "CMakeLists.txt");
    EXPECT_FALSE(in_source_tree(dir));
    fs::remove_all(pkg);
}

TEST(CourseLayout, TemplatesIsNoLayout) {
    const fs::path pkg = package_dir("list");
    const fs::path dir = pkg / "config" / "course";
    std::ofstream(dir / "templates.yaml") << "\n";
    std::ofstream(dir / "pool.yaml") << "\n";
    std::ofstream(dir / "notes.txt") << "\n";
    EXPECT_EQ(list_layouts(dir), std::vector<std::string>{"pool"});
    fs::remove_all(pkg);
}

}  // namespace vortex::mission
