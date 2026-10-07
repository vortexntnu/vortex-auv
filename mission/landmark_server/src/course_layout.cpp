#include "landmark_server/course_layout.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <regex>
#include <set>
#include <stdexcept>
#include "landmark_server/course_model.hpp"

namespace vortex::mission {

namespace fs = std::filesystem;

namespace {

/// The `course` keys a layout file holds; the rest is templates.yaml's.
const std::vector<std::string> kLayoutKeys = {"enable", "start", "tasks"};

/// A number as text with a decimal point, rounded to @p decimals, without
/// trailing zeros. ROS refuses a parameter array that mixes integers and
/// doubles, so 180 is written 180.0.
YAML::Node number(double value, int decimals) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.*f", decimals, value);
    std::string s(buf);
    while (s.size() > 2 && s.back() == '0' && s[s.size() - 2] != '.') {
        s.pop_back();
    }
    if (s == "-0.0") {
        s = "0.0";
    }
    return YAML::Node(s);
}

YAML::Node flow_sequence(std::initializer_list<YAML::Node> items) {
    YAML::Node seq(YAML::NodeType::Sequence);
    for (const auto& item : items) {
        seq.push_back(item);
    }
    seq.SetStyle(YAML::EmitterStyle::Flow);
    return seq;
}

/// Decimal numbers rewritten short: values that came through ROS parameters
/// read 1.2 as 1.1999999999999999. Integers and words stay as they are.
void tidy_numbers(YAML::Node node) {
    if (node.IsScalar()) {
        const auto text = node.Scalar();
        if (text.find('.') != std::string::npos) {
            try {
                node = number(node.as<double>(), 6);
            } catch (const YAML::Exception&) {
            }
        }
        return;
    }
    for (auto item : node) {
        tidy_numbers(node.IsMap() ? item.second : YAML::Node(item));
    }
}

void merge(YAML::Node into, const YAML::Node& from) {
    for (const auto& kv : from) {
        const auto key = kv.first.as<std::string>();
        if (kv.second.IsMap() && into[key] && into[key].IsMap()) {
            merge(into[key], kv.second);
        } else {
            into[key] = YAML::Clone(kv.second);
        }
    }
}

void check_finite(double value, const std::string& where) {
    if (!std::isfinite(value)) {
        throw std::runtime_error(where + " is not a finite number");
    }
}

}  // namespace

bool valid_layout_name(const std::string& name) {
    static const std::regex pattern("[A-Za-z0-9_-]+");
    return name != "templates" && std::regex_match(name, pattern);
}

std::vector<std::string> list_layouts(const fs::path& dir) {
    std::vector<std::string> names;
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(dir, ec)) {
        const auto& p = entry.path();
        if (entry.is_regular_file() && p.extension() == ".yaml" &&
            valid_layout_name(p.stem().string())) {
            names.push_back(p.stem().string());
        }
    }
    std::sort(names.begin(), names.end());
    return names;
}

bool in_source_tree(const fs::path& layout_dir) {
    std::error_code ec;
    return fs::exists(layout_dir.parent_path().parent_path() / "CMakeLists.txt",
                      ec);
}

LayoutFile read_layout_file(const fs::path& file) {
    YAML::Node doc;
    try {
        doc = YAML::LoadFile(file.string());
    } catch (const std::exception& e) {
        throw std::runtime_error("cannot read " + file.string() + ": " +
                                 e.what());
    }
    const YAML::Node params = doc["/**"]["ros__parameters"];
    if (!params || !params["course"] || !params["course"].IsMap()) {
        throw std::runtime_error(
            file.string() + " is not a course layout (no course parameters)");
    }
    LayoutFile out;
    out.course = YAML::Clone(params["course"]);
    if (params["course_gui_state"]) {
        out.gui_state = params["course_gui_state"].as<std::string>();
    }
    return out;
}

YAML::Node with_layout_file(const YAML::Node& base,
                            const YAML::Node& layout_course) {
    YAML::Node out = base && base.IsMap() ? YAML::Clone(base)
                                          : YAML::Node(YAML::NodeType::Map);
    for (const auto& key : kLayoutKeys) {
        out.remove(key);
    }
    if (layout_course && layout_course.IsMap()) {
        merge(out, layout_course);
    }
    return out;
}

double template_region_radius(const YAML::Node& course,
                              const std::string& template_name) {
    const YAML::Node templates = course["templates"];
    const YAML::Node tmpl = templates ? templates[template_name] : YAML::Node();
    // An unknown template is reported by the course parser.
    return tmpl && tmpl.IsMap() && tmpl["region_radius_m"]
               ? tmpl["region_radius_m"].as<double>()
               : TaskSpec{}.region_radius_m;
}

YAML::Node apply_layout(const YAML::Node& course, const CourseLayout& layout) {
    YAML::Node out = course && course.IsMap() ? YAML::Clone(course)
                                              : YAML::Node(YAML::NodeType::Map);
    check_finite(layout.start_x, "start x");
    check_finite(layout.start_y, "start y");
    out["enable"] = layout.enable;
    out["start"] =
        flow_sequence({number(layout.start_x, 3), number(layout.start_y, 3)});

    const YAML::Node old_tasks = course["tasks"];
    YAML::Node tasks(YAML::NodeType::Map);
    std::set<std::string> names;
    for (const auto& t : layout.tasks) {
        if (t.name.empty()) {
            throw std::runtime_error("a task has no name");
        }
        const std::string where = "course.tasks." + t.name;
        if (!names.insert(t.name).second) {
            throw std::runtime_error(where + ": the name is used twice");
        }
        if (t.template_name.empty()) {
            throw std::runtime_error(where + ": missing template");
        }
        check_finite(t.x, where + " x");
        check_finite(t.y, where + " y");
        check_finite(t.yaw_deg, where + " yaw");
        check_finite(t.region_radius_m, where + ".region_radius_m");

        // Its own tolerances stay, unless it is another prop now.
        const YAML::Node old = old_tasks ? old_tasks[t.name] : YAML::Node();
        YAML::Node n =
            old && old.IsMap() && old["template"] &&
                    old["template"].as<std::string>() == t.template_name
                ? YAML::Clone(old)
                : YAML::Node(YAML::NodeType::Map);
        tidy_numbers(n);
        n["template"] = t.template_name;
        n["prior"] = flow_sequence(
            {number(t.x, 3), number(t.y, 3), number(t.yaw_deg, 2)});
        const double tmpl_radius =
            template_region_radius(course, t.template_name);
        if (t.region_radius_m > 0.0 &&
            std::abs(t.region_radius_m - tmpl_radius) > 1e-6) {
            n["region_radius_m"] = number(t.region_radius_m, 2);
        } else {
            n.remove("region_radius_m");
        }
        n.SetStyle(YAML::EmitterStyle::Flow);
        tasks[t.name] = n;
    }
    out["tasks"] = tasks;
    return out;
}

fs::path write_layout_file(const fs::path& file,
                           const YAML::Node& course,
                           const std::string& gui_state,
                           const std::string& timestamp) {
    YAML::Node doc(YAML::NodeType::Map);
    fs::path backup;
    std::error_code ec;
    if (fs::exists(file, ec)) {
        try {
            doc = YAML::LoadFile(file.string());
        } catch (const std::exception& e) {
            throw std::runtime_error("cannot read " + file.string() + ": " +
                                     e.what());
        }
        backup = file.parent_path() / "backup" /
                 (file.stem().string() + "_" + timestamp + ".yaml");
        fs::create_directories(backup.parent_path(), ec);
        if (!fs::copy_file(file, backup, fs::copy_options::overwrite_existing,
                           ec)) {
            throw std::runtime_error("cannot back up " + file.string() +
                                     " to " + backup.string() + ": " +
                                     ec.message());
        }
    }
    if (!doc.IsMap()) {
        doc = YAML::Node(YAML::NodeType::Map);
    }

    YAML::Node params = doc["/**"]["ros__parameters"];
    if (!params["course"] || !params["course"].IsMap()) {
        params["course"] = YAML::Node(YAML::NodeType::Map);
    }
    YAML::Node c = params["course"];
    for (const auto& key : kLayoutKeys) {
        if (course[key]) {
            c[key] = YAML::Clone(course[key]);
        } else {
            c.remove(key);
        }
    }
    if (gui_state.empty()) {
        params.remove("course_gui_state");
    } else {
        params["course_gui_state"] = gui_state;
    }

    YAML::Emitter emitter;
    emitter.SetIndent(2);
    emitter << doc;
    if (!emitter.good()) {
        throw std::runtime_error("cannot write the layout: " +
                                 emitter.GetLastError());
    }
    const std::string header =
        "# Course layout " + file.stem().string() +
        ": where the tasks are (landmark_server).\n"
        "# Saved by landmark_server/set_course (" +
        timestamp +
        "); the version before is in backup/.\n"
        "# What the tasks look like: templates.yaml.\n"
        "#\n"
        "# Course frame: origin at the gate centre, x through the gate (away "
        "from\n"
        "# the start), y right, z down. start [x, y]: where the vehicle "
        "starts.\n"
        "# A task: {template, prior: [x, y, yaw_deg]}, yaw_deg where the front "
        "of\n"
        "# the prop faces, from x towards y; region_radius_m when not the\n"
        "# template's.\n";

    const fs::path tmp = file.string() + ".tmp";
    {
        std::ofstream out(tmp);
        out << header << emitter.c_str() << "\n";
        if (!out) {
            throw std::runtime_error("cannot write " + tmp.string());
        }
    }
    fs::rename(tmp, file, ec);
    if (ec) {
        throw std::runtime_error("cannot write " + file.string() + ": " +
                                 ec.message());
    }
    return backup;
}

}  // namespace vortex::mission
