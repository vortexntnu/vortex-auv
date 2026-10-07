#include <spdlog/spdlog.h>
#include <algorithm>
#include <cmath>
#include <ctime>
#include <regex>
#include "landmark_server/landmark_server_ros.hpp"

// Course layouts: one file per pool or competition course in the directory
// of templates.yaml. get_course hands one to the operator GUI, set_course
// puts a new one in use (and saves it). The same tasks with new priors keep
// the map; another task list, template or enable starts a new map.

namespace vortex::mission {

namespace {

namespace fs = std::filesystem;

const std::vector<std::string> kLayoutKeys = {"enable", "start", "tasks"};

std::string timestamp_now() {
    const std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_r(&now, &local);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &local);
    return buf;
}

rclcpp::ParameterValue scalar_value(const std::string& text) {
    if (text == "true" || text == "false") {
        return rclcpp::ParameterValue(text == "true");
    }
    static const std::regex integer("[-+]?[0-9]+");
    if (std::regex_match(text, integer)) {
        return rclcpp::ParameterValue(static_cast<int64_t>(std::stoll(text)));
    }
    char* end = nullptr;
    const double d = std::strtod(text.c_str(), &end);
    if (!text.empty() && end == text.c_str() + text.size()) {
        return rclcpp::ParameterValue(d);
    }
    return rclcpp::ParameterValue(text);
}

/// A YAML value as ROS would load it from a parameter file.
rclcpp::ParameterValue yaml_to_parameter_value(const YAML::Node& node) {
    if (!node.IsSequence()) {
        return scalar_value(node.Scalar());
    }
    std::vector<rclcpp::ParameterValue> items;
    for (const auto& item : node) {
        items.push_back(scalar_value(item.Scalar()));
    }
    const auto all = [&](rclcpp::ParameterType type) {
        return std::all_of(items.begin(), items.end(),
                           [&](const auto& v) { return v.get_type() == type; });
    };
    if (!items.empty() && all(rclcpp::ParameterType::PARAMETER_BOOL)) {
        std::vector<bool> out;
        for (const auto& v : items) {
            out.push_back(v.get<bool>());
        }
        return rclcpp::ParameterValue(out);
    }
    if (!items.empty() && all(rclcpp::ParameterType::PARAMETER_INTEGER)) {
        std::vector<int64_t> out;
        for (const auto& v : items) {
            out.push_back(v.get<int64_t>());
        }
        return rclcpp::ParameterValue(out);
    }
    const bool numeric =
        std::all_of(items.begin(), items.end(), [](const auto& v) {
            return v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ||
                   v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE;
        });
    if (numeric) {
        std::vector<double> out;
        for (const auto& item : node) {
            out.push_back(item.as<double>());
        }
        return rclcpp::ParameterValue(out);
    }
    std::vector<std::string> out;
    for (const auto& item : node) {
        out.push_back(item.Scalar());
    }
    return rclcpp::ParameterValue(out);
}

void flatten(const YAML::Node& node,
             const std::string& name,
             std::map<std::string, rclcpp::ParameterValue>& out) {
    if (node.IsMap()) {
        for (const auto& kv : node) {
            flatten(kv.second, name + "." + kv.first.as<std::string>(), out);
        }
        return;
    }
    out[name] = yaml_to_parameter_value(node);
}

}  // namespace

void LandmarkServerNode::init_layouts() {
    const auto file = this->declare_parameter<std::string>("course_file", "");
    course_gui_state_ =
        this->declare_parameter<std::string>("course_gui_state", "");
    if (!file.empty()) {
        std::error_code ec;
        const fs::path real = fs::weakly_canonical(file, ec);
        const fs::path path = ec ? fs::path(file) : real;
        layout_dir_ = path.parent_path();
        active_layout_ = path.stem().string();
    }
    spdlog::info("LandmarkServer: course layout '{}' ({}){}", active_layout_,
                 layout_dir_.string(),
                 in_source_tree(layout_dir_)
                     ? ""
                     : ", not in the package source: set_course cannot save "
                       "(build with colcon --symlink-install)");
}

YAML::Node LandmarkServerNode::layout_course_tree(
    const std::string& layout,
    std::string* gui_state) const {
    // What every layout shares: templates.yaml, else the course in use
    // without its layout.
    YAML::Node base = with_layout_file(course_tree_, YAML::Node());
    const fs::path templates = layout_dir_ / "templates.yaml";
    std::error_code ec;
    if (fs::exists(templates, ec)) {
        base =
            with_layout_file(read_layout_file(templates).course, YAML::Node());
    }
    const fs::path file = layout_dir_ / (layout + ".yaml");
    if (!fs::exists(file, ec)) {
        return base;
    }
    const LayoutFile f = read_layout_file(file);
    if (gui_state != nullptr) {
        *gui_state = f.gui_state;
    }
    return with_layout_file(base, f.course);
}

void LandmarkServerNode::handle_get_course(
    const std::shared_ptr<vortex_msgs::srv::GetCourse::Request> req,
    std::shared_ptr<vortex_msgs::srv::GetCourse::Response> res) {
    const std::string layout =
        req->layout.empty() ? active_layout_ : req->layout;
    res->layout = layout;
    res->active_layout = active_layout_;
    if (!layout_dir_.empty()) {
        res->layouts = list_layouts(layout_dir_);
        res->file = (layout_dir_ / (layout + ".yaml")).string();
    }
    if (!active_layout_.empty() &&
        std::find(res->layouts.begin(), res->layouts.end(), active_layout_) ==
            res->layouts.end()) {
        res->layouts.push_back(active_layout_);  // in use, not saved yet
    }

    YAML::Node tree;
    CourseConfig config;
    try {
        if (layout == active_layout_) {
            tree = course_tree_;
            res->gui_state = course_gui_state_;
        } else {
            if (!valid_layout_name(layout)) {
                throw std::runtime_error("invalid layout name '" + layout +
                                         "'");
            }
            if (!fs::exists(layout_dir_ / (layout + ".yaml"))) {
                throw std::runtime_error("no layout '" + layout + "' in " +
                                         layout_dir_.string());
            }
            tree = layout_course_tree(layout, &res->gui_state);
        }
        config = parse_course_config(tree);
    } catch (const std::exception& e) {
        res->success = false;
        res->message = e.what();
        return;
    }

    res->enable = config.enable;
    res->start_x = config.start_xy.x();
    res->start_y = config.start_xy.y();
    for (const auto& t : config.tasks) {
        vortex_msgs::msg::CoursePrior p;
        p.name = t.name;
        p.template_name = t.template_name;
        p.x = t.prior_xy.x();
        p.y = t.prior_xy.y();
        p.yaw_deg = t.prior_yaw * 180.0 / M_PI;
        p.region_radius_m = t.region_radius_m;
        res->tasks.push_back(p);
    }
    for (const auto& tmpl : config.templates) {
        vortex_msgs::msg::CourseTemplate tm;
        tm.name = tmpl.name;
        tm.region_radius_m = template_region_radius(tree, tmpl.name);
        for (const auto& m : tmpl.variants.front().members) {
            tm.part_names.push_back(m.name);
            geometry_msgs::msg::Point p;
            p.x = m.offset.x();
            p.y = m.offset.y();
            p.z = m.offset.z();
            tm.part_offsets.push_back(p);
        }
        res->templates.push_back(tm);
    }
    res->success = true;
    res->message = "layout " + layout + ": " +
                   std::to_string(config.tasks.size()) + " tasks";
}

void LandmarkServerNode::handle_set_course(
    const std::shared_ptr<vortex_msgs::srv::SetCourse::Request> req,
    std::shared_ptr<vortex_msgs::srv::SetCourse::Response> res) {
    const std::string layout =
        req->layout.empty() ? active_layout_ : req->layout;
    const bool same_layout = layout == active_layout_;
    const auto refuse = [&](const std::string& why) {
        res->success = false;
        res->message = why;
        spdlog::warn("LandmarkServer: set_course refused: {}", why);
    };
    if ((!same_layout || req->save) && !valid_layout_name(layout)) {
        return refuse("invalid layout name '" + layout +
                      "': letters, digits, _ and -, not 'templates'");
    }

    CourseLayout request;
    request.enable = req->enable;
    request.start_x = req->start_x;
    request.start_y = req->start_y;
    for (const auto& p : req->tasks) {
        request.tasks.push_back(
            {p.name, p.template_name, p.x, p.y, p.yaw_deg, p.region_radius_m});
    }

    // The whole layout is checked by the code that reads it at start, before
    // anything changes.
    YAML::Node tree;
    CourseConfig config;
    try {
        const YAML::Node base =
            same_layout ? course_tree_ : layout_course_tree(layout, nullptr);
        tree = apply_layout(base, request);
        config = parse_course_config(tree);
    } catch (const std::exception& e) {
        return refuse(e.what());
    }

    std::string saved;
    if (req->save) {
        if (layout_dir_.empty()) {
            return refuse(
                "cannot save: the layout directory is unknown (course_file not "
                "set)");
        }
        if (!in_source_tree(layout_dir_)) {
            return refuse("cannot save: " + layout_dir_.string() +
                          " is not the package source; build with colcon "
                          "--symlink-install");
        }
        const fs::path file = layout_dir_ / (layout + ".yaml");
        try {
            const fs::path backup =
                write_layout_file(file, tree, req->gui_state, timestamp_now());
            saved = "; saved to " + file.string() +
                    (backup.empty() ? ""
                                    : " (the old one in backup/" +
                                          backup.filename().string() + ")");
        } catch (const std::exception& e) {
            return refuse(std::string("not saved: ") + e.what());
        }
    }

    std::optional<CourseModel::LayoutUpdate> update;
    if (same_layout) {
        update = map_->course().update_layout(config);
    }
    map_config_.course = config;
    map_->set_config(map_config_);
    if (!update) {
        restart_course(config);
    }
    course_tree_ = tree;
    course_gui_state_ = req->gui_state;
    active_layout_ = layout;
    sync_course_parameters();

    const auto names = [](const std::vector<std::string>& v) {
        std::string out;
        for (const auto& n : v) {
            out += (out.empty() ? "" : ", ") + n;
        }
        return out;
    };
    res->success = true;
    res->map_cleared = !update.has_value();
    std::string message = "layout " + layout + ": " +
                          std::to_string(config.tasks.size()) + " tasks" +
                          (config.enable ? "" : ", course model off");
    if (update) {
        res->applied = update->applied;
        res->kept = update->kept;
        if (!update->applied.empty()) {
            message += "; new prior in use: " + names(update->applied);
        }
        if (!update->kept.empty()) {
            message += "; placed, pose kept: " + names(update->kept);
        }
    } else {
        for (const auto& t : config.tasks) {
            res->applied.push_back(t.name);
        }
        message += "; new map (another layout, task list, template or enable)";
    }
    res->message = message + saved;
    spdlog::info("LandmarkServer: {}", res->message);
    publish_course_state();
}

void LandmarkServerNode::restart_course(const CourseConfig& config) {
    map_->course().reset(config);
    map_->clear();
    drop_counts_.clear();
    clear_graph();
    track_manager_config_.per_class_configs = config_class_configs_;
    apply_track_limits();
    track_manager_ = std::make_unique<vortex::filtering::PoseTrackManager>(
        track_manager_config_);
    filter_time_sec_.reset();
}

void LandmarkServerNode::sync_course_parameters() {
    std::map<std::string, rclcpp::ParameterValue> wanted;
    for (const auto& key : kLayoutKeys) {
        if (course_tree_[key]) {
            flatten(course_tree_[key], "course." + key, wanted);
        }
    }
    const auto in_layout = [](const std::string& name) {
        return std::any_of(
            kLayoutKeys.begin(), kLayoutKeys.end(), [&](const auto& key) {
                const std::string prefix = "course." + key;
                return name == prefix || name.rfind(prefix + ".", 0) == 0;
            });
    };
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;

    syncing_course_parameters_ = true;
    try {
        for (const auto& name : this->list_parameters({"course"}, 0).names) {
            if (in_layout(name) && !wanted.contains(name)) {
                this->undeclare_parameter(name);
            }
        }
        for (const auto& [name, value] : wanted) {
            if (!this->has_parameter(name)) {
                this->declare_parameter(name, value, descriptor);
            } else if (this->get_parameter(name).get_parameter_value() !=
                       value) {
                this->set_parameter(rclcpp::Parameter(name, value));
            }
        }
        if (!layout_dir_.empty()) {
            this->set_parameter(rclcpp::Parameter(
                "course_file",
                (layout_dir_ / (active_layout_ + ".yaml")).string()));
        }
        this->set_parameter(
            rclcpp::Parameter("course_gui_state", course_gui_state_));
    } catch (const std::exception& e) {
        spdlog::warn("LandmarkServer: course parameters not updated: {}",
                     e.what());
    }
    syncing_course_parameters_ = false;
}

}  // namespace vortex::mission
