#include <spdlog/spdlog.h>
#include <algorithm>
#include <sstream>
#include "landmark_server/landmark_server_ros.hpp"

// The config files arrive as parameter overrides with dotted names. They are
// turned back into one YAML tree and parsed by the ROS-free parsers
// (class_config, course_model, landmark_graph), at start and on every live
// change, so a value is checked the same way both times.

namespace vortex::mission {

namespace {

YAML::Node parameter_value_to_yaml(const rclcpp::ParameterValue& value) {
    YAML::Node node;
    switch (value.get_type()) {
        case rclcpp::ParameterType::PARAMETER_BOOL:
            node = value.get<bool>();
            break;
        case rclcpp::ParameterType::PARAMETER_INTEGER:
            node = value.get<int64_t>();
            break;
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
            node = value.get<double>();
            break;
        case rclcpp::ParameterType::PARAMETER_STRING:
            node = value.get<std::string>();
            break;
        case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
            for (const double d : value.get<std::vector<double>>()) {
                node.push_back(d);
            }
            break;
        case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
            for (const auto i : value.get<std::vector<int64_t>>()) {
                node.push_back(i);
            }
            break;
        case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
            for (const auto& str : value.get<std::vector<std::string>>()) {
                node.push_back(str);
            }
            break;
        case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
            for (const bool b : value.get<std::vector<bool>>()) {
                node.push_back(b);
            }
            break;
        default:
            break;
    }
    return node;
}

std::string root_of(const std::string& name) {
    return name.substr(0, name.find('.'));
}

bool has_root(const std::vector<std::string>& roots, const std::string& name) {
    return std::find(roots.begin(), roots.end(), root_of(name)) != roots.end();
}

/// A YAML tree from the dotted parameter names under the given roots.
YAML::Node overrides_to_yaml(
    const std::map<std::string, rclcpp::ParameterValue>& overrides,
    const std::vector<std::string>& roots) {
    YAML::Node root(YAML::NodeType::Map);
    for (const auto& [name, value] : overrides) {
        if (!has_root(roots, name)) {
            continue;
        }
        std::vector<std::string> parts;
        std::stringstream ss(name);
        for (std::string part; std::getline(ss, part, '.');) {
            parts.push_back(part);
        }
        YAML::Node node = root;
        for (std::size_t i = 0; i + 1 < parts.size(); ++i) {
            if (!node[parts[i]]) {
                node[parts[i]] = YAML::Node(YAML::NodeType::Map);
            }
            node.reset(node[parts[i]]);
        }
        node[parts.back()] = parameter_value_to_yaml(value);
    }
    return root;
}

/// Map rules and the map view: these can change while the server runs.
const std::vector<std::string> kLiveRoots = {"intake", "course_frame",
                                             "classes", "rules", "markers"};
/// Tracker, graph, detector noise and course layout: read once at start.
const std::vector<std::string> kRestartRoots = {"track_config", "graph",
                                                "detector_noise", "course"};
/// Debug switches, applied at once.
const std::vector<std::string> kDebugSwitches = {"debug.enable",
                                                 "debug.markers"};

}  // namespace

void LandmarkServerNode::load_config() {
    const auto overrides =
        this->get_node_parameters_interface()->get_parameter_overrides();
    const YAML::Node tree = overrides_to_yaml(
        overrides, {"intake", "course_frame", "classes", "rules", "markers",
                    "graph", "detector_noise", "course"});
    map_config_ = parse_map_config(tree);
    intake_config_ = map_config_.intake;

    // The graph weighs the detections with the same detector noise as the
    // tracker.
    if (tree["graph"] && tree["graph"]["frame_id"]) {
        throw std::runtime_error(
            "graph.frame_id: moved to debug.graph_frame_id (display only)");
    }
    LandmarkGraphConfig graph_config = parse_graph_config(tree["graph"]);
    const DetectorNoise& noise = map_config_.intake.noise;
    graph_config.meas_base_std_m = noise.base_std_m;
    graph_config.meas_along_std_per_m = noise.along_std_per_m;
    graph_config.meas_across_std_per_m = noise.across_std_per_m;
    graph_ = std::make_unique<LandmarkGraph>(graph_config);

    // Per-class tracker settings (track_config.<CLASS>) on top of the
    // default, then no more tracks of a class than the map can use.
    const YAML::Node track_tree = overrides_to_yaml(overrides, {"track_config"});
    track_manager_config_.per_class_configs = parse_per_class_track_config(
        track_tree["track_config"], track_manager_config_.default_class_config);
    map_ = std::make_unique<RetainedLandmarks>(map_config_);
    apply_track_limits();
    track_manager_ = std::make_unique<vortex::filtering::PoseTrackManager>(
        track_manager_config_);
    course_ = std::make_unique<CourseFrameTracker>(map_config_.course_frame);

    // The values that differ between sim.yaml and pool.yaml, so the log
    // shows which environment is running.
    const auto& z_lock = map_config_.map_rules.z_lock;
    const auto& tc = track_manager_config_.default_class_config;
    spdlog::info(
        "LandmarkServer config: graph {}, z_lock {} (floor {:.3f} m), "
        "detector noise {:.3f} m + {:.3f}/m along, {:.4f}/m across, sensor "
        "std {:.2f} m, max_pos_error {:.2f} m, graph yaw noise {:.2f} deg/m",
        graph_->config().enable ? "on" : "off", z_lock.enable ? "on" : "off",
        z_lock.floor_z, noise.base_std_m, noise.along_std_per_m,
        noise.across_std_per_m, tc.sens_std_dev, tc.max_pos_error,
        graph_->config().odom_yaw_std_deg_per_m);
    if (map_config_.course.enable) {
        std::string tasks;
        for (const auto& t : map_config_.course.tasks) {
            tasks += (tasks.empty() ? "" : ", ") + t.name;
        }
        spdlog::info("LandmarkServer: course layout with {} tasks: {}",
                     map_config_.course.tasks.size(), tasks);
    } else {
        spdlog::warn(
            "LandmarkServer: no course layout (course.enable false): every "
            "class is mapped as a free landmark");
    }

    declare_config_parameters(overrides);
    parameters_cb_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
            return on_parameters_set(params);
        });
}

void LandmarkServerNode::declare_config_parameters(
    const std::map<std::string, rclcpp::ParameterValue>& overrides) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    // `ros2 param set ... 4` for a double must not fail on the type: the
    // parser checks the values.
    descriptor.dynamic_typing = true;
    for (const auto& [name, value] : overrides) {
        if ((has_root(kLiveRoots, name) || has_root(kRestartRoots, name)) &&
            !this->has_parameter(name)) {
            this->declare_parameter(name, value, descriptor);
        }
    }
}

rcl_interfaces::msg::SetParametersResult LandmarkServerNode::on_parameters_set(
    const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    const auto unchanged = [this](const rclcpp::Parameter& p) {
        return this->has_parameter(p.get_name()) &&
               this->get_parameter(p.get_name()).get_parameter_value() ==
                   p.get_parameter_value();
    };

    bool map_rules_changed = false;
    for (const auto& p : params) {
        const auto& name = p.get_name();
        if (std::find(kDebugSwitches.begin(), kDebugSwitches.end(), name) !=
            kDebugSwitches.end()) {
            if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                result.successful = false;
                result.reason = name + " must be true or false";
                return result;
            }
            (name == "debug.enable" ? debug_enabled_ : markers_enabled_) =
                p.as_bool();
            if (this->has_parameter(name) && !unchanged(p)) {
                spdlog::info("LandmarkServer: {} = {}", name,
                             p.value_to_string());
            }
            continue;
        }
        if (name == "debug.graph_frame_id" && !this->has_parameter(name)) {
            continue;  // declared at start
        }
        if (has_root(kRestartRoots, name) ||
            name == "debug.graph_frame_id") {
            // Loading a whole config file sets these too: the same value is
            // fine, a new one needs a restart.
            if (unchanged(p)) {
                continue;
            }
            result.successful = false;
            result.reason = name +
                            ": read at start; restart the landmark server to "
                            "change it";
            return result;
        }
        map_rules_changed = map_rules_changed || has_root(kLiveRoots, name);
    }
    if (!map_rules_changed) {
        return result;
    }

    // The whole rule set as it would be after this change, parsed by the
    // same code as at start, so a bad value is rejected here.
    std::map<std::string, rclcpp::ParameterValue> values;
    const auto listed = this->list_parameters(kLiveRoots, 0);
    for (const auto& p : this->get_parameters(listed.names)) {
        values[p.get_name()] = p.get_parameter_value();
    }
    for (const auto& p : params) {
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
            values.erase(p.get_name());
        } else {
            values[p.get_name()] = p.get_parameter_value();
        }
    }
    LandmarkMapConfig config;
    try {
        config = parse_map_config(overrides_to_yaml(values, kLiveRoots));
    } catch (const std::exception& e) {
        result.successful = false;
        result.reason = std::string("invalid map rules: ") + e.what();
        return result;
    }

    for (const auto& p : params) {
        if (!has_root(kLiveRoots, p.get_name()) || unchanged(p)) {
            continue;
        }
        if (!this->has_parameter(p.get_name())) {
            spdlog::warn(
                "LandmarkServer: new parameter {} (not in the config files): "
                "check the spelling, unknown keys are ignored",
                p.get_name());
        }
        spdlog::info("LandmarkServer: {} = {}", p.get_name(),
                     p.value_to_string());
    }
    {
        std::lock_guard<std::mutex> lock(intake_mtx_);
        config.intake.noise = intake_config_.noise;  // read at start only
        intake_config_ = config.intake;
    }
    std::lock_guard<std::mutex> lock(pending_map_config_mtx_);
    pending_map_config_ = std::move(config);
    return result;
}

void LandmarkServerNode::apply_pending_map_config() {
    std::optional<LandmarkMapConfig> pending;
    {
        std::lock_guard<std::mutex> lock(pending_map_config_mtx_);
        pending.swap(pending_map_config_);
    }
    if (!pending) {
        return;
    }
    pending->course = map_config_.course;  // read at start only
    map_config_ = std::move(*pending);
    map_->set_config(map_config_);
    course_->set_config(map_config_.course_frame);
    spdlog::info("LandmarkServer: map rules updated");
}

}  // namespace vortex::mission
