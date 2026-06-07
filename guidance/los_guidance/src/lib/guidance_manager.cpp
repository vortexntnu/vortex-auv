#include "los_guidance/lib/guidance_manager.hpp"
#include <spdlog/spdlog.h>
#include <yaml-cpp/node/node.h>
#include <cmath>
#include <limits>
#include "los_guidance/lib/utils.hpp"

namespace vortex::guidance::los {

// Constructor
LosGuidanceStateManager::LosGuidanceStateManager(
    const std::string& yaml_file_path)
    : yaml_file_path_(yaml_file_path) {
    YAML::Node config = utils::load_yaml_config(yaml_file_path_);
    parse_common_config(config["common"]);
    std::unique_lock<std::mutex> lock(mutex_);
    emplace_los_from_config(config);
}

// Parse common config
void LosGuidanceStateManager::parse_common_config(
    const YAML::Node& common_config) {
    try {
        std::unique_lock<std::mutex> lock(mutex_);

        u_desired_ = common_config["u_desired"].as<double>();
        max_pitch_angle_ = common_config["max_pitch_angle"].as<double>();
        goal_reached_tol_ = common_config["goal_reached_tol"].as<double>();
        missed_goal_timeout_s_ =
            common_config["missed_goal_timeout"].as<double>();
        missed_goal_distance_margin_ =
            common_config["missed_goal_distance_margin"].as<double>();
        time_step_s_ = common_config["time_step_s"].as<double>();

        lock.unlock();
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(
            std::string("Failed to load common parameters: ") + e.what());
    }
}

// Emplace the variant for the given method using parameters from config.
// Must be called while holding mutex_.
void LosGuidanceStateManager::emplace_los_for_method(
    types::ActiveLosMethod method,
    const YAML::Node& config) {
    try {
        switch (method) {
            case types::ActiveLosMethod::PROPORTIONAL: {
                auto c = config["prop_los"];
                ProportionalLosParams params;
                params.lookahead_distance_h =
                    c["lookahead_distance_h"].as<double>();
                params.lookahead_distance_v =
                    c["lookahead_distance_v"].as<double>();
                los_method_.emplace<ProportionalLOSGuidance>(params);
                break;
            }
            case types::ActiveLosMethod::INTEGRAL: {
                auto c = config["integer_los"];
                IntegralLosParams params;
                params.proportional_gain_h =
                    c["proportional_gain_h"].as<double>();
                params.proportional_gain_v =
                    c["proportional_gain_v"].as<double>();
                params.integral_gain_h = c["integral_gain_h"].as<double>();
                params.integral_gain_v = c["integral_gain_v"].as<double>();
                params.time_step_s = time_step_s_;
                los_method_.emplace<IntegralLOSGuidance>(params);
                break;
            }
            case types::ActiveLosMethod::ADAPTIVE: {
                auto c = config["adaptive_los"];
                AdaptiveLosParams params;
                params.lookahead_distance_h =
                    c["lookahead_distance_h"].as<double>();
                params.lookahead_distance_v =
                    c["lookahead_distance_v"].as<double>();
                params.adaptation_gain_h = c["adaptation_gain_h"].as<double>();
                params.adaptation_gain_v = c["adaptation_gain_v"].as<double>();
                params.time_step_s = time_step_s_;
                los_method_.emplace<AdaptiveLOSGuidance>(params);
                break;
            }
            case types::ActiveLosMethod::VECTOR_FIELD: {
                auto c = config["vector_field_los"];
                VectorFieldLosParams params;
                params.max_approach_angle_h =
                    c["max_approach_angle_h"].as<double>();
                params.max_approach_angle_v =
                    c["max_approach_angle_v"].as<double>();
                params.proportional_gain_h =
                    c["proportional_gain_h"].as<double>();
                params.proportional_gain_v =
                    c["proportional_gain_v"].as<double>();
                params.time_step_s = time_step_s_;
                los_method_.emplace<VectorFieldLOSGuidance>(params);
                break;
            }
        }
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(
            std::string("Failed to load LOS method parameters: ") + e.what());
    }
}

// Read active_los_method from config["common"] and emplace the variant.
// Must be called while holding mutex_.
void LosGuidanceStateManager::emplace_los_from_config(
    const YAML::Node& config) {
    const auto m = config["common"]["active_los_method"];
    if (!m) {
        throw std::runtime_error("Missing required field 'active_los_method'");
    }

    types::ActiveLosMethod method;
    try {
        method = types::int_to_active_los_method(m.as<int>());
    } catch (const YAML::BadConversion&) {
        method = types::string_to_active_los_method(m.as<std::string>());
    }

    emplace_los_for_method(method, config);
}

// Update waypoint
void LosGuidanceStateManager::update_waypoint(const types::Point& new_wp) {
    std::unique_lock<std::mutex> lock(mutex_);

    if (!has_active_segment_) {
        path_inputs_.prev_point = path_inputs_.current_position;
        path_inputs_.next_point = new_wp;
        has_active_segment_ = true;
    } else {
        path_inputs_.prev_point = path_inputs_.next_point;
        path_inputs_.next_point = new_wp;
    }

    lock.unlock();
}

// Update position
void LosGuidanceStateManager::update_position(const types::Point& position) {
    std::unique_lock<std::mutex> lock(mutex_);
    path_inputs_.current_position = position;
    lock.unlock();
}

// Update yaw
void LosGuidanceStateManager::update_yaw(double yaw) {
    std::unique_lock<std::mutex> lock(mutex_);
    current_yaw_ = yaw;
    lock.unlock();
}

// Set LOS method — immediately re-emplaces the variant with fresh params from
// the YAML config file, which also implicitly resets any stateful method.
void LosGuidanceStateManager::set_los_method(types::ActiveLosMethod method) {
    YAML::Node config = utils::load_yaml_config(yaml_file_path_);
    parse_common_config(config["common"]);
    std::unique_lock<std::mutex> lock(mutex_);
    emplace_los_for_method(method, config);
    lock.unlock();
}

// Initialize goal
void LosGuidanceStateManager::initialize_goal(const types::Point& new_wp) {
    YAML::Node config = utils::load_yaml_config(yaml_file_path_);
    parse_common_config(config["common"]);

    std::unique_lock<std::mutex> lock(mutex_);

    if (!has_active_segment_) {
        path_inputs_.prev_point = path_inputs_.current_position;
        path_inputs_.next_point = new_wp;
        has_active_segment_ = true;
    } else {
        path_inputs_.prev_point = path_inputs_.next_point;
        path_inputs_.next_point = new_wp;
    }

    emplace_los_from_config(config);

    nearest_been_to_goal_ = std::numeric_limits<double>::infinity();
    time_since_nearest_goal_s_ = 0.0;

    lock.unlock();
}

// Check if goal is feasible
bool LosGuidanceStateManager::is_goal_feasible(const types::Point& goal_point) {
    std::unique_lock<std::mutex> lock(mutex_);
    types::GuidanceInputs inputs_copy = path_inputs_;
    double max_pitch_angle_copy = max_pitch_angle_;
    lock.unlock();

    const auto& current_position = inputs_copy.current_position;

    const double dx = goal_point.x - current_position.x;
    const double dy = goal_point.y - current_position.y;
    const double dz = goal_point.z - current_position.z;

    const double horizontal_distance = std::sqrt(dx * dx + dy * dy);
    const double required_pitch = std::atan2(-dz, horizontal_distance);

    return std::abs(required_pitch) <= max_pitch_angle_copy;
}

// Check if goal is missed
bool LosGuidanceStateManager::is_goal_missed() {
    std::unique_lock<std::mutex> lock(mutex_);
    types::GuidanceInputs inputs_copy = path_inputs_;
    lock.unlock();

    const double distance_to_goal =
        (inputs_copy.current_position - inputs_copy.next_point)
            .as_vector()
            .norm();

    if (distance_to_goal < nearest_been_to_goal_) {
        nearest_been_to_goal_ = distance_to_goal;
        time_since_nearest_goal_s_ = 0.0;
        return false;
    }

    if (distance_to_goal >
        nearest_been_to_goal_ + missed_goal_distance_margin_) {
        time_since_nearest_goal_s_ += time_step_s_;
    } else {
        time_since_nearest_goal_s_ = 0.0;
    }

    return time_since_nearest_goal_s_ >= missed_goal_timeout_s_;
}

// Check if goal is reached
bool LosGuidanceStateManager::is_goal_reached(double tolerance) {
    std::unique_lock<std::mutex> lock(mutex_);
    types::GuidanceInputs inputs_copy = path_inputs_;
    lock.unlock();

    return (inputs_copy.current_position - inputs_copy.next_point)
               .as_vector()
               .norm() < tolerance;
}

// Calculate outputs
types::GuidanceOutputs LosGuidanceStateManager::calculate_outputs() {
    std::unique_lock<std::mutex> lock(mutex_);
    types::GuidanceInputs inputs_copy = path_inputs_;
    types::GuidanceOutputs outputs;
    std::visit(
        [&](auto& los) {
            using T = std::decay_t<decltype(los)>;
            if constexpr (!std::is_same_v<T, std::monostate>) {
                outputs = los.calculate_outputs(inputs_copy);
            } else {
                spdlog::error("LOS method not initialized");
            }
        },
        los_method_);
    return outputs;
}

// Getters
double LosGuidanceStateManager::get_max_pitch_angle() const {
    std::unique_lock<std::mutex> lock(mutex_);
    return max_pitch_angle_;
}

double LosGuidanceStateManager::get_current_yaw() const {
    std::unique_lock<std::mutex> lock(mutex_);
    return current_yaw_;
}

double LosGuidanceStateManager::get_u_desired() const {
    std::unique_lock<std::mutex> lock(mutex_);
    return u_desired_;
}

}  // namespace vortex::guidance::los
