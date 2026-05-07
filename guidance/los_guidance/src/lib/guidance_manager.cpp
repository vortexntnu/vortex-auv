#include "los_guidance/lib/guidance_manager.hpp"
#include <spdlog/spdlog.h>
#include <yaml-cpp/node/node.h>
#include <cmath>
#include <limits>
#include "los_guidance/lib/utils.hpp"

namespace vortex::guidance::los {

// Constructor
LosGuidanceStateManager::LosGuidanceStateManager(
    const std::string& yaml_file_path) {
    YAML::Node config = utils::load_yaml_config(yaml_file_path);

    parse_common_config(config["common"]);
    set_adaptive_los_guidance(config);
    set_proportional_los_guidance(config);
    set_integral_los_guidance(config);
    set_vector_field_guidance(config);
}

// Adaptive LOS setup
void LosGuidanceStateManager::set_adaptive_los_guidance(YAML::Node config) {
    auto adaptive_los_config = config["adaptive_los"];
    auto params = AdaptiveLosParams{};

    try {
        params.lookahead_distance_h =
            adaptive_los_config["lookahead_distance_h"].as<double>();
        params.lookahead_distance_v =
            adaptive_los_config["lookahead_distance_v"].as<double>();
        params.adaptation_gain_h =
            adaptive_los_config["adaptation_gain_h"].as<double>();
        params.adaptation_gain_v =
            adaptive_los_config["adaptation_gain_v"].as<double>();
        params.time_step = time_step_s_;

        adaptive_los_ = std::make_unique<AdaptiveLOSGuidance>(params);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(
            std::string("Failed to load adaptive_los parameters: ") + e.what());
    }
}

// Proportional LOS setup
void LosGuidanceStateManager::set_proportional_los_guidance(YAML::Node config) {
    auto proportional_los_config = config["prop_los"];
    auto params = ProportionalLosParams{};

    try {
        params.lookahead_distance_h =
            proportional_los_config["lookahead_distance_h"].as<double>();
        params.lookahead_distance_v =
            proportional_los_config["lookahead_distance_v"].as<double>();

        proportional_los_ = std::make_unique<ProportionalLOSGuidance>(params);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(
            std::string("Failed to load proportional_los parameters: ") +
            e.what());
    }
}

// Integral LOS setup
void LosGuidanceStateManager::set_integral_los_guidance(YAML::Node config) {
    auto integral_los_config = config["integer_los"];
    auto params = IntegralLosParams{};

    try {
        params.proportional_gain_h =
            integral_los_config["proportional_gain_h"].as<double>();
        params.proportional_gain_v =
            integral_los_config["proportional_gain_v"].as<double>();
        params.integral_gain_h =
            integral_los_config["integral_gain_h"].as<double>();
        params.integral_gain_v =
            integral_los_config["integral_gain_v"].as<double>();
        params.time_step = time_step_s_;

        integral_los_ = std::make_unique<IntegralLOSGuidance>(params);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(
            std::string("Failed to load integral_los parameters: ") + e.what());
    }
}

// Vector field LOS setup
void LosGuidanceStateManager::set_vector_field_guidance(YAML::Node config) {
    auto vector_field_config = config["vector_field_los"];
    auto params = VectorFieldLosParams{};

    try {
        params.max_approach_angle_h =
            vector_field_config["max_approach_angle_h"].as<double>();
        params.max_approach_angle_v =
            vector_field_config["max_approach_angle_v"].as<double>();
        params.proportional_gain_h =
            vector_field_config["proportional_gain_h"].as<double>();
        params.proportional_gain_v =
            vector_field_config["proportional_gain_v"].as<double>();
        params.time_step = time_step_s_;

        vector_field_los_ = std::make_unique<VectorFieldLOSGuidance>(params);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(
            std::string("Failed to load vector_field_los parameters: ") +
            e.what());
    }
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

        const auto m = common_config["active_los_method"];
        if (!m) {
            throw std::runtime_error(
                "Missing required field 'active_los_method'");
        }
        try {
            method_ = types::int_to_active_los_method(m.as<int>());
        } catch (const YAML::BadConversion&) {
            method_ = types::string_to_active_los_method(m.as<std::string>());
        }

        time_step_s_ = common_config["time_step_s"].as<double>();

        lock.unlock();
    } catch (const YAML::Exception& e) {
        throw std::runtime_error(
            std::string("Failed to load common parameters: ") + e.what());
    }
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

// Set LOS method
void LosGuidanceStateManager::set_los_method(types::ActiveLosMethod method) {
    std::unique_lock<std::mutex> lock(mutex_);
    method_ = method;
    lock.unlock();
}

// Initialize goal
void LosGuidanceStateManager::initialize_goal(const types::Point& new_wp) {
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

    adaptive_los_->reset();

    nearest_been_to_goal_ = std::numeric_limits<double>::infinity();
    time_since_nearest_goal_s_ = 0.0;
}

// Check if goal is feasible
bool LosGuidanceStateManager::is_goal_feasible(
    std::shared_ptr<const vortex_msgs::action::GuidanceWaypoint::Goal> goal) {
    std::unique_lock<std::mutex> lock(mutex_);
    types::Inputs inputs_copy = path_inputs_;
    double max_pitch_angle_copy = max_pitch_angle_;
    lock.unlock();

    const auto& current_position = inputs_copy.current_position;
    const auto& goal_point = goal->waypoint.pose.position;

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
    types::Inputs inputs_copy = path_inputs_;
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
    types::Inputs inputs_copy = path_inputs_;
    lock.unlock();

    return (inputs_copy.current_position - inputs_copy.next_point)
               .as_vector()
               .norm() < tolerance;
}

// Calculate outputs
types::Outputs LosGuidanceStateManager::calculate_outputs() {
    std::unique_lock<std::mutex> lock(mutex_);
    types::Inputs inputs_copy = path_inputs_;
    types::ActiveLosMethod method_copy = method_;
    lock.unlock();

    types::Outputs outputs;

    switch (method_copy) {
        case types::ActiveLosMethod::ADAPTIVE:
            outputs = adaptive_los_->calculate_outputs(inputs_copy);
            break;
        case types::ActiveLosMethod::PROPORTIONAL:
            outputs = proportional_los_->calculate_outputs(inputs_copy);
            break;
        case types::ActiveLosMethod::INTEGRAL:
            outputs = integral_los_->calculate_outputs(inputs_copy);
            break;
        case types::ActiveLosMethod::VECTOR_FIELD:
            outputs = vector_field_los_->calculate_outputs(inputs_copy);
            break;
        default:
            spdlog::error("Invalid LOS method selected");
            break;
    }

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
