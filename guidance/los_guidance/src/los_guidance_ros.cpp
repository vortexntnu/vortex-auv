#include "los_guidance/los_guidance_ros.hpp"
#include "los_guidance/lib/types.hpp"
#include <spdlog/spdlog.h>
#include <string>
#include <vortex/utils/qos_profiles.hpp>
#include <yaml-cpp/node/node.h>

const auto start_message = R"(
  _     ___  ____     ____       _     _
 | |   / _ \/ ___|   / ___|_   _(_) __| | __ _ _ __   ___ ___
 | |  | | | \___ \  | |  _| | | | |/ _` |/ _` | '_ \ / __/ _ \
 | |__| |_| |___) | | |_| | |_| | | (_| | (_| | | | | (_|  __/
 |_____\___/|____/   \____|\__,_|_|\__,_|\__,_|_| |_|\___\___|

)";

namespace vortex::guidance::los{

    LosGuidanceNode::LosGuidanceNode() : Node("los_guidance_node"){
        double time_step_s = this->declare_parameter<double>("time_step");
        time_step_ = std::chrono::milliseconds(static_cast<int>(time_step_s * 1000));
        //auto config = this->declare_parameter<YAML::Node>("los_config_file");


        YAML::Node config = get_los_config("config/guidance_params.yaml");

        parse_common_config(config["common"]);
        set_subscribers_and_publisher();
        set_action_server();
        //set_los_mode_service();
        set_adaptive_los_guidance(config);
        set_proportional_los_guidance(config);
        set_integral_los_guidance(config);

        spdlog::info(start_message);
    }

    void LosGuidanceNode::set_subscribers_and_publisher() {
        
        this->declare_parameter<std::string>("topics.pose");
        this->declare_parameter<std::string>("topics.guidance.los");
        this->declare_parameter<std::string>("topics.waypoint");

        std::string pose_topic = this->get_parameter("topics.pose").as_string();
        std::string guidance_topic =
            this->get_parameter("topics.guidance.los").as_string();
        std::string waypoint_topic =
            this->get_parameter("topics.waypoint").as_string();

        auto qos_sensor_data = vortex::utils::qos_profiles::sensor_data_profile(1);

        reference_pub_ = this->create_publisher<vortex_msgs::msg::LOSGuidance>(
            guidance_topic, qos_sensor_data);

        waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            waypoint_topic, qos_sensor_data,
            std::bind(&LosGuidanceNode::waypoint_callback, this,
                    std::placeholders::_1));

        pose_sub_ = this->create_subscription<
            geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_topic, qos_sensor_data,
            std::bind(&LosGuidanceNode::pose_callback, this,
                    std::placeholders::_1));
    }

    void LosGuidanceNode::set_action_server() {
        this->declare_parameter<std::string>("action_servers.los");
        std::string action_server_name =
            this->get_parameter("action_servers.los").as_string();
        cb_group_ =
            this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        action_server_ =
            rclcpp_action::create_server<vortex_msgs::action::LOSGuidance>(
                this, action_server_name,
                std::bind(&LosGuidanceNode::handle_goal, this,
                        std::placeholders::_1, std::placeholders::_2),
                std::bind(&LosGuidanceNode::handle_cancel, this,
                        std::placeholders::_1),
                std::bind(&LosGuidanceNode::handle_accepted, this,
                        std::placeholders::_1),
                rcl_action_server_get_default_options(), cb_group_);
    }

    

    void LosGuidanceNode::set_adaptive_los_guidance(YAML::Node config) {
        auto adaptive_los_config = config["adaptive_los"];
        auto params = AdaptiveLosParams{};
        params.lookahead_distance_h =
            adaptive_los_config["lookahead_distance_h"].as<double>();
        params.lookahead_distance_v =
            adaptive_los_config["lookahead_distance_v"].as<double>();
        params.gamma_h = 
            adaptive_los_config["gama_h"].as<double>();
        params.gamma_v = 
            adaptive_los_config["gama_v"].as<double>();
        params.time_step =
            adaptive_los_config["time_step"].as<double>();

        m_adaptive_los = std::make_unique<AdaptiveLOSGuidance>(params);
    }

    void LosGuidanceNode::set_proportional_los_guidance(YAML::Node config) {
        auto proportional_los_config = config["proportional_los"];
        auto params = ProportionalLosParams{};
        params.lookahead_distance_h =
            proportional_los_config["lookahead_distance_h"].as<double>();
        params.lookahead_distance_v =
            proportional_los_config["lookahead_distance_v"].as<double>();
        params.k_p_h = proportional_los_config["k_p_h"].as<double>();
        params.k_p_v = proportional_los_config["k_p_v"].as<double>();
        params.time_step =
            proportional_los_config["time_step"].as<double>();

        m_proportional_los = std::make_unique<ProportionalLOSGuidance>(params);
    }

    void LosGuidanceNode::set_integral_los_guidance(YAML::Node config) {
        auto integral_los_config = config["integral_los"];
        auto params = IntegralLosParams{};
        params.lookahead_distance_h =
            integral_los_config["lookahead_distance_h"].as<double>();
        params.lookahead_distance_v =
            integral_los_config["lookahead_distance_v"].as<double>();
        params.k_p_h = integral_los_config["k_p_h"].as<double>();
        params.k_p_v = integral_los_config["k_p_v"].as<double>();
        params.k_i_h = integral_los_config["k_i_h"].as<double>();
        params.k_i_v = integral_los_config["k_i_v"].as<double>();
        params.time_step =
            integral_los_config["time_step"].as<double>();

        m_integral_los = std::make_unique<IntegralLOSGuidance>(params);
    }

    void LosGuidanceNode::waypoint_callback(
        const geometry_msgs::msg::PointStamped::SharedPtr los_waypoint) {

        path_inputs_.prev_point = path_inputs_.current_position;
        path_inputs_.next_point = types::Point::point_from_ros(los_waypoint->point);
        spdlog::info("Received waypoint"); // remember to print waypoint that you get 
    }

    void LosGuidanceNode::pose_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
            current_pose) {

        path_inputs_.current_position = types::Point::point_from_ros(current_pose->pose.pose.position);
 
    }

    rclcpp_action::GoalResponse LosGuidanceNode::handle_goal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const vortex_msgs::action::LOSGuidance::Goal> goal) {
        (void)goal;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (goal_handle_) {
                if (goal_handle_->is_active()) {
                    spdlog::info("Aborting current goal and accepting new goal");
                    preempted_goal_id_ = goal_handle_->get_goal_id();
                }
            }
        }
        spdlog::info("Accepted goal request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse LosGuidanceNode::handle_cancel(
        const std::shared_ptr<
            rclcpp_action::ServerGoalHandle<vortex_msgs::action::LOSGuidance>>
            goal_handle) {
        spdlog::info("Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void LosGuidanceNode::handle_accepted(
        const std::shared_ptr<
            rclcpp_action::ServerGoalHandle<vortex_msgs::action::LOSGuidance>>
            goal_handle) {
        execute(goal_handle);
    }

    vortex_msgs::msg::LOSGuidance LosGuidanceNode::fill_los_reference(types::Outputs outputs) {
        vortex_msgs::msg::LOSGuidance reference_msg;
        reference_msg.pitch = outputs.theta_d;
        reference_msg.yaw = outputs.psi_d;
        reference_msg.surge = u_desired_;

        return reference_msg;
    }

    YAML::Node LosGuidanceNode::get_los_config(std::string yaml_file_path) {
        YAML::Node config = YAML::LoadFile(yaml_file_path);
        return config;
    }

    void LosGuidanceNode::parse_common_config(YAML::Node common_config) {
        u_desired_ = common_config["u_desired"].as<double>();
        goal_reached_tol_ = common_config["goal_reached_tol"].as<double>();
    }

    void LosGuidanceNode::execute(


        const std::shared_ptr<
            rclcpp_action::ServerGoalHandle<vortex_msgs::action::LOSGuidance>>
            goal_handle) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            this->goal_handle_ = goal_handle;
        }

        spdlog::info("Executing goal");

        const geometry_msgs::msg::PointStamped los_waypoint = goal_handle->get_goal()->goal;

        path_inputs_.prev_point = path_inputs_.current_position;
        path_inputs_.next_point = types::Point::point_from_ros(los_waypoint.point);

        auto feedback = std::make_shared<vortex_msgs::action::LOSGuidance::Feedback>();
        auto result = std::make_shared<vortex_msgs::action::LOSGuidance::Result>();

        rclcpp::Rate loop_rate(1000.0 / time_step_.count());

        while (rclcpp::ok()) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (goal_handle->get_goal_id() == preempted_goal_id_) {
                    result->success = false;
                    goal_handle->abort(result);
                    return;
                }
            }

            if (goal_handle->is_canceling()) {
                result->success = false;
                goal_handle->canceled(result);
                spdlog::info("Goal canceled");
                return;
            }

            types::Outputs outputs;

            switch(m_method) {
                case types::ActiveLosMethod::ADAPTIVE:
                    outputs = m_adaptive_los->calculate_outputs(path_inputs_);
                    break;
                case types::ActiveLosMethod::PROPORTIONAL:
                    outputs = m_proportional_los->calculate_outputs(path_inputs_);
                    break;
                case types::ActiveLosMethod::INTEGRAL:
                    outputs = m_integral_los->calculate_outputs(path_inputs_);
                    break;
                default:
                    spdlog::error("Invalid LOS method selected");
                    result->success = false;
                    goal_handle->abort(result);
                    return;
            }

            vortex_msgs::msg::LOSGuidance reference_msg = fill_los_reference(outputs);
            feedback->feedback = reference_msg;

            goal_handle->publish_feedback(feedback);
            reference_pub_->publish(reference_msg);

        if ((path_inputs_.current_position - path_inputs_.next_point).as_vector().norm() < goal_reached_tol_) {
            result->success = true;
            goal_handle->succeed(result);
            spdlog::info("Goal reached");
            return;
        }

            loop_rate.sleep();
        }
    }  

} // namespace vortex::guidance
