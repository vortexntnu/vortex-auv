#include "los_guidance/los_guidance_ros.hpp"
#include <spdlog/spdlog.h>
#include <vortex/utils/qos_profiles.hpp>

const auto start_message = R"(
  _     ___  ____     ____       _     _
 | |   / _ \/ ___|   / ___|_   _(_) __| | __ _ _ __   ___ ___
 | |  | | | \___ \  | |  _| | | | |/ _` |/ _` | '_ \ / __/ _ \
 | |__| |_| |___) | | |_| | |_| | | (_| | (_| | | | | (_|  __/
 |_____\___/|____/   \____|\__,_|_|\__,_|\__,_|_| |_|\___\___|

)";

namespace vortex::guidance {

LOSGuidanceNode::LOSGuidanceNode() : Node("los_guidance_node") {
    time_step_ = std::chrono::milliseconds(10);

    set_subscribers_and_publisher();

    set_action_server();

    set_adaptive_los_guidance();

    spdlog::info(start_message);
}

void LOSGuidanceNode::set_subscribers_and_publisher() {
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
        std::bind(&LOSGuidanceNode::waypoint_callback, this,
                  std::placeholders::_1));

    pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, qos_sensor_data,
        std::bind(&LOSGuidanceNode::pose_callback, this,
                  std::placeholders::_1));
}

void LOSGuidanceNode::set_action_server() {
    this->declare_parameter<std::string>("action_servers.los");
    std::string action_server_name =
        this->get_parameter("action_servers.los").as_string();
    cb_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    action_server_ =
        rclcpp_action::create_server<vortex_msgs::action::LOSGuidance>(
            this, action_server_name,
            std::bind(&LOSGuidanceNode::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&LOSGuidanceNode::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&LOSGuidanceNode::handle_accepted, this,
                      std::placeholders::_1),
            rcl_action_server_get_default_options(), cb_group_);
}

void LOSGuidanceNode::set_adaptive_los_guidance() {
    this->declare_parameter<double>("los.lookahead_distance_h");
    this->declare_parameter<double>("los.lookahead_distance_v");
    this->declare_parameter<double>("los.gamma_h");
    this->declare_parameter<double>("los.gamma_v");
    this->declare_parameter<double>("los.time_step");
    this->declare_parameter<double>("los.u_desired");
    this->declare_parameter<double>("los.goal_reached_tol");

    u_desired_ = this->get_parameter("los.u_desired").as_double();
    goal_reached_tol_ = this->get_parameter("los.goal_reached_tol").as_double();

    LOS::Params params;
    params.lookahead_distance_h =
        this->get_parameter("los.lookahead_distance_h").as_double();
    params.lookahead_distance_v =
        this->get_parameter("los.lookahead_distance_v").as_double();
    params.gamma_h = this->get_parameter("los.gamma_h").as_double();
    params.gamma_v = this->get_parameter("los.gamma_v").as_double();
    params.time_step = this->get_parameter("los.time_step").as_double();

    adaptive_los_guidance_ = std::make_unique<AdaptiveLOSGuidance>(params);
}

void LOSGuidanceNode::waypoint_callback(
    const geometry_msgs::msg::PointStamped::SharedPtr los_waypoint) {
    fill_los_waypoints(*los_waypoint);
    adaptive_los_guidance_->update_angles(last_point_, next_point_);
    spdlog::info("Received waypoint");
}

void LOSGuidanceNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
        current_pose) {
    eta_.x = current_pose->pose.pose.position.x;
    eta_.y = current_pose->pose.pose.position.y;
    eta_.z = current_pose->pose.pose.position.z;
}

rclcpp_action::GoalResponse LOSGuidanceNode::handle_goal(
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

rclcpp_action::CancelResponse LOSGuidanceNode::handle_cancel(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::LOSGuidance>>
        goal_handle) {
    spdlog::info("Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void LOSGuidanceNode::handle_accepted(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::LOSGuidance>>
        goal_handle) {
    execute(goal_handle);
}

void LOSGuidanceNode::fill_los_waypoints(
    const geometry_msgs::msg::PointStamped& los_waypoint) {
    last_point_.x = eta_.x;
    last_point_.y = eta_.y;
    last_point_.z = eta_.z;

    next_point_.x = los_waypoint.point.x;
    next_point_.y = los_waypoint.point.y;
    next_point_.z = los_waypoint.point.z;
}

vortex_msgs::msg::LOSGuidance LOSGuidanceNode::fill_los_reference() {
    vortex_msgs::msg::LOSGuidance reference_msg;
    reference_msg.pitch = pitch_d_;
    reference_msg.yaw = yaw_d_;
    reference_msg.surge = u_desired_;

    return reference_msg;
}

void LOSGuidanceNode::execute(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::LOSGuidance>>
        goal_handle) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        this->goal_handle_ = goal_handle;
    }

    spdlog::info("Executing goal");

    const geometry_msgs::msg::PointStamped los_waypoint =
        goal_handle->get_goal()->goal;

    fill_los_waypoints(los_waypoint);

    adaptive_los_guidance_->update_angles(last_point_, next_point_);

    auto feedback =
        std::make_shared<vortex_msgs::action::LOSGuidance::Feedback>();
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

        LOS::CrossTrackError errors =
            adaptive_los_guidance_->calculate_crosstrack_error(last_point_,
                                                               eta_);

        yaw_d_ = adaptive_los_guidance_->calculate_psi_d(errors.y_e);
        pitch_d_ = adaptive_los_guidance_->calculate_theta_d(errors.z_e);

        adaptive_los_guidance_->update_adaptive_estimates(errors);

        vortex_msgs::msg::LOSGuidance reference_msg = fill_los_reference();

        feedback->feedback = reference_msg;

        goal_handle->publish_feedback(feedback);
        reference_pub_->publish(reference_msg);

        if ((eta_ - next_point_).as_vector().norm() < goal_reached_tol_) {
            result->success = true;
            goal_handle->succeed(result);
            vortex_msgs::msg::LOSGuidance reference_msg = fill_los_reference();
            reference_pub_->publish(reference_msg);
            spdlog::info("Goal reached");
            return;
        }

        loop_rate.sleep();
    }
}

}  // namespace vortex::guidance
