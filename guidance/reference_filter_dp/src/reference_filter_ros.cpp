#include "reference_filter_dp/reference_filter_ros.hpp"
#include <spdlog/spdlog.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <string_view>
#include <vortex/utils/math.hpp>
#include <vortex/utils/qos_profiles.hpp>
#include <vortex/utils/types.hpp>

const auto start_message = R"(
  ____       __                                _____ _ _ _
 |  _ \ ___ / _| ___ _ __ ___ _ __   ___ ___  |  ___(_) | |_ ___ _ __
 | |_) / _ \ |_ / _ \ '__/ _ \ '_ \ / __/ _ \ | |_  | | | __/ _ \ '__|
 |  _ <  __/  _|  __/ | |  __/ | | | (_|  __/ |  _| | | | ||  __/ |
 |_| \_\___|_|  \___|_|  \___|_| |_|\___\___| |_|   |_|_|\__\___|_|

 )";

namespace vortex::guidance {

ReferenceFilterNode::ReferenceFilterNode(const rclcpp::NodeOptions& options)
    : Node("reference_filter_node", options) {
    time_step_ = std::chrono::milliseconds(10);

    set_subscribers_and_publisher();

    set_action_server();

    set_refererence_filter();

    spdlog::info(start_message);
}

void ReferenceFilterNode::set_subscribers_and_publisher() {
    this->declare_parameter<std::string>("topics.pose");
    this->declare_parameter<std::string>("topics.twist");
    this->declare_parameter<std::string>("topics.guidance.dp");
    this->declare_parameter<std::string>("topics.aruco_board_pose_camera");

    std::string pose_topic = this->get_parameter("topics.pose").as_string();
    std::string twist_topic = this->get_parameter("topics.twist").as_string();
    std::string guidance_topic =
        this->get_parameter("topics.guidance.dp").as_string();
    std::string aruco_board_pose_camera_topic =
        this->get_parameter("topics.aruco_board_pose_camera").as_string();

    auto qos_sensor_data = vortex::utils::qos_profiles::sensor_data_profile(1);
    reference_pub_ = this->create_publisher<vortex_msgs::msg::ReferenceFilter>(
        guidance_topic, qos_sensor_data);
    reference_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        aruco_board_pose_camera_topic, qos_sensor_data,
        std::bind(&ReferenceFilterNode::reference_callback, this,
                  std::placeholders::_1));
    pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, qos_sensor_data,
        std::bind(&ReferenceFilterNode::pose_callback, this,
                  std::placeholders::_1));
    twist_sub_ = this->create_subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>(
        twist_topic, qos_sensor_data,
        std::bind(&ReferenceFilterNode::twist_callback, this,
                  std::placeholders::_1));
}

void ReferenceFilterNode::set_action_server() {
    this->declare_parameter<std::string>("action_servers.reference_filter");
    std::string action_server_name =
        this->get_parameter("action_servers.reference_filter").as_string();
    cb_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    action_server_ = rclcpp_action::create_server<
        vortex_msgs::action::ReferenceFilterWaypoint>(
        this, action_server_name,
        std::bind(&ReferenceFilterNode::handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&ReferenceFilterNode::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&ReferenceFilterNode::handle_accepted, this,
                  std::placeholders::_1),
        rcl_action_server_get_default_options(), cb_group_);
}

void ReferenceFilterNode::set_refererence_filter() {
    this->declare_parameter<std::vector<double>>("zeta");
    this->declare_parameter<std::vector<double>>("omega");

    std::vector<double> zeta = this->get_parameter("zeta").as_double_array();
    std::vector<double> omega = this->get_parameter("omega").as_double_array();

    Eigen::Vector6d zeta_eigen = Eigen::Map<Eigen::Vector6d>(zeta.data());
    Eigen::Vector6d omega_eigen = Eigen::Map<Eigen::Vector6d>(omega.data());

    ReferenceFilterParams filter_params{omega_eigen, zeta_eigen};
    reference_filter_ = std::make_unique<ReferenceFilter>(filter_params);
}

void ReferenceFilterNode::reference_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double z = msg->pose.position.z;
    const auto& o = msg->pose.orientation;
    Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
    Eigen::Vector3d euler_angles = vortex::utils::math::quat_to_euler(q);
    double roll{euler_angles(0)};
    double pitch{euler_angles(1)};
    double yaw{euler_angles(2)};
    r_ << x, y, z, roll, pitch, yaw;
}

void ReferenceFilterNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    current_pose_ = *msg;
}

void ReferenceFilterNode::twist_callback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    current_twist_ = *msg;
}

rclcpp_action::GoalResponse ReferenceFilterNode::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const vortex_msgs::action::ReferenceFilterWaypoint::Goal>
        goal) {
    (void)uuid;
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

rclcpp_action::CancelResponse ReferenceFilterNode::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle) {
    spdlog::info("Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ReferenceFilterNode::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle) {
    execute(goal_handle);
}

Eigen::Vector18d ReferenceFilterNode::fill_reference_state() {
    Eigen::Vector18d x = Eigen::Vector18d::Zero();
    x(0) = current_pose_.pose.pose.position.x;
    x(1) = current_pose_.pose.pose.position.y;
    x(2) = current_pose_.pose.pose.position.z;

    const auto& o = current_pose_.pose.pose.orientation;
    Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
    Eigen::Vector3d euler_angles = vortex::utils::math::quat_to_euler(q);
    double roll{euler_angles(0)};
    double pitch{euler_angles(1)};
    double yaw{euler_angles(2)};

    x(3) = vortex::utils::math::ssa(roll);
    x(4) = vortex::utils::math::ssa(pitch);
    x(5) = vortex::utils::math::ssa(yaw);

    vortex::utils::types::Eta eta{current_pose_.pose.pose.position.x,
                                  current_pose_.pose.pose.position.y,
                                  current_pose_.pose.pose.position.z,
                                  roll,
                                  pitch,
                                  yaw};
    Eigen::Matrix6d J = eta.as_j_matrix();
    vortex::utils::types::Nu nu{current_twist_.twist.twist.linear.x,
                                current_twist_.twist.twist.linear.y,
                                current_twist_.twist.twist.linear.z,
                                current_twist_.twist.twist.angular.x,
                                current_twist_.twist.twist.angular.y,
                                current_twist_.twist.twist.angular.z};
    Eigen::Vector6d eta_dot = J * nu.to_vector();

    x(6) = eta_dot(0);
    x(7) = eta_dot(1);
    x(8) = eta_dot(2);
    x(9) = eta_dot(3);
    x(10) = eta_dot(4);
    x(11) = eta_dot(5);

    return x;
}

Eigen::Vector6d ReferenceFilterNode::fill_reference_goal(
    const geometry_msgs::msg::Pose& goal) {
    double x{goal.position.x};
    double y{goal.position.y};
    double z{goal.position.z};

    const auto& o = goal.orientation;
    Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
    Eigen::Vector3d euler_angles = vortex::utils::math::quat_to_euler(q);
    double roll{euler_angles(0)};
    double pitch{euler_angles(1)};
    double yaw{euler_angles(2)};

    Eigen::Vector6d r;
    r << x, y, z, roll, pitch, yaw;

    return r;
}

vortex_msgs::msg::ReferenceFilter ReferenceFilterNode::fill_reference_msg() {
    vortex_msgs::msg::ReferenceFilter feedback_msg;
    feedback_msg.x = x_(0);
    feedback_msg.y = x_(1);
    feedback_msg.z = x_(2);
    feedback_msg.roll = vortex::utils::math::ssa(x_(3));
    feedback_msg.pitch = vortex::utils::math::ssa(x_(4));
    feedback_msg.yaw = vortex::utils::math::ssa(x_(5));
    feedback_msg.x_dot = x_(6);
    feedback_msg.y_dot = x_(7);
    feedback_msg.z_dot = x_(8);
    feedback_msg.roll_dot = x_(9);
    feedback_msg.pitch_dot = x_(10);
    feedback_msg.yaw_dot = x_(11);

    return feedback_msg;
}

Eigen::Vector6d ReferenceFilterNode::apply_mode_logic(
    const Eigen::Vector6d& r_in, uint8_t mode)
{
    Eigen::Vector6d r_out = r_in;

    switch (mode)
    {
        case vortex_msgs::msg::Waypoint::FULL_POSE:
            break;

        case vortex_msgs::msg::Waypoint::ONLY_POSITION:
            r_out(3) = x_(3);
            r_out(4) = x_(4);
            r_out(5) = x_(5);
            break;

        case vortex_msgs::msg::Waypoint::FORWARD_HEADING:
        {
            double dx = r_in(0) - x_(0);
            double dy = r_in(1) - x_(1);

            double forward_yaw = std::atan2(dy, dx);

            r_out(3) = 0.0;
            r_out(4) = 0.0;
            r_out(5) = vortex::utils::math::ssa(forward_yaw);
            break;
        }

        case vortex_msgs::msg::Waypoint::ONLY_ORIENTATION:
            r_out(0) = x_(0);
            r_out(1) = x_(1);
            r_out(2) = x_(2);
            break;
    }

    return r_out;
}

void ReferenceFilterNode::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        this->goal_handle_ = goal_handle;
    }

    spdlog::info("Executing goal");

    x_ = fill_reference_state();

    const geometry_msgs::msg::Pose goal = goal_handle->get_goal()->waypoint.pose;
    uint8_t mode = goal_handle->get_goal()->waypoint.mode;

    Eigen::Vector6d r_temp = fill_reference_goal(goal);
    r_ = apply_mode_logic(r_temp, mode);

    auto feedback = std::make_shared<
        vortex_msgs::action::ReferenceFilterWaypoint::Feedback>();
    auto result = std::make_shared<
        vortex_msgs::action::ReferenceFilterWaypoint::Result>();

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
        Eigen::Vector18d x_dot = reference_filter_->calculate_x_dot(x_, r_);
        x_ += x_dot * time_step_.count() / 1000.0;

        vortex_msgs::msg::ReferenceFilter feedback_msg = fill_reference_msg();

        feedback->reference = feedback_msg;

        goal_handle->publish_feedback(feedback);
        reference_pub_->publish(feedback_msg);

        if ((x_.head(6) - r_.head(6)).norm() < 0.1) {
            result->success = true;
            goal_handle->succeed(result);
            x_.head(6) = r_.head(6);
            vortex_msgs::msg::ReferenceFilter feedback_msg =
                fill_reference_msg();
            reference_pub_->publish(feedback_msg);
            spdlog::info("Goal reached");
            return;
        }

        loop_rate.sleep();
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ReferenceFilterNode)

}  // namespace vortex::guidance
