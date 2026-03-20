#include "reference_filter_dp/ros/reference_filter_ros.hpp"
#include <spdlog/spdlog.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <thread>
#include <vortex/utils/math.hpp>
#include <vortex/utils/ros/qos_profiles.hpp>
#include <vortex/utils/ros/ros_conversions.hpp>
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
    int time_step_ms = this->declare_parameter<int>("time_step_ms");
    time_step_ = std::chrono::milliseconds(time_step_ms);

    set_subscribers_and_publisher();

    set_action_server();

    set_refererence_filter();

    spdlog::info(start_message);
}

void ReferenceFilterNode::set_subscribers_and_publisher() {
    this->declare_parameter<std::string>("topics.pose");
    this->declare_parameter<std::string>("topics.twist");
    this->declare_parameter<std::string>("topics.guidance.dp");
    this->declare_parameter<std::string>("topics.reference_pose");

    std::string pose_topic = this->get_parameter("topics.pose").as_string();
    std::string twist_topic = this->get_parameter("topics.twist").as_string();
    std::string guidance_topic =
        this->get_parameter("topics.guidance.dp").as_string();
    std::string reference_pose_topic =
        this->get_parameter("topics.reference_pose").as_string();

    auto qos_sensor_data = vortex::utils::qos_profiles::sensor_data_profile(1);
    reference_pub_ = this->create_publisher<vortex_msgs::msg::ReferenceFilter>(
        guidance_topic, qos_sensor_data);
    reference_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        reference_pose_topic, qos_sensor_data,
        std::bind(&ReferenceFilterNode::reference_callback, this,
                  std::placeholders::_1));
    pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, qos_sensor_data,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
                   msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            current_pose_ =
                vortex::utils::ros_conversions::ros_pose_to_pose_euler(
                    msg->pose.pose);
        });
    twist_sub_ = this->create_subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>(
        twist_topic, qos_sensor_data,
        [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr
                   msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            current_twist_ = vortex::utils::ros_conversions::ros_twist_to_twist(
                msg->twist.twist);
        });
}

void ReferenceFilterNode::set_action_server() {
    this->declare_parameter<std::string>("action_servers.reference_filter");
    std::string action_server_name =
        this->get_parameter("action_servers.reference_filter").as_string();

    action_server_ = rclcpp_action::create_server<
        vortex_msgs::action::ReferenceFilterWaypoint>(
        this, action_server_name,
        std::bind(&ReferenceFilterNode::handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&ReferenceFilterNode::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&ReferenceFilterNode::handle_accepted, this,
                  std::placeholders::_1),
        rcl_action_server_get_default_options());
}

void ReferenceFilterNode::set_refererence_filter() {
    this->declare_parameter<std::vector<double>>("zeta");
    this->declare_parameter<std::vector<double>>("omega");

    std::vector<double> zeta = this->get_parameter("zeta").as_double_array();
    std::vector<double> omega = this->get_parameter("omega").as_double_array();

    Eigen::Vector6d zeta_eigen = Eigen::Map<Eigen::Vector6d>(zeta.data());
    Eigen::Vector6d omega_eigen = Eigen::Map<Eigen::Vector6d>(omega.data());

    filter_params_ = ReferenceFilterParams{omega_eigen, zeta_eigen};

    double dt_seconds = time_step_.count() / 1000.0;
    follower_ = std::make_unique<WaypointFollower>(filter_params_, dt_seconds);
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

    Eigen::Vector6d r_temp;
    r_temp << x, y, z, roll, pitch, yaw;

    auto mode = static_cast<WaypointMode>(active_mode_.load());
    follower_->set_reference(r_temp, mode);
}

rclcpp_action::GoalResponse ReferenceFilterNode::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const vortex_msgs::action::ReferenceFilterWaypoint::Goal>
        goal) {
    (void)uuid;
    (void)goal;
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
    const uint64_t my_generation = ++goal_generation_;
    std::thread([this, goal_handle, my_generation]() {
        execute(goal_handle, my_generation);
    }).detach();
}

Eigen::Vector6d ReferenceFilterNode::measured_pose_vector6() {
    std::lock_guard<std::mutex> lock(mutex_);
    vortex::utils::types::PoseEuler pose = current_pose_;
    pose.apply_ssa();
    return pose.to_vector();
}

vortex::utils::types::PoseEuler ReferenceFilterNode::fill_reference_goal(
    const geometry_msgs::msg::Pose& goal) {
    return vortex::utils::ros_conversions::ros_pose_to_pose_euler(goal);
}

vortex_msgs::msg::ReferenceFilter ReferenceFilterNode::fill_reference_msg() {
    vortex_msgs::msg::ReferenceFilter feedback_msg;
    const Eigen::Vector18d& x = follower_->state();
    feedback_msg.x = x(0);
    feedback_msg.y = x(1);
    feedback_msg.z = x(2);
    feedback_msg.roll = vortex::utils::math::ssa(x(3));
    feedback_msg.pitch = vortex::utils::math::ssa(x(4));
    feedback_msg.yaw = vortex::utils::math::ssa(x(5));
    feedback_msg.x_dot = x(6);
    feedback_msg.y_dot = x(7);
    feedback_msg.z_dot = x(8);
    feedback_msg.roll_dot = x(9);
    feedback_msg.pitch_dot = x(10);
    feedback_msg.yaw_dot = x(11);

    return feedback_msg;
}

void ReferenceFilterNode::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle,
    uint64_t generation) {
    spdlog::info("Executing goal");

    const geometry_msgs::msg::Pose goal =
        goal_handle->get_goal()->waypoint.pose;
    uint8_t mode = goal_handle->get_goal()->waypoint.mode;
    double convergence_threshold =
        goal_handle->get_goal()->convergence_threshold;

    if (convergence_threshold <= 0.0) {
        convergence_threshold = 0.1;
        spdlog::warn(
            "ReferenceFilter: Invalid convergence_threshold received (<= 0). "
            "Using default 0.1");
    }

    active_mode_ = mode;

    Waypoint wp;
    wp.pose = fill_reference_goal(goal);
    wp.mode = static_cast<WaypointMode>(mode);

    PoseEuler pose;
    Twist twist;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pose = current_pose_;
        twist = current_twist_;
    }
    follower_->start(pose, twist, wp, convergence_threshold);

    auto feedback = std::make_shared<
        vortex_msgs::action::ReferenceFilterWaypoint::Feedback>();
    auto result = std::make_shared<
        vortex_msgs::action::ReferenceFilterWaypoint::Result>();

    rclcpp::Rate loop_rate(1000.0 / time_step_.count());

    while (rclcpp::ok()) {
        if (generation != goal_generation_.load()) {
            result->success = false;
            if (goal_handle->is_canceling()) {
                goal_handle->canceled(result);
                spdlog::info("Goal canceled (superseded)");
            } else {
                goal_handle->abort(result);
                spdlog::info("Goal preempted by newer goal");
            }
            return;
        }

        if (goal_handle->is_canceling()) {
            result->success = false;
            goal_handle->canceled(result);
            spdlog::info("Goal canceled");
            return;
        }

        Eigen::Vector6d y = measured_pose_vector6();
        StepResult step = follower_->step(y);

        vortex_msgs::msg::ReferenceFilter feedback_msg = fill_reference_msg();

        feedback->reference = feedback_msg;

        goal_handle->publish_feedback(feedback);
        reference_pub_->publish(feedback_msg);

        if (step.converged) {
            result->success = true;
            goal_handle->succeed(result);
            follower_->snap_state_to_reference();
            vortex_msgs::msg::ReferenceFilter feedback_msg =
                fill_reference_msg();
            reference_pub_->publish(feedback_msg);
            spdlog::info("Goal reached");
            return;
        }

        loop_rate.sleep();
    }
    if (!rclcpp::ok() && goal_handle->is_active()) {
        auto result = std::make_shared<
            vortex_msgs::action::ReferenceFilterWaypoint::Result>();
        result->success = false;

        try {
            goal_handle->abort(result);
        } catch (...) {
            // Ignore exceptions during shutdown
        }
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ReferenceFilterNode)

}  // namespace vortex::guidance
