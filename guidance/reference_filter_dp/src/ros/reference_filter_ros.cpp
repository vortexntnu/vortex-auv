#include "reference_filter_dp/ros/reference_filter_ros.hpp"
#include <spdlog/spdlog.h>
#include <mutex>
#include <rclcpp_components/register_node_macro.hpp>
#include <thread>
#include <vortex/utils/ros/qos_profiles.hpp>
#include <vortex/utils/ros/ros_conversions.hpp>
#include "reference_filter_dp/ros/reference_filter_ros_utils.hpp"

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
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            follower_->set_reference(
                vortex::utils::ros_conversions::ros_pose_to_pose_euler(
                    msg->pose));
        });

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

void ReferenceFilterNode::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle,
    uint64_t generation) {
    spdlog::info("Executing goal");

    double convergence_threshold =
        goal_handle->get_goal()->convergence_threshold;

    if (convergence_threshold <= 0.0) {
        convergence_threshold = 0.1;
        spdlog::warn(
            "ReferenceFilter: Invalid convergence_threshold received (<= 0). "
            "Using default 0.1");
    }

    Waypoint wp = waypoint_from_ros(goal_handle->get_goal()->waypoint);

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

        Eigen::Vector6d current_pose_vector;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            current_pose_vector = current_pose_.to_vector();
        }

        StepResult step = follower_->step(current_pose_vector);

        vortex_msgs::msg::ReferenceFilter reference_msg =
            fill_reference_msg(follower_->state());

        feedback->reference = reference_msg;

        goal_handle->publish_feedback(feedback);
        reference_pub_->publish(reference_msg);

        if (step.converged) {
            result->success = true;
            goal_handle->succeed(result);
            follower_->snap_state_to_reference();
            vortex_msgs::msg::ReferenceFilter feedback_msg =
                fill_reference_msg(follower_->state());
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
