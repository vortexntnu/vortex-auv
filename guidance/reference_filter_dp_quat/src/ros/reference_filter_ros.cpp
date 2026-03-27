#include "reference_filter_dp_quat/ros/reference_filter_ros.hpp"
#include <spdlog/spdlog.h>
#include <mutex>
#include <rclcpp_components/register_node_macro.hpp>
#include <thread>
#include <vortex/utils/ros/qos_profiles.hpp>
#include <vortex/utils/ros/waypoint_ros_conversions.hpp>
#include "reference_filter_dp_quat/ros/reference_filter_ros_utils.hpp"

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

ReferenceFilterNode::~ReferenceFilterNode() {
    preempted_ = true;
    if (execute_thread_.joinable()) {
        execute_thread_.join();
    }
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
    reference_pub_ =
        this->create_publisher<vortex_msgs::msg::ReferenceFilterQuat>(
            guidance_topic, qos_sensor_data);

    publish_rpy_debug_ = this->declare_parameter<bool>("publish_rpy_debug");
    if (publish_rpy_debug_) {
        std::string rpy_topic = this->declare_parameter<std::string>(
            "topics.guidance.dp_rpy", guidance_topic + "_rpy");
        rpy_debug_pub_ =
            this->create_publisher<vortex_msgs::msg::ReferenceFilter>(
                rpy_topic, qos_sensor_data);
        spdlog::info("RPY debug publisher enabled on topic: {}", rpy_topic);
    }

    reference_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        reference_pose_topic, qos_sensor_data,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            follower_->set_reference(
                vortex::utils::ros_conversions::ros_pose_to_pose(msg->pose));
        });

    pose_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        pose_topic, qos_sensor_data,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
                   msg) {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            current_pose_ = vortex::utils::ros_conversions::ros_pose_to_pose(
                msg->pose.pose);
        });

    twist_sub_ = this->create_subscription<
        geometry_msgs::msg::TwistWithCovarianceStamped>(
        twist_topic, qos_sensor_data,
        [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr
                   msg) {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            current_twist_ = vortex::utils::ros_conversions::ros_twist_to_twist(
                msg->twist.twist);
        });
}

void ReferenceFilterNode::set_action_server() {
    this->declare_parameter<std::string>("action_servers.reference_filter");
    std::string action_server_name =
        this->get_parameter("action_servers.reference_filter").as_string();

    action_server_ = rclcpp_action::create_server<
        vortex_msgs::action::ReferenceFilterQuatWaypoint>(
        this, action_server_name,
        [this](const auto& uuid, auto goal) {
            return handle_goal(uuid, std::move(goal));
        },
        [this](auto goal_handle) { return handle_cancel(goal_handle); },
        [this](auto goal_handle) { handle_accepted(goal_handle); });
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
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<
        const vortex_msgs::action::ReferenceFilterQuatWaypoint::Goal>
    /*goal*/) {
    spdlog::info("Accepted goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ReferenceFilterNode::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::ReferenceFilterQuatWaypoint>> /*goal_handle*/) {
    spdlog::info("Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ReferenceFilterNode::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::ReferenceFilterQuatWaypoint>> goal_handle) {
    std::lock_guard<std::mutex> lock(execute_mutex_);
    preempted_ = true;
    if (execute_thread_.joinable()) {
        execute_thread_.join();
    }
    preempted_ = false;

    execute_thread_ =
        std::thread([this, goal_handle]() { execute(goal_handle); });
}

void ReferenceFilterNode::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::ReferenceFilterQuatWaypoint>> goal_handle) {
    spdlog::info("Executing goal");

    double convergence_threshold =
        goal_handle->get_goal()->convergence_threshold;

    if (convergence_threshold <= 0.0) {
        convergence_threshold = 0.1;
        spdlog::warn(
            "ReferenceFilter: Invalid convergence_threshold received (<= 0). "
            "Using default 0.1");
    }

    const auto wp = vortex::utils::waypoints::waypoint_from_ros(goal_handle->get_goal()->waypoint);

    const auto [pose, twist] = [this] {
        std::lock_guard lock(sensor_mutex_);
        return std::pair{current_pose_, current_twist_};
    }();

    follower_->start(pose, twist, wp, convergence_threshold);

    auto result = std::make_shared<
        vortex_msgs::action::ReferenceFilterQuatWaypoint::Result>();

    rclcpp::Rate loop_rate(1000.0 / time_step_.count());

    while (rclcpp::ok()) {
        if (preempted_.load()) {
            result->success = false;
            goal_handle->abort(result);
            spdlog::info("Goal preempted by newer goal");
            return;
        }

        if (goal_handle->is_canceling()) {
            result->success = false;
            goal_handle->canceled(result);
            spdlog::info("Goal canceled");
            return;
        }

        follower_->step();

        const auto current_pose = [this] {
            std::lock_guard lock(sensor_mutex_);
            return current_pose_;
        }();

        bool target_reached = follower_->within_convergance(current_pose);

        if (target_reached) {
            follower_->snap_state_to_reference();

            auto final_reference_msg =
                fill_reference_msg(follower_->pose(), follower_->velocity());

            reference_pub_->publish(final_reference_msg);
            if (rpy_debug_pub_) {
                rpy_debug_pub_->publish(fill_reference_rpy_msg(
                    follower_->pose(), follower_->velocity()));
            }

            result->success = true;
            goal_handle->succeed(result);
            spdlog::info("Goal reached");
            return;
        }

        auto reference_msg =
            fill_reference_msg(follower_->pose(), follower_->velocity());
        reference_pub_->publish(reference_msg);
        if (rpy_debug_pub_) {
            rpy_debug_pub_->publish(fill_reference_rpy_msg(
                follower_->pose(), follower_->velocity()));
        }
        loop_rate.sleep();
    }
    if (!rclcpp::ok() && goal_handle->is_active()) {
        auto result = std::make_shared<
            vortex_msgs::action::ReferenceFilterQuatWaypoint::Result>();
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
