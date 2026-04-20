#include "reference_filter_dp_quat/ros/reference_filter_ros.hpp"
#include <spdlog/spdlog.h>
#include <mutex>
#include <rclcpp_components/register_node_macro.hpp>
#include <thread>
#include <vortex/utils/ros/qos_profiles.hpp>
#include <vortex/utils/ros/waypoint_ros_conversions.hpp>
#include "reference_filter_dp_quat/ros/reference_filter_ros_utils.hpp"

const auto start_message = R"(
██████╗ ███████╗███████╗███████╗██████╗ ███████╗███╗   ██╗ ██████╗███████╗    ███████╗██╗██╗  ████████╗███████╗██████╗      ██████╗ ██╗   ██╗ █████╗ ████████╗
██╔══██╗██╔════╝██╔════╝██╔════╝██╔══██╗██╔════╝████╗  ██║██╔════╝██╔════╝    ██╔════╝██║██║  ╚══██╔══╝██╔════╝██╔══██╗    ██╔═══██╗██║   ██║██╔══██╗╚══██╔══╝
██████╔╝█████╗  █████╗  █████╗  ██████╔╝█████╗  ██╔██╗ ██║██║     █████╗      █████╗  ██║██║     ██║   █████╗  ██████╔╝    ██║   ██║██║   ██║███████║   ██║
██╔══██╗██╔══╝  ██╔══╝  ██╔══╝  ██╔══██╗██╔══╝  ██║╚██╗██║██║     ██╔══╝      ██╔══╝  ██║██║     ██║   ██╔══╝  ██╔══██╗    ██║▄▄ ██║██║   ██║██╔══██║   ██║
██║  ██║███████╗██║     ███████╗██║  ██║███████╗██║ ╚████║╚██████╗███████╗    ██║     ██║███████╗██║   ███████╗██║  ██║    ╚██████╔╝╚██████╔╝██║  ██║   ██║
╚═╝  ╚═╝╚══════╝╚═╝     ╚══════╝╚═╝  ╚═╝╚══════╝╚═╝  ╚═══╝ ╚═════╝╚══════╝    ╚═╝     ╚═╝╚══════╝╚═╝   ╚══════╝╚═╝  ╚═╝     ╚══▀▀═╝  ╚═════╝ ╚═╝  ╚═╝   ╚═╝
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
    shutdown_.store(true);
    if (stepping_thread_.joinable()) {
        stepping_thread_.join();
    }
}

void ReferenceFilterNode::set_subscribers_and_publisher() {
    this->declare_parameter<std::string>("topics.pose");
    this->declare_parameter<std::string>("topics.twist");
    this->declare_parameter<std::string>("topics.guidance.dp_quat");
    this->declare_parameter<std::string>("topics.reference_pose");

    std::string pose_topic = this->get_parameter("topics.pose").as_string();
    std::string twist_topic = this->get_parameter("topics.twist").as_string();
    std::string guidance_topic =
        this->get_parameter("topics.guidance.dp_quat").as_string();
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

    action_server_ =
        rclcpp_action::create_server<vortex_msgs::action::GuidanceWaypoint>(
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
    std::shared_ptr<const vortex_msgs::action::GuidanceWaypoint::Goal>
    /*goal*/) {
    spdlog::info("Accepted goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ReferenceFilterNode::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::GuidanceWaypoint>> /*goal_handle*/) {
    spdlog::info("Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ReferenceFilterNode::handle_accepted(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::GuidanceWaypoint>>
        goal_handle) {
    const auto wp = vortex::utils::waypoints::waypoint_from_ros(
        goal_handle->get_goal()->waypoint);

    double threshold = goal_handle->get_goal()->convergence_threshold;
    if (threshold <= 0.0) {
        threshold = 0.1;
        spdlog::warn(
            "ReferenceFilter: invalid convergence_threshold (<= 0), using 0.1");
    }

    /**
     * Swap the active goal handle atomically and abort the outgoing one
     * so the ROS action state stays clean. The previous goal is aborted
     * rather than canceled because the client did not request cancellation.
     */
    std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::GuidanceWaypoint>>
        previous;
    {
        std::lock_guard<std::mutex> lock(active_goal_mutex_);
        previous = std::move(active_goal_handle_);
        active_goal_handle_ = goal_handle;
    }
    if (previous && previous->is_active()) {
        auto r =
            std::make_shared<vortex_msgs::action::GuidanceWaypoint::Result>();
        r->success = false;
        try {
            previous->abort(r);
        } catch (...) {
        }
        spdlog::info("Previous goal preempted");
    }

    /**
     * Cold-start vs retarget branching. On the first goal ever received,
     * initialize the filter from the measured pose and twist. On every
     * subsequent goal, only re-point the setpoint; the filter dynamics
     * carry the state forward.
     */
    if (!filter_initialized_.load()) {
        const auto [pose, twist] = [this] {
            std::lock_guard<std::mutex> lock(sensor_mutex_);
            return std::pair{current_pose_, current_twist_};
        }();
        follower_->start(pose, twist, wp, threshold);
        filter_initialized_.store(true);
        start_stepping_thread();
        spdlog::info("Filter cold-started on first goal");
    } else {
        follower_->retarget(wp, threshold);
        spdlog::info("Retargeted to new waypoint (filter state preserved)");
    }
}

void ReferenceFilterNode::start_stepping_thread() {
    if (stepping_thread_.joinable())
        return;
    stepping_thread_ = std::thread([this] { stepping_loop(); });
}

void ReferenceFilterNode::stepping_loop() {
    rclcpp::Rate loop_rate(1000.0 / time_step_.count());

    while (rclcpp::ok() && !shutdown_.load()) {
        follower_->step();

        auto msg = fill_reference_msg(follower_->pose(), follower_->velocity());
        reference_pub_->publish(msg);
        if (rpy_debug_pub_) {
            rpy_debug_pub_->publish(fill_reference_rpy_msg(
                follower_->pose(), follower_->velocity()));
        }

        /**
         * Snapshot the active goal handle under the mutex and then operate
         * on the local copy. This avoids holding the mutex across the
         * convergence check and the ROS action callbacks.
         */
        std::shared_ptr<rclcpp_action::ServerGoalHandle<
            vortex_msgs::action::GuidanceWaypoint>>
            gh;
        {
            std::lock_guard<std::mutex> lock(active_goal_mutex_);
            gh = active_goal_handle_;
        }

        if (gh && gh->is_active()) {
            const auto current_pose = [this] {
                std::lock_guard<std::mutex> lock(sensor_mutex_);
                return current_pose_;
            }();

            if (follower_->within_convergance(current_pose)) {
                follower_->snap_state_to_reference();
                auto final_msg = fill_reference_msg(follower_->pose(),
                                                    follower_->velocity());
                reference_pub_->publish(final_msg);
                if (rpy_debug_pub_) {
                    rpy_debug_pub_->publish(fill_reference_rpy_msg(
                        follower_->pose(), follower_->velocity()));
                }

                auto r = std::make_shared<
                    vortex_msgs::action::GuidanceWaypoint::Result>();
                r->success = true;
                gh->succeed(r);
                {
                    std::lock_guard<std::mutex> lock(active_goal_mutex_);
                    /**
                     * Only clear the stored handle if it is still the one
                     * we just succeeded. A concurrent handle_accepted could
                     * have already replaced it.
                     */
                    if (active_goal_handle_ == gh)
                        active_goal_handle_.reset();
                }
                spdlog::info("Goal reached");
            }
        }
        loop_rate.sleep();
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ReferenceFilterNode)

}  // namespace vortex::guidance
