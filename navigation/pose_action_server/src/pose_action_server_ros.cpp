#include <pose_action_server/pose_action_server_ros.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

PoseActionServerNode::PoseActionServerNode() : Node("pose_action_server_node") {
    auto pose_sub_topic =
        this->declare_parameter<std::string>("pose_sub_topic");
    auto action_name = this->declare_parameter<std::string>("action_name");

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10))
                          .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_sub_topic, qos,
        std::bind(&PoseActionServerNode::pose_callback, this, _1));

    action_server_ =
        rclcpp_action::create_server<vortex_msgs::action::FilteredPose>(
            this, action_name,
            std::bind(&PoseActionServerNode::handleGoal, this, _1, _2),
            std::bind(&PoseActionServerNode::handleCancel, this, _1),
            std::bind(&PoseActionServerNode::handleAccepted, this, _1));
}

rclcpp_action::GoalResponse PoseActionServerNode::handleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const vortex_msgs::action::FilteredPose::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received request to filter pose.");
    (void)uuid;
    if (is_executing_action_) {
        RCLCPP_WARN(this->get_logger(),
                    "Already executing an action, rejecting new goal.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    is_executing_action_ = true;
    num_measurements_ = goal->num_measurements;
    pose_queue_.clear();
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PoseActionServerNode::handleCancel(
    const std::shared_ptr<GoalHandleFilteredPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    is_executing_action_ = false;
    active_goal_handle_.reset();
    pose_queue_.clear();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PoseActionServerNode::handleAccepted(
    const std::shared_ptr<GoalHandleFilteredPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Goal accepted. Processing poses.");
    is_executing_action_ = true;
    active_goal_handle_ = goal_handle;
}

void PoseActionServerNode::pose_callback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg) {
    if (!is_executing_action_)
        return;

    pose_queue_.push_back(*pose_msg);

    geometry_msgs::msg::PoseStamped feedback_msg = *pose_msg;
    auto feedback =
        std::make_shared<vortex_msgs::action::FilteredPose::Feedback>();
    feedback->current_pose = feedback_msg;
    active_goal_handle_->publish_feedback(feedback);

    if (pose_queue_.size() == num_measurements_) {
        geometry_msgs::msg::PoseStamped averaged_pose;
        averaged_pose.header = pose_msg->header;

        for (const auto& pose : pose_queue_) {
            averaged_pose.pose.position.x += pose.pose.position.x;
            averaged_pose.pose.position.y += pose.pose.position.y;
            averaged_pose.pose.position.z += pose.pose.position.z;
            averaged_pose.pose.orientation.x += pose.pose.orientation.x;
            averaged_pose.pose.orientation.y += pose.pose.orientation.y;
            averaged_pose.pose.orientation.z += pose.pose.orientation.z;
            averaged_pose.pose.orientation.w += pose.pose.orientation.w;
        }

        averaged_pose.pose.position.x /= num_measurements_;
        averaged_pose.pose.position.y /= num_measurements_;
        averaged_pose.pose.position.z /= num_measurements_;
        averaged_pose.pose.orientation.x /= num_measurements_;
        averaged_pose.pose.orientation.y /= num_measurements_;
        averaged_pose.pose.orientation.z /= num_measurements_;
        averaged_pose.pose.orientation.w /= num_measurements_;

        auto result =
            std::make_shared<vortex_msgs::action::FilteredPose::Result>();
        result->filtered_pose = averaged_pose;
        active_goal_handle_->succeed(result);
        is_executing_action_ = false;
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseActionServerNode>());
    rclcpp::shutdown();
    return 0;
}
