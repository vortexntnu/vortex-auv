#include <pose_action_server/pose_action_server_ros.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

PoseActionServerNode::PoseActionServerNode() : Node("pose_action_server_node") {
    std::string pose_sub_topic =
        this->declare_parameter<std::string>("pose_sub_topic");
    std::string action_name = this->declare_parameter<std::string>("action_name");

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
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const vortex_msgs::action::FilteredPose::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received request to filter pose.");
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

    if (pose_queue_.size() < num_measurements_)
        return;

    geometry_msgs::msg::PoseStamped filtered_pose;
    filtered_pose.header = pose_msg->header;

    auto compute_mean =
        [](const std::vector<geometry_msgs::msg::PoseStamped>& pose_queue,
           auto position_extractor) {
            return std::accumulate(pose_queue.begin(), pose_queue.end(), 0.0,
                                   [&](double sum, const auto& pose) {
                                       return sum + position_extractor(pose);
                                   }) /
                   pose_queue.size();
        };

    filtered_pose.pose.position.x = compute_mean(
        pose_queue_, [](const auto& pose) { return pose.pose.position.x; });
    filtered_pose.pose.position.y = compute_mean(
        pose_queue_, [](const auto& pose) { return pose.pose.position.y; });
    filtered_pose.pose.position.z = compute_mean(
        pose_queue_, [](const auto& pose) { return pose.pose.position.z; });

    auto extract_quaternions =
        [](const std::vector<geometry_msgs::msg::PoseStamped>& pose_queue) {
            std::vector<Eigen::Quaterniond> quaternions;
            std::transform(pose_queue.begin(), pose_queue.end(),
                           std::back_inserter(quaternions),
                           [](const auto& pose) {
                               Eigen::Quaterniond q;
                               q.x() = pose.pose.orientation.x;
                               q.y() = pose.pose.orientation.y;
                               q.z() = pose.pose.orientation.z;
                               q.w() = pose.pose.orientation.w;
                               return q;
                           });
            return quaternions;
        };

    const std::vector<Eigen::Quaterniond> quaternions = extract_quaternions(pose_queue_);

    Eigen::Quaterniond mean_q = average_quaternions(quaternions);

    filtered_pose.pose.orientation.x = mean_q.x();
    filtered_pose.pose.orientation.y = mean_q.y();
    filtered_pose.pose.orientation.z = mean_q.z();
    filtered_pose.pose.orientation.w = mean_q.w();

    auto result = std::make_shared<vortex_msgs::action::FilteredPose::Result>();
    result->filtered_pose = filtered_pose;
    active_goal_handle_->succeed(result);
    is_executing_action_ = false;
}

Eigen::Quaterniond PoseActionServerNode::average_quaternions(
    const std::vector<Eigen::Quaterniond>& quaternions) {
    Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
    for (const auto& q : quaternions) {
        M += q.coeffs() * q.coeffs().transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(M);

    Eigen::Vector4d eigenvector = eigensolver.eigenvectors().col(3);

    Eigen::Quaterniond avg_q;
    avg_q.x() = eigenvector(0);
    avg_q.y() = eigenvector(1);
    avg_q.z() = eigenvector(2);
    avg_q.w() = eigenvector(3);

    return avg_q.normalized();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseActionServerNode>());
    rclcpp::shutdown();
    return 0;
}
