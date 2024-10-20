#include <reference_filter_dp/reference_filter_ros.hpp>

ReferenceFilterNode::ReferenceFilterNode()
    : Node("reference_filter_node") {
    time_step_ = std::chrono::milliseconds(10);

    this->declare_parameter<std::string>("reference_filter_topic", "/reference_topic");
    std::string reference_filter_topic = this->get_parameter("reference_filter_topic").as_string();
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    reference_pub_ = this->create_publisher<vortex_msgs::msg::ReferenceFilter>("/dp/reference", 10);
    reference_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(reference_filter_topic, qos_sensor_data, std::bind(&ReferenceFilterNode::reference_callback, this, std::placeholders::_1));
    // reference_pub_timer_ = this->create_wall_timer(time_step_, std::bind(&ReferenceFilterNode::reference_publisher_callback, this));
    
    set_refererence_filter();

    action_server_ = rclcpp_action::create_server<vortex_msgs::action::ReferenceFilterWaypoint>(
        this,
        "reference_filter",
        std::bind(&ReferenceFilterNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ReferenceFilterNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&ReferenceFilterNode::handle_accepted, this, std::placeholders::_1)
    );

    x_ = Vector18d::Zero();
}

void ReferenceFilterNode::set_refererence_filter() {
    this->declare_parameter<std::vector<double>>("zeta", {0.707, 0.707, 0.707, 0.707, 0.707, 0.707});
    this->declare_parameter<std::vector<double>>("omega", {0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
    
    std::vector<double> zeta = this->get_parameter("zeta").as_double_array();
    std::vector<double> omega = this->get_parameter("omega").as_double_array();

    const Vector6d zeta_eigen = Eigen::Map<Vector6d>(zeta.data());
    const Vector6d omega_eigen = Eigen::Map<Vector6d>(omega.data());

    reference_filter_.set_delta(zeta_eigen);
    reference_filter_.set_omega(omega_eigen);

    reference_filter_.calculate_Ad();
    reference_filter_.calculate_Bd();

}


void ReferenceFilterNode::reference_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double z = msg->pose.position.z;

    Eigen::Quaterniond quat(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d euler_angles = quaternion_to_euler(quat);

    r_ << x, y, z, euler_angles(0), euler_angles(1), euler_angles(2);
}

rclcpp_action::GoalResponse ReferenceFilterNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const vortex_msgs::action::ReferenceFilterWaypoint::Goal> goal) {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ReferenceFilterNode::handle_cancel(
    const std::shared_ptr<GoalHandleReferenceFilter> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ReferenceFilterNode::handle_accepted(const std::shared_ptr<GoalHandleReferenceFilter> goal_handle) {
    std::thread{std::bind(&ReferenceFilterNode::execute, this, goal_handle)}.detach();
}

void ReferenceFilterNode::execute(const std::shared_ptr<GoalHandleReferenceFilter> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    auto feedback = std::make_shared<vortex_msgs::action::ReferenceFilterWaypoint::Feedback>();
    auto result = std::make_shared<vortex_msgs::action::ReferenceFilterWaypoint::Result>();

    rclcpp::Rate loop_rate(1000.0 / time_step_.count());

    while (rclcpp::ok()) {
        Vector18d x_dot = reference_filter_.calculate_x_dot(x_, r_);
        x_ += x_dot * time_step_.count() / 1000.0;

        feedback->x = x_(0);
        feedback->y = x_(1);
        feedback->z = x_(2);
        feedback->roll = x_(3);
        feedback->pitch = x_(4);
        feedback->yaw = x_(5);

        goal_handle->publish_feedback(feedback);

        if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }

        loop_rate.sleep();
    }

    result->x = x_(0);
    result->y = x_(1);
    result->z = x_(2);
    result->roll = x_(3);
    result->pitch = x_(4);
    result->yaw = x_(5);
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
}



void ReferenceFilterNode::reference_publisher_callback() {
    Vector18d x_dot = reference_filter_.calculate_x_dot(x_, r_);

    x_ += x_dot * time_step_.count() / 1000.0;

    vortex_msgs::msg::ReferenceFilter reference_msg;
    
    reference_msg.x = x_(0);
    reference_msg.y = x_(1);
    reference_msg.z = x_(2);
    reference_msg.roll = x_(3);
    reference_msg.pitch = x_(4);
    reference_msg.yaw = x_(5);
    reference_msg.x_dot = x_(6);
    reference_msg.y_dot = x_(7);
    reference_msg.z_dot = x_(8);
    reference_msg.roll_dot = x_(9);
    reference_msg.pitch_dot = x_(10);
    reference_msg.yaw_dot = x_(11);

    reference_pub_->publish(reference_msg);
}

