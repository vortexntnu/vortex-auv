#include <reference_filter_dp/reference_filter_ros.hpp>

ReferenceFilterNode::ReferenceFilterNode()
    : LifecycleNode("reference_filter_node") {
    time_step_ = std::chrono::milliseconds(10);

    set_refererence_filter();
    x_ = Vector18d::Zero();
}

LifecycleCallbackReturn ReferenceFilterNode::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
    RCLCPP_INFO(this->get_logger(), "configuration step");
    (void)previous_state;

    set_subscribers_and_publisher();
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn ReferenceFilterNode::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
    RCLCPP_INFO(this->get_logger(), "activation step");
    (void)previous_state;

    set_action_server();
    rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);

    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn ReferenceFilterNode::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    RCLCPP_INFO(this->get_logger(), "deactivation step");
    
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (goal_handle_ && goal_handle_->is_active()) {
            RCLCPP_INFO(this->get_logger(), "Canceling active goal during deactivation");
            auto cancel_response = handle_cancel(goal_handle_);
        }
    }

    destroy_action_server();
    rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn ReferenceFilterNode::on_cleanup(
    const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    RCLCPP_INFO(this->get_logger(), "cleanup step");

    destroy_topics_and_publishers();
    rclcpp_lifecycle::LifecycleNode::on_cleanup(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn ReferenceFilterNode::on_shutdown(
    const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    RCLCPP_INFO(this->get_logger(), "shutdown step");
    
    destroy_action_server();
    destroy_topics_and_publishers();

    rclcpp_lifecycle::LifecycleNode::on_shutdown(previous_state);
    return LifecycleCallbackReturn::SUCCESS;
}

void ReferenceFilterNode::set_subscribers_and_publisher() {
    this->declare_parameter<std::string>("topics.namespace");
    this->declare_parameter<std::string>("topics.pose");
    this->declare_parameter<std::string>("topics.twist");
    this->declare_parameter<std::string>("topics.guidance.dp");
    this->declare_parameter<std::string>("topics.aruco_board_pose_camera");

    std::string ns = this->get_parameter("topics.namespace").as_string();
    std::string pose_topic =
        ns + this->get_parameter("topics.pose").as_string();
    std::string twist_topic =
        ns + this->get_parameter("topics.twist").as_string();
    std::string guidance_topic =
        ns + this->get_parameter("topics.guidance.dp").as_string();
    std::string aruco_board_pose_camera_topic =
        ns + this->get_parameter("topics.aruco_board_pose_camera").as_string();

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);
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

void ReferenceFilterNode::destroy_topics_and_publishers() {
    this->reference_pub_.reset();
    this->reference_sub_.reset();
    this->pose_sub_.reset();
    this->twist_sub_.reset();
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

void ReferenceFilterNode::destroy_action_server() {
    this->action_server_.reset();
    this->cb_group_.reset();
}

void ReferenceFilterNode::set_refererence_filter() {
    this->declare_parameter<std::vector<double>>(
        "zeta", {0.707, 0.707, 0.707, 0.707, 0.707, 0.707});
    this->declare_parameter<std::vector<double>>(
        "omega", {0.1, 0.1, 0.1, 0.1, 0.1, 0.1});

    std::vector<double> zeta = this->get_parameter("zeta").as_double_array();
    std::vector<double> omega = this->get_parameter("omega").as_double_array();

    const Vector6d zeta_eigen = Eigen::Map<Vector6d>(zeta.data());
    const Vector6d omega_eigen = Eigen::Map<Vector6d>(omega.data());

    reference_filter_.set_delta(zeta_eigen);
    reference_filter_.set_omega(omega_eigen);
    
    reference_filter_.calculate_Ad();
    reference_filter_.calculate_Bd();
}


void ReferenceFilterNode::reference_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double z = msg->pose.position.z;

    tf2::Quaternion q;
    q.setX(msg->pose.orientation.x);
    q.setY(msg->pose.orientation.y);
    q.setZ(msg->pose.orientation.z);
    q.setW(msg->pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

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
                RCLCPP_INFO(this->get_logger(),
                            "Aborting current goal and accepting new goal");
                preempted_goal_id_ = goal_handle_->get_goal_id();
            }
        }
    }
    RCLCPP_INFO(this->get_logger(), "Accepted goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ReferenceFilterNode::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ReferenceFilterNode::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle) {
    execute(goal_handle);
}

Vector18d ReferenceFilterNode::fill_reference_state() {
    Vector18d x = Vector18d::Zero();
    x(0) = current_pose_.pose.pose.position.x;
    x(1) = current_pose_.pose.pose.position.y;
    x(2) = current_pose_.pose.pose.position.z;

    tf2::Quaternion q;
    tf2::fromMsg(current_pose_.pose.pose.orientation, q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    x(3) = ssa(roll);
    x(4) = ssa(pitch);
    x(5) = ssa(yaw);

    Vector6d eta;
    eta << current_pose_.pose.pose.position.x,
        current_pose_.pose.pose.position.y, current_pose_.pose.pose.position.z,
        roll, pitch, yaw;
    Matrix6d J = calculate_J(eta);
    Vector6d nu;
    nu << current_twist_.twist.twist.linear.x,
        current_twist_.twist.twist.linear.y,
        current_twist_.twist.twist.linear.z,
        current_twist_.twist.twist.angular.x,
        current_twist_.twist.twist.angular.y,
        current_twist_.twist.twist.angular.z;
    Vector6d eta_dot = J * nu;

    x(6) = eta_dot(0);
    x(7) = eta_dot(1);
    x(8) = eta_dot(2);
    x(9) = eta_dot(3);
    x(10) = eta_dot(4);
    x(11) = eta_dot(5);

    return x;
}

Vector6d ReferenceFilterNode::fill_reference_goal(
    const geometry_msgs::msg::PoseStamped& goal) {
    double x = goal.pose.position.x;
    double y = goal.pose.position.y;
    double z = goal.pose.position.z;

    tf2::Quaternion q_goal;
    tf2::fromMsg(goal.pose.orientation, q_goal);

    tf2::Matrix3x3 m_goal(q_goal);
    double roll_goal, pitch_goal, yaw_goal;
    m_goal.getRPY(roll_goal, pitch_goal, yaw_goal);

    Vector6d r;
    r << x, y, z, roll_goal, pitch_goal, yaw_goal;

    return r;
}

vortex_msgs::msg::ReferenceFilter ReferenceFilterNode::fill_reference_msg() {
    vortex_msgs::msg::ReferenceFilter feedback_msg;
    feedback_msg.x = x_(0);
    feedback_msg.y = x_(1);
    feedback_msg.z = x_(2);
    feedback_msg.roll = ssa(x_(3));
    feedback_msg.pitch = ssa(x_(4));
    feedback_msg.yaw = ssa(x_(5));
    feedback_msg.x_dot = x_(6);
    feedback_msg.y_dot = x_(7);
    feedback_msg.z_dot = x_(8);
    feedback_msg.roll_dot = x_(9);
    feedback_msg.pitch_dot = x_(10);
    feedback_msg.yaw_dot = x_(11);

    return feedback_msg;
}

void ReferenceFilterNode::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        vortex_msgs::action::ReferenceFilterWaypoint>> goal_handle) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        this->goal_handle_ = goal_handle;
    }

    RCLCPP_INFO(this->get_logger(), "Executing goal");

    x_ = fill_reference_state();

    const geometry_msgs::msg::PoseStamped goal = goal_handle->get_goal()->goal;

    r_ = fill_reference_goal(goal);

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
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }
        Vector18d x_dot = reference_filter_.calculate_x_dot(x_, r_);
        x_ += x_dot * time_step_.count() / 1000.0;

        vortex_msgs::msg::ReferenceFilter feedback_msg = fill_reference_msg();

        feedback->feedback = feedback_msg;

        goal_handle->publish_feedback(feedback);
        reference_pub_->publish(feedback_msg);

        if ((x_.head(6) - r_.head(6)).norm() < 0.1) {
            result->success = true;
            goal_handle->succeed(result);
            x_.head(6) = r_.head(6);
            vortex_msgs::msg::ReferenceFilter feedback_msg =
                fill_reference_msg();
            reference_pub_->publish(feedback_msg);
            RCLCPP_INFO(this->get_logger(), "Goal reached");
            return;
        }

        loop_rate.sleep();
    }
}
