#include <cmath>
#include <guidance_vtf/guidance_vtf_ros.hpp>
#include <thread>
#include <vector>
#include "guidance_vtf/guidance_vtf_utils.hpp"
#include "guidance_vtf/typedefs.hpp"

GuidanceVTFNode::GuidanceVTFNode() : Node("guidance_vtf_node") {
    time_step_ = std::chrono::milliseconds(10);

    A_param_ = (-3 * M_PI * 1.3) / 25;

    A_ << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        A_param_, 0, 0, 0, 0, 0, 0, A_param_, 0, 0, 0, 0, 0, 0, A_param_;

    B_ << 0, 0, 0, 0, 0, 0, 0, 0, 0, 4.5, 0, 0, 0, 4.5, 0, 0, 0, 4.5;

    dt_ = 0.01;

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos_sensor_data = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    cb_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    this->declare_parameter<std::string>("reference_filter_topic",
                                         "/dp/reference");
    this->declare_parameter<std::string>("orca_odom_topic", "/orca/odom");

    std::string reference_filter_topic =
        this->get_parameter("reference_filter_topic").as_string();

    std::string orca_odom_topic =
        this->get_parameter("orca_odom_topic").as_string();

    reference_pub_ = this->create_publisher<vortex_msgs::msg::ReferenceFilter>(
        reference_filter_topic, qos_sensor_data);

    orca_odom_topic_ = this->create_subscription<nav_msgs::msg::Odometry>(
        orca_odom_topic, qos_sensor_data,
        std::bind(&GuidanceVTFNode::odometry_callback, this,
                  std::placeholders::_1));

    action_server_ =
        rclcpp_action::create_server<vortex_msgs::action::VtfGuidance>(
            this, "guidance_vtf",
            std::bind(&GuidanceVTFNode::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&GuidanceVTFNode::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&GuidanceVTFNode::handle_accepted, this,
                      std::placeholders::_1),
            rcl_action_server_get_default_options(), cb_group_);
}

rclcpp_action::GoalResponse GuidanceVTFNode::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const vortex_msgs::action::VtfGuidance::Goal> goal) {
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

rclcpp_action::CancelResponse GuidanceVTFNode::handle_cancel(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::VtfGuidance>>
        goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void GuidanceVTFNode::handle_accepted(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::VtfGuidance>>
        goal_handle) {
    execute(goal_handle);
}

vortex_msgs::msg::ReferenceFilter GuidanceVTFNode::fill_reference_msg() {
    vortex_msgs::msg::ReferenceFilter feedback_msg;
    feedback_msg.x = eta_.position.x();
    feedback_msg.y = eta_.position.y();
    feedback_msg.z = eta_.position.z();
    feedback_msg.roll = ssa(eta_.roll);
    feedback_msg.pitch = ssa(eta_.pitch);
    feedback_msg.yaw = ssa(eta_.yaw);
    feedback_msg.x_dot = eta_.velocity.x();
    feedback_msg.y_dot = eta_.velocity.y();
    feedback_msg.z_dot = eta_.velocity.z();
    feedback_msg.roll_dot = eta_.angular_velocity.x();
    feedback_msg.pitch_dot = eta_.angular_velocity.y();
    feedback_msg.yaw_dot = eta_.angular_velocity.z();

    return feedback_msg;
}

void GuidanceVTFNode::odometry_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    eta_drone_.position =
        Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y,
                 msg->pose.pose.position.z);
    eta_drone_.velocity =
        Vector3d(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
                 msg->twist.twist.linear.z);
    eta_drone_.angular_velocity =
        Vector3d(msg->twist.twist.angular.x, msg->twist.twist.angular.y,
                 msg->twist.twist.angular.z);

    tf2::Quaternion quat;
    quat.setX(msg->pose.pose.orientation.x);
    quat.setY(msg->pose.pose.orientation.y);
    quat.setZ(msg->pose.pose.orientation.z);
    quat.setW(msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    eta_drone_.roll = roll;
    eta_drone_.pitch = pitch;
    eta_drone_.yaw = yaw;
}

void GuidanceVTFNode::execute(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::VtfGuidance>>
        goal_handle) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        this->goal_handle_ = goal_handle;
    }

    RCLCPP_INFO(this->get_logger(), "Executing goal");

    // Define the same for the drone state
    eta_ = eta_drone_;
    path_index_ = 0.0;

    // Define the path for the algorithm
    const vortex_msgs::msg::Waypoints waypoints = goal_handle->get_goal()->goal;
    path_.waypoints.clear();

    path_.waypoints.push_back(Vector3d(eta_drone_.position.x(),
                                       eta_drone_.position.y(),
                                       eta_drone_.position.z()));

    for (size_t i = 0; i < waypoints.waypoints.size(); ++i) {
        path_.waypoints.push_back(Vector3d(waypoints.waypoints[i].x,
                                           waypoints.waypoints[i].y,
                                           waypoints.waypoints[i].z));
    }

    path_.generate_G1_path(0.3, 0.3);
    double target_yaw = 0;

    auto feedback =
        std::make_shared<vortex_msgs::action::VtfGuidance::Feedback>();
    auto result = std::make_shared<vortex_msgs::action::VtfGuidance::Result>();

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
        // RCLCPP_INFO(this->get_logger(), "Current path element: [%f, %f, %f]",
        // path_.path[path_index_].x(), path_.path[path_index_].y(),
        // path_.path[path_index_].z());
        Vector3d goal = path_.path[path_index_];
        Vector3d e_target = goal - eta_.position;
        Vector3d e_drone = eta_drone_.position - eta_.position;

        // Gains (choose these based on your desired performance)
        Matrix3d Kp = Matrix3d::Identity() * 15.0;
        Matrix3d Kd = Matrix3d::Identity() * 10.0;

        // Compute the desired acceleration using PD terms
        Vector3d a_des = Kp * e_target - Kd * eta_.velocity;

        // Compute the control input, canceling the natural decay of velocity
        Vector3d u =
            (a_des - (Matrix3d::Identity() * A_param_) * eta_.velocity) / 4.5;

        Vector6d eta_next = (A_ * eta_.get_pos_vel() * dt_) +
                            eta_.get_pos_vel() + (B_ * u * dt_);

        eta_.position = eta_next.head<3>();
        eta_.velocity = eta_next.tail<3>();

        // --- Velocity Saturation ---
        // Define your maximum allowed speed (e.g., 1.0 m/s)
        double max_speed = 0.2;

        // Check if the current speed exceeds max_speed and saturate if needed
        double current_speed = eta_.velocity.norm();
        if (current_speed > max_speed) {
            eta_.velocity = (max_speed / current_speed) * eta_.velocity;
        }

        // State_object direction_vector = eta_ - eta_drone_;
        // Vector3d direction = direction_vector.position;
        // double d_total = direction.norm();

        // Update angles using the error (this avoids direct interpolation
        // issues) eta_.yaw   += 0; // alpha_yaw * yaw_error; For roll, you can
        // continue to set it to zero or implement a similar strategy.
        eta_.pitch = 0.0;
        eta_.roll = 0.0;

        // Smoothly update yaw to be the angle between the current position and
        // the waypoint
        if (static_cast<int>(path_index_) % 200 == 0 &&
            static_cast<int>(path_index_) <
                static_cast<int>(path_.path.size())) {
            Vector3d waypoint = path_.path[static_cast<int>(path_index_)];
            double delta_x = waypoint.x() - eta_.position.x();
            double delta_y = waypoint.y() - eta_.position.y();
            target_yaw = std::atan2(delta_y, delta_x);
        }

        // Interpolate between current yaw and target yaw
        double alpha = 0.01;  // Smoothing factor (0.0 to 1.0)
        eta_.yaw = (1 - alpha) * eta_.yaw + alpha * target_yaw;

        // eta_.roll = roll;
        // eta_.pitch = pitch;
        // eta_.yaw = yaw;

        vortex_msgs::msg::ReferenceFilter feedback_msg = fill_reference_msg();

        feedback->feedback = feedback_msg;

        goal_handle->publish_feedback(feedback);
        reference_pub_->publish(feedback_msg);

        if ((e_target.norm() < 0.4) && (e_drone.norm() < 0.4)) {
            std::cout << "Moving to the next path element" << std::endl;
            std::cout << "Current path index: " << path_index_ << std::endl;
            path_index_ += 1.0;
        } else if (path_index_ >= path_.path.size()) {
            result->success = true;
            goal_handle->succeed(result);
            return;
        }

        loop_rate.sleep();
    }
}
