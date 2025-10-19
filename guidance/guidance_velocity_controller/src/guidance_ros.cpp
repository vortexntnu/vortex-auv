#include <guidance_velocity_controller/guidance_ros.hpp>

GuidanceNode::GuidanceNode() : Node("guidance_node") {
  establish_communications();
  setup_parameters();
  spdlog::info("LOS guidance node initialized");
}

void GuidanceNode::establish_communications() {
  this->declare_parameter<std::string>("topics.pose");
  this->declare_parameter<std::string>("topics.guidance.los");
  this->declare_parameter<std::string>("topics.waypoint");
  this->declare_parameter<std::string>("action_servers.los");

  const std::string pose_topic      = this->get_parameter("topics.pose").as_string();
  const std::string guidance_topic  = this->get_parameter("topics.guidance.los").as_string();
  const std::string waypoint_topic  = this->get_parameter("topics.waypoint").as_string();
  const std::string actions_server  = this->get_parameter("action_servers.los").as_string();

  refrence_out = this->create_publisher<vortex_msgs::msg::LOSGuidance>(guidance_topic, 10);

  waypoint_in = this->create_subscription<geometry_msgs::msg::PointStamped>(
      waypoint_topic, 10,
      std::bind(&GuidanceNode::waypoint_callback, this, std::placeholders::_1));

  pose_in = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic, 10,
      std::bind(&GuidanceNode::pose_callback, this, std::placeholders::_1));

  guidance_action_server = rclcpp_action::create_server<vortex_msgs::action::LOSGuidance>(
      this,
      actions_server,
      std::bind(&GuidanceNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&GuidanceNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&GuidanceNode::handle_accepted, this, std::placeholders::_1));
}

void GuidanceNode::setup_parameters() {
  this->declare_parameter<double>("lookahead_distance_h");
  this->declare_parameter<double>("lookahead_distance_v");
  this->declare_parameter<double>("gamma_h");
  this->declare_parameter<double>("gamma_v");
  this->declare_parameter<double>("time_step");    // seconds
  this->declare_parameter<double>("u_desired");    // m/s
  this->declare_parameter<double>("u_min");
  this->declare_parameter<double>("d_scale");

  const double lookahead_distance_h_ = this->get_parameter("lookahead_distance_h").as_double();
  const double lookahead_distance_v_ = this->get_parameter("lookahead_distance_v").as_double();
  const double gamma_h_              = this->get_parameter("gamma_h").as_double();
  const double gamma_v_              = this->get_parameter("gamma_v").as_double();
  const double time_step_s           = this->get_parameter("time_step").as_double();
  const double u_desired_param       = this->get_parameter("u_desired").as_double();
  const double u_min_                = this->get_parameter("u_min").as_double();
  const double d_scale_              = this->get_parameter("d_scale").as_double();

  // Construct guidance object
  adaptive_los_guidance_ = std::make_unique<LOSGuidance>(
      lookahead_distance_h_,
      lookahead_distance_v_,
      gamma_h_,
      gamma_v_,
      time_step_s,
      u_desired_param,
      u_min_,
      d_scale_);

  // Convert seconds -> milliseconds for loop timing
  time_step = std::chrono::milliseconds(static_cast<int>(std::round(time_step_s * 1000.0)));
}


void GuidanceNode::fill_los_waypoints(const geometry_msgs::msg::PointStamped & msg) {
  last_waypoint_.x = current_position_.x;
  last_waypoint_.y = current_position_.y;
  last_waypoint_.z = current_position_.z;

  target_waypoint_.x = msg.point.x;
  target_waypoint_.y = msg.point.y;
  target_waypoint_.z = msg.point.z;
}

void GuidanceNode::waypoint_callback(const geometry_msgs::msg::PointStamped::SharedPtr g_waypoints) {
  fill_los_waypoints(*g_waypoints);
  // NOTE: update_angles(prev, next)
  adaptive_los_guidance_->update_angles(last_waypoint_, target_waypoint_);
  spdlog::info("Waypoint received");
}

void GuidanceNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  current_position_.x = msg->pose.position.x;
  current_position_.y = msg->pose.position.y;
  current_position_.z = msg->pose.position.z;
}

rclcpp_action::GoalResponse GuidanceNode::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const vortex_msgs::action::LOSGuidance::Goal> goal) {
  (void)goal;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (goal_handle_ && goal_handle_->is_active()) {
      spdlog::info("Aborting current goal and accepting new goal");
      preempted_goal_id_ = goal_handle_->get_goal_id();
    }
  }
  spdlog::info("Accepted goal request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GuidanceNode::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
  spdlog::info("Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GuidanceNode::handle_accepted(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::LOSGuidance>>
        goal_handle) {
  execute_guidance(goal_handle);
}

vortex_msgs::msg::LOSGuidance GuidanceNode::fill_los_reference() {
  vortex_msgs::msg::LOSGuidance reference_msg;
  reference_msg.pitch = pitch_d_;
  reference_msg.yaw   = yaw_d_;
  reference_msg.surge = u_d_;
  return reference_msg;
}

void GuidanceNode::execute_guidance(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<vortex_msgs::action::LOSGuidance>>
        goal_handle) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle_ = goal_handle;
  }

  const geometry_msgs::msg::PointStamped goal_to_go = goal_handle->get_goal()->goal;
  fill_los_waypoints(goal_to_go);
  // correct member + argument order: (prev, next)
  adaptive_los_guidance_->update_angles(last_waypoint_, target_waypoint_);

  const double loop_hz =
      (time_step.count() > 0) ? (1000.0 / static_cast<double>(time_step.count())) : 10.0;
  rclcpp::Rate loop_rate(loop_hz);

  auto feedback = std::make_shared<vortex_msgs::action::LOSGuidance::Feedback>();
  auto result   = std::make_shared<vortex_msgs::action::LOSGuidance::Result>();

  while (rclcpp::ok()) {
    // Preemption / cancel checks
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (goal_handle_ != goal_handle) {
        spdlog::info("New goal accepted, preempting current goal");
        result->success = false;
        goal_handle->canceled(result);
        return;
      }
      if (goal_handle->is_canceling()) {
        spdlog::info("Goal canceled");
        result->success = false;
        goal_handle->canceled(result);
        return;
      }
    }

    // Guidance update
    adaptive_los_guidance_->cross_track_error(last_waypoint_, current_position_);
    yaw_d_   = adaptive_los_guidance_->desired_heading();
    pitch_d_ = adaptive_los_guidance_->desired_pitch();
    u_d_     = adaptive_los_guidance_->desired_surge_speed(target_waypoint_, current_position_);
    adaptive_los_guidance_->update_adaptive_estimates();

    // Publish feedback + reference
    const vortex_msgs::msg::LOSGuidance reference_msg = fill_los_reference();
    feedback->feedback = reference_msg;
    goal_handle->publish_feedback(feedback);
    refrence_out->publish(reference_msg);

    // Goal check using explicit distance calc
    const double dx = current_position_.x - target_waypoint_.x;
    const double dy = current_position_.y - target_waypoint_.y;
    const double dz = current_position_.z - target_waypoint_.z;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist <= 0.5) {
      result->success = true;
      goal_handle->succeed(result);
      refrence_out->publish(fill_los_reference());
      spdlog::info("Goal reached");
      return;
    }

    loop_rate.sleep();
  }
}

