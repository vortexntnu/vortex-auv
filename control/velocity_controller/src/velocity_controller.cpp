#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "velocity_controller/velocity_controller.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include "std_msgs/msg/bool.hpp"
#include "velocity_controller/PID_setup.hpp"
#include <cmath>
//#include "vortex-msgs/msg" kan legge til nye meldinger nå

//Lager en klasse velocity node

//Konstruktør
Velocity_node::Velocity_node() : Node("velocity_controller_node"), PID_surge(1,1,1), PID_yaw(1,1,1), PID_pitch(1,1,1)
{
  //Dytter info til log
  RCLCPP_INFO(this->get_logger(), "Velocity control node has been started.");

  //Parameter from config.
  this->declare_parameter<std::string>("topics.thrust_topic");
  this->declare_parameter<std::string>("topics.guidance_topic");
  this->declare_parameter<std::string>("topics.twist_topic");
  this->declare_parameter<std::string>("topics.pose_topic");
  this->declare_parameter<std::string>("topics.killswitch_topic");
  this->declare_parameter<double>("max_force");
  this->max_force = this->get_parameter("max_force").as_double();
  this->topic_thrust = this->get_parameter("topics.thrust_topic").as_string();
  this->topic_guidance = this->get_parameter("topics.guidance_topic").as_string();
  this->topic_twist = this->get_parameter("topics.twist_topic").as_string();
  this->topic_killswitch = this->get_parameter("topics.killswitch_topic").as_string();
  this->topic_pose = this->get_parameter("topics.pose_topic").as_string();

  // Publishers
  publisher_thrust = create_publisher<geometry_msgs::msg::WrenchStamped>(topic_thrust, 10);
  
  //Subscribers
  subscriber_twist = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    topic_twist, 10, 
    std::bind(&Velocity_node::twist_callback,this, std::placeholders::_1));

  subscriber_pose = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    topic_pose, 10,
    std::bind(&Velocity_node::pose_callback,this, std::placeholders::_1));

  subscriber_guidance = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    topic_guidance,10,
    std::bind(&Velocity_node::guidance_callback,this, std::placeholders::_1));
  subscriber_killswitch = this->create_subscription<std_msgs::msg::Bool>(
    topic_killswitch,10,
    std::bind(&Velocity_node::killswitch_callback,this, std::placeholders::_1));

  //Timer
  this->declare_parameter<int>("calculation_rate");
  this->declare_parameter<int>("publish_rate");
  this->calculation_rate = this->get_parameter("calculation_rate").as_int();
  this->publish_rate = this->get_parameter("publish_rate").as_int();
  timer_calculation = this->create_wall_timer(std::chrono::milliseconds(calculation_rate), std::bind(&Velocity_node::publish_thrust, this));
  timer_publish = this->create_wall_timer(std::chrono::milliseconds(publish_rate), std::bind(&Velocity_node::calc_thrust, this));

  PID_surge.set_output_limits(-max_force, max_force);
  PID_pitch.set_output_limits(-max_force, max_force);
  PID_yaw.set_output_limits(-max_force, max_force);
}



//Publish/timer functions
void Velocity_node::publish_thrust()
{
  publisher_thrust->publish(thrust_out);
}

//** må forbedre integrasjon og derivasjons beregningene
void Velocity_node::calc_thrust()
{
  PID_surge.calculate_thrust(reference.surge, current_state.surge,calculation_rate/1000.0);
  PID_pitch.calculate_thrust(reference.pitch, current_state.pitch,calculation_rate/1000.0);
  PID_yaw.calculate_thrust(reference.yaw, current_state.yaw,calculation_rate/1000.0);
  thrust_out.wrench.force.x = PID_surge.output();
  thrust_out.wrench.torque.y = PID_pitch.output();
  thrust_out.wrench.torque.z = PID_yaw.output();

  return;
}



//Callback functions
void Velocity_node::guidance_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg_ptr){
  //RCLCPP_INFO(this->get_logger(), "Received reference: '%f'", msg_ptr->wrench.force.x);
  reference = *msg_ptr;
  return;
}
void Velocity_node::twist_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg_ptr){
  //RCLCPP_INFO(this->get_logger(), "Received velocity and orientation: '%f'", msg_ptr->wrench.force.x);
  current_state.surge = msg_ptr->twist.twist.linear.x;
  return;
}
void Velocity_node::pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg_ptr){
  angle temp=quaternion_to_euler_angle(msg_ptr->pose.pose.orientation.w, msg_ptr->pose.pose.orientation.x, msg_ptr->pose.pose.orientation.y, msg_ptr->pose.pose.orientation.z);
  current_state.pitch = temp.thetat;
  current_state.yaw = temp.psit;
  return;
}

//**Needs to update to shutdown the node
void Velocity_node::killswitch_callback(const std_msgs::msg::Bool::SharedPtr msg_ptr){
  RCLCPP_INFO(this->get_logger(), "Received killswitch: '%d'", msg_ptr->data);
  if(msg_ptr->data == true){
    reference = guidance_data();
    current_state = guidance_data();
    RCLCPP_INFO(this->get_logger(), "Killswitch activated, reference and current state set to zero");
  }
  return;
}
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Velocity_node>());
  rclcpp::shutdown();
  return 0;
}

//----------------------------------------------------------------------------------------------------------------
//Operator overloading for geometry_msgs::msg::WrenchStamped
geometry_msgs::msg::WrenchStamped operator-(const geometry_msgs::msg::WrenchStamped & a, const geometry_msgs::msg::WrenchStamped & b)
{
  geometry_msgs::msg::WrenchStamped result;
  result.wrench.force.x = a.wrench.force.x - b.wrench.force.x;
  result.wrench.force.y = a.wrench.force.y - b.wrench.force.y;
  result.wrench.force.z = a.wrench.force.z - b.wrench.force.z;
  result.wrench.torque.x = a.wrench.torque.x - b.wrench.torque.x;
  result.wrench.torque.y = a.wrench.torque.y - b.wrench.torque.y;
  result.wrench.torque.z = a.wrench.torque.z - b.wrench.torque.z;
  return result;
}
geometry_msgs::msg::WrenchStamped operator+(const geometry_msgs::msg::WrenchStamped & a, const geometry_msgs::msg::WrenchStamped & b)
{
  geometry_msgs::msg::WrenchStamped result;
  result.wrench.force.x = a.wrench.force.x + b.wrench.force.x;
  result.wrench.force.y = a.wrench.force.y + b.wrench.force.y;
  result.wrench.force.z = a.wrench.force.z + b.wrench.force.z;
  result.wrench.torque.x = a.wrench.torque.x + b.wrench.torque.x;
  result.wrench.torque.y = a.wrench.torque.y + b.wrench.torque.y;
  result.wrench.torque.z = a.wrench.torque.z + b.wrench.torque.z;
  return result;
}
//operator overloading for guidance_data
guidance_data guidance_data::operator-(const guidance_data & b) const
{
  guidance_data result;
  result.surge = this->surge - b.surge;
  result.pitch = this->pitch - b.pitch;
  result.yaw = this->yaw - b.yaw; 
  return result;
}

guidance_data& guidance_data::operator=(const std_msgs::msg::Float64MultiArray& msg)
{
  if (msg.data.size()>=3){
    surge=msg.data[0];
    pitch=msg.data[1];
    yaw=msg.data[2];
  }
  else{
      //throw std::runtime_error("Guidance message too short, needs at least 3 values");
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Guidance message too short, needs at least 3 values");
  }
  return *this;
}

guidance_data::guidance_data(std_msgs::msg::Float64MultiArray msg){
        if (msg.data.size()>=3){
            surge=msg.data[0];
            pitch=msg.data[1];
            yaw=msg.data[2];
        }
        else{
            //throw std::runtime_error("Guidance message too short, needs at least 3 values");
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Guidance message too short, needs at least 3 values");
        }
    };