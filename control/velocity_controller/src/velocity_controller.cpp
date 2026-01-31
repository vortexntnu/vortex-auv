#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "velocity_controller/velocity_controller.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
//#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
//#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include "std_msgs/msg/bool.hpp"
#include "velocity_controller/PID_setup.hpp"
#include <cmath>
#include <Eigen/Dense>
#include "vortex_msgs/msg/los_guidance.hpp"
#include "vortex/utils/math.hpp"
#include "velocity_controller/utilities.hpp"


//Konstruktør
Velocity_node::Velocity_node() : Node("velocity_controller_node"), PID_surge(100,5,0), PID_yaw(1,0.1,0), PID_pitch(35,1,0), lqr_controller()
{
  //Dytter info til log
  RCLCPP_INFO(this->get_logger(), "Velocity control node has been started.");

  //Parameter from config.
  get_new_parameters();

  
  // Publishers
  rclcpp::QoS orca_QoS(2);
  orca_QoS.keep_last(2).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  
  publisher_thrust = create_publisher<geometry_msgs::msg::WrenchStamped>(topic_thrust, orca_QoS);
  publisher_reference = create_publisher<std_msgs::msg::Float64MultiArray>("/reference",2);
  
  //Subscribers  
  subscriber_Odometry = this->create_subscription<nav_msgs::msg::Odometry>(
    topic_odometry,10,
    std::bind(&Velocity_node::odometry_callback,this,std::placeholders::_1));
  subscriber_guidance = this->create_subscription<vortex_msgs::msg::LOSGuidance>(
    topic_guidance,10,
    std::bind(&Velocity_node::guidance_callback,this, std::placeholders::_1));
  subscriber_killswitch = this->create_subscription<std_msgs::msg::Bool>(
    topic_killswitch,10,
    std::bind(&Velocity_node::killswitch_callback,this, std::placeholders::_1));


  
  //Timer
  
  timer_calculation = this->create_wall_timer(std::chrono::milliseconds(publish_rate), std::bind(&Velocity_node::calc_thrust, this));
  timer_publish = this->create_wall_timer(std::chrono::milliseconds(publish_rate), std::bind(&Velocity_node::publish_thrust, this));
  //Controllers
  PID_surge.set_output_limits(-max_force, max_force);
  PID_pitch.set_output_limits(-max_force, max_force);
  PID_yaw.set_output_limits(-max_force, max_force);
  if(!lqr_controller.set_matrices(Q,R,inertia_matrix,max_force,dampening_matrix_low,dampening_matrix_high)||!lqr_controller.set_interval(static_cast<double>(publish_rate)/1000)){
    controller_type=1;
    RCLCPP_INFO(this->get_logger(),"Switching to PID");
  };

}





//Publish/timer functions
void Velocity_node::publish_thrust()
{
  publisher_thrust->publish(thrust_out);
}

//** må forbedre integrasjon og derivasjons beregningene
void Velocity_node::calc_thrust()
{
  angle NED_error={guidance_values.roll-current_state.roll,guidance_values.pitch-current_state.pitch,guidance_values.yaw-current_state.yaw};
  angle error=NED_to_BODY(NED_error,current_state);
  Guidance_data mod_g_values=guidance_values;
  if (error.psit<3.14/2 && error.thetat<3.14/2){ //Need to fix to pi
  mod_g_values.surge=guidance_values.surge*cos(error.psit)*cos(error.thetat);
  }
  else{
    mod_g_values.surge=0;
  }
  switch (controller_type)
  {
  case 1:{
    
    PID_surge.calculate_thrust(mod_g_values.surge-current_state.surge,publish_rate/1000.0);
    PID_pitch.calculate_thrust(error.thetat,publish_rate/1000.0);
    PID_yaw.calculate_thrust(error.psit,publish_rate/1000.0);
    thrust_out.wrench.force.x = PID_surge.get_output();
    thrust_out.wrench.torque.y = PID_pitch.get_output();
    thrust_out.wrench.torque.z = PID_yaw.get_output();
    
    break;
  }
  case 2:{
    
    Eigen::Vector3d u=lqr_controller.calculate_thrust(current_state,mod_g_values);
    if (u==Eigen::Vector3d{9999,9999,9999}){
      controller_type=1;
      RCLCPP_ERROR(this->get_logger(),"Switching to PID");
    }
    else{
      thrust_out.wrench.force.x=u[0];
      thrust_out.wrench.torque.y=u[1];
      thrust_out.wrench.torque.z=u[2];
    }
    break;
  }
  default:{
    //Some crash handling here
    RCLCPP_ERROR(this->get_logger(),"Unknown controller set");
    break;
  }
  }
  std_msgs::msg::Float64MultiArray msg;
    msg.data={guidance_values.surge,guidance_values.pitch,guidance_values.yaw};
    publisher_reference->publish(msg);
  return;
}



//Callback functions
void Velocity_node::guidance_callback(const vortex_msgs::msg::LOSGuidance::SharedPtr msg_ptr){
  guidance_values = *msg_ptr;
  //RCLCPP_DEBUG(this->get_logger(), "Guidance received: surge=%.3f pitch=%.3f yaw=%.3f",
  //             guidance_values.surge, guidance_values.pitch, guidance_values.yaw);
  return;
}

void Velocity_node::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg_ptr){
  //RCLCPP_INFO(this->get_logger(),"Recieved odometry");
  angle temp=quaternion_to_euler_angle(msg_ptr->pose.pose.orientation.w, msg_ptr->pose.pose.orientation.x, msg_ptr->pose.pose.orientation.y, msg_ptr->pose.pose.orientation.z);
  //angles
  current_state.roll = temp.phit;
  current_state.pitch = temp.thetat;
  current_state.yaw = temp.psit;
  //angular velocity
  current_state.roll_rate=msg_ptr->twist.twist.angular.x;
  current_state.pitch_rate=msg_ptr->twist.twist.angular.y;
  current_state.yaw_rate=msg_ptr->twist.twist.angular.z;
  //velocity
  current_state.surge = msg_ptr->twist.twist.linear.x;
  current_state.sway = msg_ptr->twist.twist.linear.y;
  current_state.heave = msg_ptr->twist.twist.linear.z;
  return;
}

//**Needs to update to shutdown the node
void Velocity_node::killswitch_callback(const std_msgs::msg::Bool::SharedPtr msg_ptr){
  RCLCPP_INFO(this->get_logger(), "Received killswitch: '%d'", msg_ptr->data);
  if(msg_ptr->data == true){
    guidance_values = Guidance_data();
    current_state = Guidance_data();
    RCLCPP_INFO(this->get_logger(), "Killswitch activated, reference and current state set to zero");
  }
  return;
}


void Velocity_node::get_new_parameters(){
  this->declare_parameter<std::string>("topics.thrust_topic");
  this->topic_thrust = this->get_parameter("topics.thrust_topic").as_string();
  this->declare_parameter<std::string>("topics.guidance_topic");
  this->topic_guidance = this->get_parameter("topics.guidance_topic").as_string();
  this->declare_parameter<std::string>("topics.odom_topic");
  this->topic_odometry = this->get_parameter("topics.odom_topic").as_string();
  this->declare_parameter<std::string>("topics.killswitch_topic");
  this->topic_killswitch = this->get_parameter("topics.killswitch_topic").as_string();
  this->declare_parameter<double>("max_force");
  this->max_force = this->get_parameter("max_force").as_double();  
  this->declare_parameter<int>("publish_rate");
  this->publish_rate = this->get_parameter("publish_rate").as_int();
  this->declare_parameter<int>("controller_type");
  this->controller_type=this->get_parameter("controller_type").as_int();
  

  //LQR Parameters
  
  this->declare_parameter<std::vector<double>>("LQR_params.Q");
  Q=this->get_parameter("LQR_params.Q").as_double_array();
  this->declare_parameter<std::vector<double>>("LQR_params.R");
  R=this->get_parameter("LQR_params.R").as_double_array();
  this->declare_parameter<std::vector<double>>("inertia_matrix");
  this->get_parameter("inertia_matrix", inertia_matrix);

  //D
  this->declare_parameter<std::vector<double>>("dampening_matrix_low");
  this->declare_parameter<std::vector<double>>("dampening_matrix_high");
  this->dampening_matrix_low=this->get_parameter("dampening_matrix_low").as_double_array();
  this->dampening_matrix_high=this->get_parameter("dampening_matrix_high").as_double_array();



  

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
/*
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
Guidance_data Guidance_data::operator-(const Guidance_data & b) const
{
  Guidance_data result;
  result.surge = this->surge - b.surge;
  result.pitch = this->pitch - b.pitch;
  result.yaw = this->yaw - b.yaw; 
  return result;
}

Guidance_data& Guidance_data::operator=(const std_msgs::msg::Float64MultiArray& msg)
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
*/

