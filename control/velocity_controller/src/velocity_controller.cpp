#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "velocity_controller/velocity_controller.hpp"
#include <rmw/types.h>
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "velocity_controller/NMPC_setup.hpp"
#include "velocity_controller/PID_setup.hpp"
#include <array>
#include <chrono>
#include <cmath>
#include <Eigen/Dense>
#include <lifecycle_msgs/msg/detail/transition__struct.hpp>
#include <numbers>
#include <rclcpp/utilities.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <vector>
#include "vortex_msgs/msg/los_guidance.hpp"
#include "vortex/utils/math.hpp"
#include "velocity_controller/utilities.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>


//Konstruktør
Velocity_node::Velocity_node() : rclcpp_lifecycle::LifecycleNode("velocity_controller_lifecycle"), lqr_controller(), pub_QoS(10),  sub_QoS(10)
{
  get_new_parameters();
  pub_QoS.keep_last(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT).durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
  sub_QoS.keep_last(10).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT).durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  //TODO: dont need to save the Q3 R3 and the other matries, just use #define
  //NMPC controller
  NMPC.set_matrices(Q3,R3, inertia_matrix, max_force,  dampening_matrix_low, dampening_matrix_high);
  NMPC.set_interval(publish_rate/1000.0);
  //NMPC.initialize_MPC(); 
  //NMPC acados controller
  NMPC_acados.init();
  NMPC_acados.set_max_force(max_force);
  std::vector<double> W=Q2;
  W.insert(W.end(),R2.begin(),R2.end());
  std::vector<double> We=Q2;
  NMPC_acados.set_weights(W, We);
  
  
  //Controllers
  PID_surge.set_output_limits(-max_force, max_force);
  PID_pitch.set_output_limits(-max_force, max_force);
  PID_yaw.set_output_limits(-max_force, max_force);
  PID_surge.set_parameters(surge_params,publish_rate/1000.0);
  PID_pitch.set_parameters(pitch_params,publish_rate/1000.0);
  PID_yaw.set_parameters(yaw_params,publish_rate/1000.0);

  if(!lqr_controller.set_matrices(Q,R,inertia_matrix,max_force,dampening_matrix_low,dampening_matrix_high)||!lqr_controller.set_interval(static_cast<double>(publish_rate)/1000)){
    controller_type=1;
    RCLCPP_INFO(this->get_logger(),"Switching to PID");
  };
  if(auto_start){
    startup_timer_=create_wall_timer(std::chrono::milliseconds(0), [this](){startup_timer_->cancel(); trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);});
  }
  RCLCPP_INFO(this->get_logger(), "Velocity control node has been started.");

  return;
}


void Velocity_node::calc_thrust()
{
  if (odometry_dropout_guard){
    publish_counter++;
    if(publish_counter>=100){
      reset_controllers();
      RCLCPP_WARN(this->get_logger(),"Odometry dropout, no thrust");
      return;
    }
  }
  //TODO: Do I need ssa here?
  angle ref_in_body=angle_NED_to_body({0,vortex::utils::math::ssa(guidance_values.pitch),vortex::utils::math::ssa(guidance_values.yaw)},current_state.get_angle());
  Guidance_data error={guidance_values.surge-current_state.surge,-ref_in_body.thetat,-ref_in_body.psit};

  if(anti_overshoot){
    if (abs(error.yaw)<std::numbers::pi/2 || abs(error.yaw)<std::numbers::pi/2){ 
    error.surge=guidance_values.surge*cos(error.yaw)*cos(error.pitch);
    }
  }
  switch (controller_type)
  {
  case 1:{
    //TODO: some logic for removing the change in reference
    PID_surge.calculate_thrust(error.surge);
    PID_pitch.calculate_thrust(error.pitch,-current_state.pitch_rate);
    PID_yaw.calculate_thrust(error.yaw, -current_state.yaw_rate);
    thrust_out.wrench.force.x = PID_surge.get_output();
    thrust_out.wrench.torque.y = PID_pitch.get_output();
    thrust_out.wrench.torque.z = PID_yaw.get_output();
    
    break;
  }
  case 2:{
    
    
    if (!lqr_controller.calculate_thrust(current_state,error)){
      controller_type=1;
      RCLCPP_ERROR(this->get_logger(),"Switching to PID");
    }
    else{
      Eigen::Vector3d u=lqr_controller.get_thrust();
      thrust_out.wrench.force.x=u[0];
      thrust_out.wrench.torque.y=u[1];
      thrust_out.wrench.torque.z=u[2];
    }
    break;
  }
  case 3:{
    //RCLCPP_INFO(this->get_logger(),"Guidance: %f, %f, %f",guidance_values.surge,guidance_values.pitch,guidance_values.yaw);
    Eigen::Matrix<double,3,1> u;
    if (NMPC.calculate_thrust(guidance_values, current_state)){
      controller_type=1;
      RCLCPP_ERROR(this->get_logger(),"Switching to PID");
      rclcpp::shutdown();
    }
    else{
    u=NMPC.get_thrust();
    thrust_out.wrench.force.x=u[0];
    thrust_out.wrench.torque.y=u[1];
    thrust_out.wrench.torque.z=u[2];
    RCLCPP_INFO(this->get_logger(),"NMPC: surge: %f, pitch %f, yaw %f",u(0),u(1),u(2));
    }

    break;
  }
  case 4:{
    std::vector<double>u;
    std::array<double, 5> x_ref={guidance_values.surge,0.0,0.0,guidance_values.pitch,guidance_values.yaw}; //surge, pitch_rate, yaw_rate, pitch, yaw

    NMPC_acados.setReference(x_ref);
    std::array<double,9> state_array={current_state.surge,current_state.sway,current_state.heave,current_state.roll_rate,current_state.pitch_rate,current_state.yaw_rate,current_state.roll,current_state.pitch,current_state.yaw};
    NMPC_acados.setState(state_array);
    int status=NMPC_acados.solve_once();
    RCLCPP_INFO(this->get_logger(),"Status %i",status);
    if(status){
      
      rclcpp::shutdown();
    };
    u=NMPC_acados.getU0();
    thrust_out.wrench.force.x=u[0];
    thrust_out.wrench.torque.y=u[1];
    thrust_out.wrench.torque.z=u[2];
    break;
  }
  default:{
    //Some crash handling here
    RCLCPP_ERROR(this->get_logger(),"Unknown controller set");
    break;
  }
  }
  publisher_thrust->publish(thrust_out);
  return;
}



//Callback functions
void Velocity_node::guidance_callback(const vortex_msgs::msg::LOSGuidance::SharedPtr msg_ptr){
  if(reset_on_new_ref){ //On big step changes, reset the controllers to avoid big overshoots
    if(abs(msg_ptr->surge-guidance_values.surge)>=0.1)  reset_controllers(1);
    if (abs(msg_ptr->pitch-guidance_values.pitch)>std::numbers::pi/4)reset_controllers(2); 
    if (abs(msg_ptr->yaw-guidance_values.yaw)<std::numbers::pi/4)reset_controllers(3);
  }
  guidance_values = msg_ptr; //overloaded to fix all the internal states
  
  return;
}

void Velocity_node::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg_ptr){
  publish_counter=0;
  current_state=msg_ptr; //overloaded to fix all the internal states
  return;
}


void Velocity_node::get_new_parameters(){
  //topics //TODO: check what happens when same parameter in global and local file
  this->declare_parameter<std::string>("topics.wrench_input");
  this->topic_thrust = this->get_parameter("topics.wrench_input").as_string();
  this->declare_parameter<std::string>("topics.guidance.los");
  this->topic_guidance = this->get_parameter("topics.guidance.los").as_string();
  this->declare_parameter<std::string>("topics.odom");
  this->topic_odometry = this->get_parameter("topics.odom").as_string();
  
  //variables

  this->declare_parameter<double>("max_force");
  this->max_force = this->get_parameter("max_force").as_double();  
  this->declare_parameter<int>("publish_rate");
  this->publish_rate = this->get_parameter("publish_rate").as_int();
  this->declare_parameter<int>("controller_type");
  this->controller_type=this->get_parameter("controller_type").as_int();
  
  //PID Params
  this->declare_parameter<std::vector<double>>("PID_params.surge");
  surge_params=this->get_parameter("PID_params.surge").as_double_array();
  this->declare_parameter<std::vector<double>>("PID_params.pitch");
  pitch_params=this->get_parameter("PID_params.pitch").as_double_array();
  this->declare_parameter<std::vector<double>>("PID_params.yaw");
  yaw_params=this->get_parameter("PID_params.yaw").as_double_array();

  //LQR Parameters

  this->declare_parameter<std::vector<double>>("LQR_params.Q");
  Q=this->get_parameter("LQR_params.Q").as_double_array();
  this->declare_parameter<std::vector<double>>("LQR_params.R");
  R=this->get_parameter("LQR_params.R").as_double_array();
  this->declare_parameter<std::vector<double>>("physical.mass_matrix");
  inertia_matrix=this->get_parameter("physical.mass_matrix").as_double_array();

  //D
  this->declare_parameter<std::vector<double>>("dampening_matrix_low");
  this->declare_parameter<std::vector<double>>("dampening_matrix_high");
  this->dampening_matrix_low=this->get_parameter("dampening_matrix_low").as_double_array();
  this->dampening_matrix_high=this->get_parameter("dampening_matrix_high").as_double_array();

  //NMPC acados Parameters
  this->declare_parameter<std::vector<double>>("NMPCA_params.Q");
  this->declare_parameter<std::vector<double>>("NMPCA_params.R");
  Q2=this->get_parameter("NMPCA_params.Q").as_double_array();
  R2=this->get_parameter("NMPCA_params.R").as_double_array();
  //NMPC

  this->declare_parameter<std::vector<double>>("NMPC_params.Q");
  this->declare_parameter<std::vector<double>>("NMPC_params.R");
  Q3=this->get_parameter("NMPC_params.Q").as_double_array();
  R3=this->get_parameter("NMPC_params.R").as_double_array();

  
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn Velocity_node::on_configure(const rclcpp_lifecycle::State &){
  RCLCPP_INFO(get_logger(), "Configure VC");
  
  // Publishers
  publisher_thrust = create_publisher<geometry_msgs::msg::WrenchStamped>(topic_thrust, pub_QoS);
  
  //Subscribers
  subscriber_Odometry = this->create_subscription<nav_msgs::msg::Odometry>( topic_odometry,sub_QoS, std::bind(&Velocity_node::odometry_callback,this,std::placeholders::_1));
  subscriber_guidance = this->create_subscription<vortex_msgs::msg::LOSGuidance>( topic_guidance,sub_QoS,std::bind(&Velocity_node::guidance_callback,this, std::placeholders::_1));
  //Timer
  if(first_start&&auto_start){
    startup_timer_=create_wall_timer(std::chrono::milliseconds(0),[this](){startup_timer_->cancel(); trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);});
  }
  first_start=false;
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Velocity_node::on_activate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Activating...");
  timer_calculation = this->create_wall_timer(std::chrono::milliseconds(publish_rate), std::bind(&Velocity_node::calc_thrust, this));
  auto ret = LifecycleNode::on_activate(state);
  
  return ret;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Velocity_node::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Deactivating...");
  auto ret = LifecycleNode::on_deactivate(state);
  timer_calculation.reset();
  reset_controllers();
  return ret;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Velocity_node::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning up...");
  timer_calculation.reset();
  publisher_thrust.reset();
  subscriber_guidance.reset();
  subscriber_Odometry.reset();
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
Velocity_node::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting down from state %s", state.label().c_str());
  if(timer_calculation) timer_calculation->cancel();
  timer_calculation.reset();
  publisher_thrust.reset();
  subscriber_guidance.reset();
  subscriber_Odometry.reset();
  should_exit_=true;
  return CallbackReturn::SUCCESS;
}

void Velocity_node::reset_controllers(int nr){
    switch (nr) {
    case 0:
      PID_pitch.reset_controller();
      PID_surge.reset_controller();
      PID_yaw.reset_controller();
      lqr_controller.reset_controller();
      break;
    case 1:
      PID_surge.reset_controller();
      lqr_controller.reset_controller(1);

      break;

    case 2:
      PID_pitch.reset_controller();
      lqr_controller.reset_controller(2);

      break;

    case 3:
      PID_yaw.reset_controller();
      lqr_controller.reset_controller(3);
      break;

    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto lc_node = std::make_shared<Velocity_node>();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(lc_node->get_node_base_interface());

  while (rclcpp::ok()&&!lc_node->should_exit_){
      exec.spin_some();
  }
  //rclcpp::shutdown();
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

