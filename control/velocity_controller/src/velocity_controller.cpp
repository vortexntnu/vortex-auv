#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "velocity_controller/velocity_controller.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
//#include "vortex-msgs/msg" kan legge til nye meldinger nå

//Lager en klasse velocity node

//Konstruktør
Velocity_node::Velocity_node() : Node("velocity_controller_node")
{
  //Dytter info til log
  RCLCPP_INFO(this->get_logger(), "Velocity control node has been started.");

  //Parameter from config.
  this->declare_parameter<std::string>("topics.thrust_topic");
  this->declare_parameter<std::string>("topics.guidance_topic");
  this->declare_parameter<std::string>("topics.twist_topic");
  this->declare_parameter<std::string>("topics.killswitch_topic");
  this->topic_thrust = this->get_parameter("topics.thrust_topic").as_string();
  this->topic_guidance = this->get_parameter("topics.guidance_topic").as_string();
  this->topic_twist = this->get_parameter("topics.twist_topic").as_string();
  this->topic_killswitch = this->get_parameter("topics.killswitch_topic").as_string();

  // Publishers
  publisher_thrust = create_publisher<geometry_msgs::msg::WrenchStamped>(topic_thrust, 10);
  
  //Subscribers
  subscriber_twist = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    topic_twist, 10, 
    std::bind(&Velocity_node::twist_callback,this, std::placeholders::_1));
  subscriber_guidance = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    topic_guidance,10,
    std::bind(&Velocity_node::guidance_callback,this, std::placeholders::_1));
  subscriber_killswitch = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    topic_killswitch,10,
    std::bind(&Velocity_node::killswitch_callback,this, std::placeholders::_1));

  //Timer
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&Velocity_node::publish_thrust, this));

  
}



//Publish functions
void Velocity_node::publish_thrust()
{
  auto message = geometry_msgs::msg::WrenchStamped();
  message.wrench.force.x = 1.0;
  message.wrench.force.y = 0.0;
  message.wrench.force.z = 0.0;
  message.wrench.torque.x = 0.0;
  message.wrench.torque.y = 0.0;
  message.wrench.torque.z = 0.0;
  publisher_thrust->publish(message);
}

//Callback functions
void Velocity_node::guidance_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg_ptr){
  RCLCPP_INFO(this->get_logger(), "Received reference: '%f'", msg_ptr->wrench.force.x);
  reference = *msg_ptr;
  return;
}
void Velocity_node::twist_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg_ptr){
  RCLCPP_INFO(this->get_logger(), "Received velocity and orientation: '%f'", msg_ptr->wrench.force.x);
  current_velocity_and_orientation = *msg_ptr;
  return;
}

//**Needs to update to shutdown the node
void Velocity_node::killswitch_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg_ptr){
  RCLCPP_INFO(this->get_logger(), "Received killswitch: '%f'", msg_ptr->wrench.force.x);
  if(msg_ptr->wrench.force.x == 1.0){
    reference = geometry_msgs::msg::WrenchStamped();
    current_velocity_and_orientation = geometry_msgs::msg::WrenchStamped();
    RCLCPP_INFO(this->get_logger(), "Killswitch activated, reference and current velocity set to zero");
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