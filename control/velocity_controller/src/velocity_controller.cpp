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
  this->declare_parameter<std::string>("topic_info_out");
  this->declare_parameter<std::string>("topic_ref_in");
  this->declare_parameter<std::string>("topic_velocity_and_orientation_in");
  info_out_topic = this->get_parameter("topic_info_out").as_string();
  reference_topic=this->get_parameter("topic_ref_in").as_string();
  velocity_and_orientation_topic = this->get_parameter("topic_velocity_and_orientation_in").as_string();

  // Lager en publisher som publisher på topic, velocity topic, 10 i "sikkerhet"
  publisher_ = create_publisher<geometry_msgs::msg::WrenchStamped>(info_out_topic, 10);
  sub_velocity_and_orientation_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    velocity_and_orientation_topic, 10,
    std::bind(&Velocity_node::recieve_new_velocity_and_orientation,this, std::placeholders::_1));
  //Lager en timer som kaller funksjonen timer_callback hvert 500ms
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&Velocity_node::send_velocity, this));

  subscriber_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    reference_topic,10,
    std::bind(&Velocity_node::recieve_new_reference,this, std::placeholders::_1));
}



//Utdata på topic
void Velocity_node::send_velocity()
{
  auto message = geometry_msgs::msg::WrenchStamped();
  message.wrench.force.x = 1.0;
  message.wrench.force.y = 0.0;
  message.wrench.force.z = 0.0;
  message.wrench.torque.x = 0.0;
  message.wrench.torque.y = 0.0;
  message.wrench.torque.z = 0.0;
  publisher_->publish(message);
}

//Ny referanse funksjon:
void Velocity_node::recieve_new_reference(const geometry_msgs::msg::WrenchStamped::SharedPtr msg_ptr){
  RCLCPP_INFO(this->get_logger(), "Received reference: '%f'", msg_ptr->wrench.force.x);
  reference = *msg_ptr;
  return;
}
//Ny velocity og orientation funksjon:
void Velocity_node::recieve_new_velocity_and_orientation(const geometry_msgs::msg::WrenchStamped::SharedPtr msg_ptr){
  RCLCPP_INFO(this->get_logger(), "Received velocity and orientation: '%f'", msg_ptr->wrench.force.x);
  current_velocity_and_orientation = *msg_ptr;
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