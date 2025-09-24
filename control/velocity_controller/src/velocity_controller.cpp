#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
//#include "vortex-msgs/msg" kan legge til nye meldinger nå

//Lager en klasse velocity node
class Velocity_node : public rclcpp::Node
{
public:
//Konstruktør
    Velocity_node() : Node("velocity_controller_node")
    {
        //Dytter info til log
        RCLCPP_INFO(this->get_logger(), "Velocity control node has been started.");

        //Parameter from config. !!Needs to create launch file to prevent writing where to get the parameters from
        this->declare_parameter<std::string>("topic_info_out");
        this->declare_parameter<std::string>("topic_ref_in");
        info_out_topic = this->get_parameter("topic_info_out").as_string();
        reference_topic=this->get_parameter("topic_ref_in").as_string();

        // Lager en publisher som publisher på topic, velocity topic, 10 i "sikkerhet"
        publisher_ = create_publisher<std_msgs::msg::String>(info_out_topic, 10);
        //Lager en timer som kaller funksjonen timer_callback hvert 500ms
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&Velocity_node::send_velocity, this));

        subscriber_ = this->create_subscription<std_msgs::msg::String>(
          reference_topic,10,
          std::bind(&Velocity_node::recieve_new_reference,this, std::placeholders::_1));
    }


//Alle funksjonens variabler
//Publisher og timer instansene
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

    std::string info_out_topic;
    std::string reference_topic;


//Utdata på topic
    void send_velocity()
    {
      auto message = std_msgs::msg::String();
      message.data = "Velocity command";
      //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    
//Ny referanse funksjon:
    void recieve_new_reference(const std_msgs::msg::String::SharedPtr msg_ptr){
    RCLCPP_INFO(this->get_logger(), "Received reference: '%s'", msg_ptr->data.c_str());

    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Velocity_node>());
  rclcpp::shutdown();
  return 0;
}

