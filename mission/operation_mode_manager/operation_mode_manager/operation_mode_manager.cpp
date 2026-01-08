#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

#include "vortex_msgs/srv/operation_mode_srv.hpp"
#include "vortex_msgs/msg/operation_mode.hpp"

class OperationModeManager : public rclcpp::Node
{
  public:
    explicit OperationModeManager(const rclcpp::NodeOptions & options) : Node("operation_mode_manager", options),
      killswitch_(true),
      mode_(vortex_msgs::srv::OperationModeSRV::Request::MANUAL)
    {
      declare_parameters();
      setup_publishers();
      setup_service();

      RCLCPP_INFO(this->get_logger(), "Operation Mode Manager Node started. Initial mode: Manual, Killswitch: true");
    }

  private:
    // Parameters
    std::string wrench_input_topic_;
    std::string killswitch_topic_;
    std::string operation_mode_topic_;

    // State
    bool killswitch_;
    int mode_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr killswitch_pub_;
    rclcpp::Publisher<vortex_msgs::msg::OperationMode>::SharedPtr mode_pub_;

    // Service
    rclcpp::Service<vortex_msgs::srv::OperationModeSRV>::SharedPtr service_;

    void declare_parameters()
    {
      this->declare_parameter<std::string>("topics.wrench_input", "");
      this->declare_parameter<std::string>("topics.killswitch", "");
      this->declare_parameter<std::string>("topics.operation_mode", "");

      wrench_input_topic_ =
        this->get_parameter("topics.wrench_input").as_string();
      killswitch_topic_ =
        this->get_parameter("topics.killswitch").as_string();
      operation_mode_topic_ =
        this->get_parameter("topics.operation_mode").as_string();
    }

    void setup_publishers()
    {
      wrench_pub_ =
        this->create_publisher<geometry_msgs::msg::WrenchStamped>(
          wrench_input_topic_, rclcpp::SensorDataQoS());

      killswitch_pub_ =
        this->create_publisher<std_msgs::msg::Bool>(
          killswitch_topic_, rclcpp::QoS(2).reliable());

      mode_pub_ =
        this->create_publisher<vortex_msgs::msg::OperationMode>(
          operation_mode_topic_, rclcpp::QoS(2).reliable());

      // Initial killswitch = true
      std_msgs::msg::Bool msg;
      msg.data = true;
      killswitch_pub_->publish(msg);

      // Initial mode = MANUAL
      vortex_msgs::msg::OperationMode mode_msg;
      mode_msg.mode = vortex_msgs::srv::OperationModeSRV::Request::MANUAL;
      mode_pub_->publish(mode_msg);
    }
    void setup_service()
    {
    service_ = this->create_service<vortex_msgs::srv::OperationModeSRV>(
      "set_operation_mode",
      [this](
        const std::shared_ptr<vortex_msgs::srv::OperationModeSRV::Request> request,
        std::shared_ptr<vortex_msgs::srv::OperationModeSRV::Response> response)
      {
        set_operation_mode_callback(request, response);
      });  
    }

    void set_operation_mode_callback(
      const std::shared_ptr<vortex_msgs::srv::OperationModeSRV::Request> request,
      std::shared_ptr<vortex_msgs::srv::OperationModeSRV::Response> response)
    {

    switch (request->mode) {
        // Killswitch requested
        case 0:
            if (request->absolute) {
                RCLCPP_INFO(this->get_logger(), "Killswitch set to true");
                killswitch_ = true;

                geometry_msgs::msg::WrenchStamped empty_wrench_msg;
                empty_wrench_msg.header.stamp = this->get_clock()->now();
                empty_wrench_msg.header.frame_id = "base_link";
                wrench_pub_->publish(empty_wrench_msg);

            } else {
                if (killswitch_) {
                    RCLCPP_INFO(this->get_logger(), "Killswitch set to false");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Killswitch set to true");
                    geometry_msgs::msg::WrenchStamped empty_wrench_msg;
                    empty_wrench_msg.header.stamp = this->get_clock()->now();
                    empty_wrench_msg.header.frame_id = "base_link";
                    wrench_pub_->publish(empty_wrench_msg);
                }
                killswitch_ = !killswitch_;
            }
            break;
        
        // Autonomous mode requested
        case 1:
            if (request->absolute) {
                RCLCPP_INFO(this->get_logger(), "Killswitch set to false");
                killswitch_ = false;
            }
            RCLCPP_INFO(this->get_logger(), "Mode set to AUTONOMOUS");
            mode_ = vortex_msgs::srv::OperationModeSRV::Request::AUTONOMOUS;
            break;
        
        // Manual mode requested
        case 2:
            if (request->absolute) {
                killswitch_ = false;
            }
            RCLCPP_INFO(this->get_logger(), "Mode set to MANUAL");
            mode_ = vortex_msgs::srv::OperationModeSRV::Request::MANUAL;
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "Invalid mode requested");
            break;
        }
    
    // Publish killswitch status
    std_msgs::msg::Bool killswitch_msg;
    killswitch_msg.data = killswitch_;
    killswitch_pub_->publish(killswitch_msg); 

    // Publish operation mode
    vortex_msgs::msg::OperationMode mode_msg;
    mode_msg.mode = mode_;
    mode_pub_->publish(mode_msg); 

    response->mode = mode_;
    response->killswitch_status = killswitch_;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OperationModeManager>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}