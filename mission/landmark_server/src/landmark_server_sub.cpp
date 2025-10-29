#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <vortex_msgs/msg/landmark_array.hpp>
using std::placeholders::_1;

using vortex_msgs::msg::LandmarkArray;
using vortex_msgs::msg::Landmark;

class LandmarkArraySubscriber : public rclcpp::Node
{
  public:
    LandmarkArraySubscriber()
    : rclcpp::Node("landmark_array_subscriber")
    {
      storedLandmarks_=std::make_shared<LandmarkArray>();


      sub_ = this->create_subscription<LandmarkArray>(
      "/landmarks", rclcpp::QoS(10), std::bind(&LandmarkArraySubscriber::callback, this, _1));
    }

  private:
   void callback(const LandmarkArray::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "frame='%s'  count=%zu",
                msg->header.frame_id.c_str(), msg->landmarks.size());

    if (!msg->landmarks.empty()) {
      // NOTE: adjust this depending on your actual Landmark.msg:
      // If Landmark has .odom.pose.pose.position:
      const auto &p = msg->landmarks.front().odom.pose.pose.position;
      // If Landmark has .pose.pose.position instead, use that line instead.

      RCLCPP_INFO(get_logger(), "first landmark pos = (%.2f, %.2f, %.2f)",
                  p.x, p.y, p.z);
    }
  }

  rclcpp::Subscription<LandmarkArray>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandMarkArraySubscriber>());
  rclcpp::shutdown();
  return 0;
}