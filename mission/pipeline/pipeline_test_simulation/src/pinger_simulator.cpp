#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pipeline_test_interfaces/msg/pinger_doa.hpp"
#include <random>
#include <cmath>
using namespace std::chrono_literals;

class PingerSimulator : public rclcpp::Node {
public:
  PingerSimulator() : Node("pinger_simulator"),
    rng_(rd_()), delay_sec_(5, 15), noise_(0.0, 0.05) {

    declare_parameter("odom_topic", "/orca/odom");    // <- adjust if needed
    declare_parameter("pinger_x", 8.0);
    declare_parameter("pinger_y", 4.0);
    declare_parameter("pinger_z", -1.0);

    odom_topic_ = get_parameter("odom_topic").as_string();
    pinger_pos_[0] = get_parameter("pinger_x").as_double();
    pinger_pos_[1] = get_parameter("pinger_y").as_double();
    pinger_pos_[2] = get_parameter("pinger_z").as_double();

    pub_ = create_publisher<pipeline_test_interfaces::msg::PingerDOA>("/pinger/doa", 10);
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      [this](nav_msgs::msg::Odometry::SharedPtr msg){
        auv_pos_[0] = msg->pose.pose.position.x;
        auv_pos_[1] = msg->pose.pose.position.y;
        auv_pos_[2] = msg->pose.pose.position.z;
        got_odom_ = true;
      });

    detection_delay_ = std::chrono::seconds(delay_sec_(rng_));
    start_time_ = now();

    timer_ = create_wall_timer(200ms, std::bind(&PingerSimulator::tick, this));

    RCLCPP_INFO(get_logger(),
      "Pinger at [%.2f, %.2f, %.2f], listening to odom: %s",
      pinger_pos_[0], pinger_pos_[1], pinger_pos_[2], odom_topic_.c_str());
  }

private:
  void tick() {
    if (!got_odom_) return;

    // DOA = normalize(pinger - auv)
    double dx = pinger_pos_[0] - auv_pos_[0];
    double dy = pinger_pos_[1] - auv_pos_[1];
    double dz = pinger_pos_[2] - auv_pos_[2];
    double n = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (n < 1e-6) return;
    dx /= n; dy /= n; dz /= n;

    pipeline_test_interfaces::msg::PingerDOA out;
    out.doa.header.frame_id = "odom";
    out.doa.header.stamp = now();
    out.doa.vector.x = dx;
    out.doa.vector.y = dy;
    out.doa.vector.z = dz;

    double elapsed = (now() - start_time_).seconds();
    if (elapsed < detection_delay_.count()) {
      confidence_ = std::clamp(confidence_ + 0.02 + noise_(rng_), 0.0, 1.0);
      out.detected = false;
    } else {
      confidence_ = std::clamp(0.9 + 0.05*noise_(rng_), 0.0, 1.0);
      out.detected = true;
    }
    out.confidence = confidence_;

    pub_->publish(out);
  }

  std::string odom_topic_;
  rclcpp::Publisher<pipeline_test_interfaces::msg::PingerDOA>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
  std::random_device rd_;
  std::mt19937 rng_;
  std::uniform_int_distribution<int> delay_sec_;
  std::normal_distribution<double> noise_;
  std::chrono::seconds detection_delay_;
  double pinger_pos_[3];
  double auv_pos_[3]{0,0,0};
  double confidence_ = 0.0;
  bool got_odom_ = false;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PingerSimulator>());
  rclcpp::shutdown();
  return 0;
}
