#include "ros/ros.h"
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <vector>
#include <boost/array.hpp>

#include "vortex_msgs/PropulsionCommand.h"

class ControllerTest : public ::testing::Test
{
public:
  ControllerTest()
  {
    cmdPub = nh.advertise<vortex_msgs::PropulsionCommand>("propulsion_command", 10);
    sub = nh.subscribe("rov_forces", 10, &ControllerTest::Callback, this);
    message_received = false;
  }

  void SetUp()
  {
    while (!IsNodeReady())
      ros::spinOnce();
  }

  void PublishCommand(boost::array<double, 6> motion, std::vector<uint8_t> mode)
  {
    vortex_msgs::PropulsionCommand msg;
    msg.motion = motion;
    msg.control_mode = mode;
    cmdPub.publish(msg);
  }

  void ExpectTauNear(std::vector<double> arr)
  {
    for (int i = 0; i < tau.size(); ++i)
      EXPECT_NEAR(tau[i], arr[i], MAX_ERROR);
  }

  void WaitForMessage()
  {
    while (!message_received)
      ros::spinOnce();
  }

  Eigen::Matrix<double, 6, 1> tau;
  const double MAX_ERROR = 0.0001;

private:
  ros::NodeHandle nh;
  ros::Publisher cmdPub;
  ros::Subscriber sub;

  bool message_received;

  void Callback(const geometry_msgs::Wrench& msg)
  {
    tf::wrenchMsgToEigen(msg, tau);
    message_received = true;
  }

  bool IsNodeReady()
  {
    return ((cmdPub.getNumSubscribers() > 0) && (sub.getNumPublishers() > 0));
  }
};

TEST_F(ControllerTest, CheckResponsiveness)
{
  WaitForMessage();
}

TEST_F(ControllerTest, OpenLoop)
{
  PublishCommand({1, 0, 0, 0, 0, 0}, {1, 0, 0, 0});  // NOLINT(whitespace/braces)
  ros::Duration(0.5).sleep();
  WaitForMessage();

  ExpectTauNear({48.895, 0, 0, 0, 0, 0});  // NOLINT(whitespace/braces)
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "controller_test");

  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
