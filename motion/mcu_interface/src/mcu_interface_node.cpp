#include "ros/ros.h"
#include "McuInterface.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mcu_interface");
  ros::NodeHandle nh_mcu;

  std::string device_str;
  const char * device;

  if (!nh_mcu.getParam("/mcu/device", device_str))
  {
    device = "/dev/ttySAC0";
    ROS_WARN("Failed to read parameter MCU device, defaulting to %s ", device);
  }
  else
  {
    device = device_str.c_str();
  }

  McuInterface mcu_interface(device);

  ros::Subscriber arming_sub = nh_mcu.subscribe("/mcu_arm", 1000, &McuInterface::arming_callback, &mcu_interface);
  ros::Subscriber light_pwm_sub = nh_mcu.subscribe("/mcu_light", 1000, &McuInterface::light_pwm_callback, &mcu_interface);
  ros::Subscriber thruster_pwm_sub = nh_mcu.subscribe("/pwm", 1000, &McuInterface::thruster_pwm_callback, &mcu_interface);
  ros::Subscriber heartbeat_sub = nh_mcu.subscribe("/mcu_heartbeat", 1000, &McuInterface::heartbeat_callback, &mcu_interface);

  ros::Publisher read_leak_sensor_pub = nh_mcu.advertise<std_msgs::UInt8>("/mcu_read_leak_sensor", 1000);

  std_msgs::UInt8 msg;
  msg.data = 0;
  double prev_time = ros::Time::now().toSec();

  while (ros::ok())
  {
    if ((ros::Time::now().toSec() - prev_time) > 1.0)
    {
      if (mcu_interface.read_leak_sensor())
      {
        msg.data = 1;
      }
      else
      {
        msg.data = 0;
      }

      read_leak_sensor_pub.publish(msg);
      prev_time = ros::Time::now().toSec();
    }

    ros::spinOnce();
  }
  return 0;
}
