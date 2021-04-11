#include "simple_odom.h"

simple_odom::SimpleOdom(ros::NodeHandle nh) nh(nh)
{
    // get params

    // subscribers and publishers

    // wait for first imu and dvl msg

    ROS_INFO("SimpleOdom initialized");
}

simple_odom::spin()
{
    // create odom msg

    // publish odom
}

void imuCallback(const sensor_msgs::Imu &imu_msg)
{
    tf::twistMsgToEigen(odom_msg.twist.twist, velocity);
    orientation = Eigen::Quaterniond(odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x,
                                     odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z);
}



void dvlCallback(const nav_msgs::Odometry &odom_msg)
{
}

int main(int argc, char **argv)
{
    const bool DEBUG_MODE = false; // debug logs are printed to console when true

    ros::init(argc, argv, "velocity_controller");
    ros::NodeHandle nh;

    if (DEBUG_MODE)
    {
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
        ros::console::notifyLoggerLevelsChanged();
    }

    SimpleOdom simple_odom(nh);
    simple_odom::spin();
}
