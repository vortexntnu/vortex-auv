#include "ros/ros.h"
#include <vortex_msgs/Pwm.h>

class MCU_Interface{
    public:
        MCU_Interface();

        void pwm_cb(vortex_msgs::Pwm pwm_msg);

        void execute();
    private:
        ros::NodeHandle nh;
        ros::Subscriber pwm_sub;
        ros::Rate loop_rate;

};