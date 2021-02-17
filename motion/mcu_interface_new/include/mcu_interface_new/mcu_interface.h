#include "ros/ros.h"

#include <vortex_msgs/Pwm.h>
#include <vortex_msgs/ThrusterForces.h>

#include <string>
#include <cmath>
#include <vector>

class MCU_Interface{
    public:
        MCU_Interface();

        /** Calculate PWM from thruster forces and write to MCU*/

        void execute();
    private:
        ros::NodeHandle nh;
        ros::Subscriber thruster_forces_sub;
        ros::Rate loop_rate;

        int num_thrusters;
        std::vector<double> thrust_offset;
        std::vector<double> lookup_thrust;
        std::vector<double> lookup_pulse_width;
        std::vector<double> thruster_mapping;
        std::vector<double> thruster_direction;

        const int thrust_range_limit = 100;

        /** Callbacks */
        void thruster_forces_cb(const vortex_msgs::ThrusterForces &msg) ;

        /** Utility */
        bool is_healthy(const vortex_msgs::ThrusterForces &msg);

        double thrust_to_microseconds(const double thrust);
        void output_to_zero();

        void transfer_to_mcu(std::vector<double> pwm);



};