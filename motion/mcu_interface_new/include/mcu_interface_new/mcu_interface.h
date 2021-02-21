#pragma once

#include "ros/ros.h"

#include <vortex_msgs/Pwm.h>
#include <vortex_msgs/ThrusterForces.h>
#include <std_msgs/String.h>

#include <string>
#include <cmath>
#include <vector>

#include "mcu_interface_new/i2c.h"
#include "mcu_interface_new/interp.h"

class MCU_Interface{
    public:
        MCU_Interface();

        /** Calculate PWM from thruster forces and write to MCU*/

        void execute();
    private:
        I2CDevice device;

        ros::NodeHandle nh;
        ros::Subscriber thruster_forces_sub;
        ros::Subscriber thruster_arm_sub;
        ros::Rate loop_rate;

        int num_thrusters;
        std::vector<double> lookup_thrust;
        std::vector<double> lookup_pulse_width;
        std::vector<double> thruster_offset;
        std::vector<double> thruster_mapping;
        std::vector<double> thruster_direction;

        const int thrust_range_limit = 100;

        /** Callbacks */
        void thruster_forces_cb(const vortex_msgs::ThrusterForces &msg);
        void thruster_arm_cb(const std_msgs::String &msg);

        /** Utility */
        bool is_healthy(const vortex_msgs::ThrusterForces &msg);

        double thrust_to_microseconds(const double thrust);
        void output_to_zero();

        void i2c_init(int MCU_addr);
        void transfer_to_mcu(const std::vector<double> pwm);
        void transfer_to_mcu(u_int8_t a_byte);

};