#include "mcu_interface_new/mcu_interface.h"

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "mcu_interface_new/i2c.h"

#include <time.h>
#include <stdlib.h>

#include "mcu_interface_new/interp.h"
MCU_Interface::MCU_Interface ():loop_rate(10){
    thruster_forces_sub = nh.subscribe("/thrust/thruster_forces", 10, &MCU_Interface::thruster_forces_cb, this);
    thruster_arm_sub = nh.subscribe("/thrust/arm", 10, &MCU_Interface::thruster_arm_cb, this);

    if (!nh.getParam("/propulsion/thrusters/num", num_thrusters)) {
        ROS_WARN("Could not get parameter '/propulsion/thrusters/num', using default.");
        num_thrusters = 8;
    }

    std::vector<double> default_vec(num_thrusters, 0.0);

    if (!nh.getParam("/propulsion/thrusters/characteristics/thrust",  lookup_thrust)) {
        ROS_WARN("Could not get parameter '/propulsion/thrusters/characteristics/thrust', using default.");
        lookup_thrust = default_vec;
    }

    if (!nh.getParam("/propulsion/thrusters/characteristics/pulse_width", lookup_pulse_width)) {
        ROS_WARN("Could not get parameter '/propulsion/thrusters/characteristics/pulse_width', using default.");
        lookup_pulse_width = default_vec;    
    }

    if (!nh.getParam("/propulsion/thrusters/offset", thruster_offset)) {
        ROS_WARN("Could not get parameter '/propulsion/thrusters/offset', using default.");
        thruster_offset = default_vec;
    }

    if (!nh.getParam("/propulsion/thrusters/map", thruster_mapping)) {
        ROS_WARN("Could not get parameter '/propulsion/thrusters/map', using default.");
        thruster_mapping = default_vec;
    }

    if (!nh.getParam("/propulsion/thrusters/direction", thruster_direction)) {
        ROS_WARN("Could not get parameter '/propulsion/thrusters/direction', using default.");
        thruster_direction = default_vec; 
    }

}


/** Callbacks */

void MCU_Interface::thruster_forces_cb(const vortex_msgs::ThrusterForces &msg) {

    if (!is_healthy(msg)) return;

    std::vector<double> thrust = msg.thrust;
    std::vector<double> microseconds;

    for(int i = 0; i < num_thrusters; i++) {
        microseconds.push_back(thrust_to_microseconds(thrust[i] + thruster_offset[i]));
    }

    MCU_Interface::transfer_to_mcu(microseconds);
}

void MCU_Interface::thruster_arm_cb(const std_msgs::String &msg) {
    if (msg.data == "arm me daddy") {
        ROS_INFO("ARMING THRUSTERS, WATCH YOUR FINGERS, TOES, AND ANY OTHER EXPOSED LIMBS");
        // Transfer a string, "command_arm" to mcu
    }

}


/** Utility*/

void MCU_Interface::output_to_zero() {
    double neutral_pulse_width = thrust_to_microseconds(0);
    std::vector<double> microseconds;

    for(int i = 0; i < num_thrusters; i++) {
        microseconds.push_back(neutral_pulse_width);
    }

    transfer_to_mcu(microseconds);

}

bool MCU_Interface::is_healthy(const vortex_msgs::ThrusterForces &msg) {
    std::vector<double> thrust = msg.thrust;

    if( thrust.size() != num_thrusters) {
        ROS_WARN("Size mismatch between thruster force message and defined number of thrusters, ignoring...");
        return false;
    }

    for(const float tau : thrust) {
        if(isnan(tau) || isinf(tau) || abs(tau) > thrust_range_limit) {
            ROS_WARN("Force element out of range, ignoring...");
            return false;
        }
    }

    return true;
}


double MCU_Interface::thrust_to_microseconds(const double thrust) {
    return interpolate(thrust, lookup_thrust, lookup_pulse_width);
}

void MCU_Interface::i2c_init(int MCU_addr) {
    int bus;
    //const char *data = "01110";

    /* First open i2c bus */
    if ((bus = i2c_open("/dev/i2c-8")) == -1) {

        perror("Open i2c bus error");
    }

    /* Fill i2c device struct */
    MCU_Interface::device.bus = bus;
    MCU_Interface::device.addr = MCU_addr;
    MCU_Interface::device.tenbit = 0;
    MCU_Interface::device.delay = 10;
    MCU_Interface::device.flags = 0;
    MCU_Interface::device.page_bytes = 10;
    MCU_Interface::device.iaddr_bytes = 0; /* Set this to zero, and using i2c_ioctl_xxxx API will ignore chip internal address */
}

void MCU_Interface::transfer_to_mcu(const std::vector<double> pwm) {
// To compile and run all files:
// cc -c ./i2c_test.c && cc ./i2c.o ./i2c_test.o -o main && ./main

    std::vector<int> pwm_int_vec(pwm.begin(), pwm.end());
    
    int pwm_arr[num_thrusters];
    std::copy(pwm_int_vec.begin(), pwm_int_vec.end(), pwm_arr);

    const I2CDevice dev_1 = MCU_Interface::device;

    char num_str[num_thrusters];
    for (int i = 0; i < num_thrusters; i++) {
        uint8_t tmp_int8 = 0.128 * pwm_arr[i] - 1;
        num_str[i] = tmp_int8;
    }

    if (i2c_ioctl_write(&dev_1, 0x0, num_str, strlen(num_str)) != strlen(num_str))
    {
        /* Error process */
    }
    // for (u_int16_t i = 9000; i < 100000; i++) {
    //     u_int16_t num = rand() % 65535;
    //     char num_str[5];
    //     sprintf(num_str, "%d", i);

    // 	printf("%s\n", num_str);

    //     /* Write data to i2c */
    //     if (i2c_ioctl_write(&MCU_Interface::device, 0x0, num_str, strlen(num_str)) != strlen(num_str)) {
    //         /* Error process */
    //     }
    // }
    //i2c_close(MCU_Interface::device.bus);
}

void MCU_Interface::transfer_to_mcu(u_int8_t a_byte) {
// To compile and run all files:
// cc -c ./i2c_test.c && cc ./i2c.o ./i2c_test.o -o main && ./main
    const I2CDevice dev_1 = MCU_Interface::device;

    for (int outer = 0; outer < 1000; outer++) {
        char num_str[num_thrusters];
        for (int i = 0; i < num_thrusters; i++)
        {
            num_str[i] = a_byte;
        }

        if (i2c_ioctl_write(&dev_1, 0x0, num_str, strlen(num_str)) != strlen(num_str))
        {
            /* Error process */
        }
    }

    
    // for (u_int16_t i = 9000; i < 100000; i++) {
    //     u_int16_t num = rand() % 65535;
    //     char num_str[5];
    //     sprintf(num_str, "%d", i);

    // 	printf("%s\n", num_str);

    //     /* Write data to i2c */
    //     if (i2c_ioctl_write(&MCU_Interface::device, 0x0, num_str, strlen(num_str)) != strlen(num_str)) {
    //         /* Error process */
    //     }
    // }
    //i2c_close(MCU_Interface::device.bus);
}


void MCU_Interface::execute(){
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv){
    ros::init(argc,argv,"mcu_interface_new");
    MCU_Interface mcu_i;
    mcu_i.execute();
}
