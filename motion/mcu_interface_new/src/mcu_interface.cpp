#include "mcu_interface_new/mcu_interface.h"

MCU_Interface::MCU_Interface ():loop_rate(10){
    thruster_forces_sub = nh.subscribe("/thrust/thruster_forces", 10, &MCU_Interface::thruster_forces_cb, this);


    if (!nh.getParam("/propulsion/thrusters/num", num_thrusters)) {
        ROS_WARN("Could not get parameter '/propulsion/thrusters/num', using default.");
        num_thrusters = 8;
    }

    std::vector<double> default_vec(num_thrusters, 0.0);

    if (!nh.getParam("/propulsion/thrusters/offset", thrust_offset)) {
        ROS_WARN("Could not get parameter '/propulsion/thrusters/offset', using default.");
        thrust_offset = default_vec;
    }

    if (!nh.getParam("/propulsion/thrusters/characteristics/thrust",  lookup_thrust)) {
        ROS_WARN("Could not get parameter '/propulsion/thrusters/characteristics/thrust', using default.");
        thrust_offset = default_vec;
    }

    if (!nh.getParam("/propulsion/thrusters/characteristics/pulse_width", lookup_pulse_width)) {
        ROS_WARN("Could not get parameter '/propulsion/thrusters/characteristics/pulse_width', using default.");
        thrust_offset = default_vec;    
    }

    if (!nh.getParam("/propulsion/thrusters/map", thruster_mapping)) {
        ROS_WARN("Could not get parameter '/propulsion/thrusters/map', using default.");
        thrust_offset = default_vec;
    }

    if (!nh.getParam("/propulsion/thrusters/direction", thruster_direction)) {
        ROS_WARN("Could not get parameter '/propulsion/thrusters/direction', using default.");
        thrust_offset = default_vec; 
    }

}


/** Callbacks */

void MCU_Interface::thruster_forces_cb(const vortex_msgs::ThrusterForces &msg) {

    if (!is_healthy(msg)) return;

    std::vector<double> thrust = msg.thrust;
    std::vector<double> microseconds;

    for(int i = 0; i < num_thrusters; i++) {
        microseconds.push_back(thrust_to_microseconds(thrust[i] + thrust_offset[i]));
    }

    transfer_to_mcu(microseconds);

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
    // Need to implement the equivalent of numpy.interp(thrust, lookup_thrust, lookup_pulse_width)
    return 0;
}

void MCU_Interface::transfer_to_mcu(std::vector<double> pwm) {

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