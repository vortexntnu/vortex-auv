#ifndef THRUSTER_INTERFACE_AUV_DRIVER_HPP
#define THRUSTER_INTERFACE_AUV_DRIVER_HPP

#include <vector>
#include <string>
#include <iostream>
#include <Eigen/Dense>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <map>
#include <sys/ioctl.h>

class ThrusterInterfaceAUVDriver {
public:
    explicit ThrusterInterfaceAUVDriver(int I2C_BUS = 1, 
                               int PICO_I2C_ADDRESS = 0x21, 
                               double SYSTEM_OPERATIONAL_VOLTAGE = 16.0,
                               std::string ROS2_PACKAGE_NAME_FOR_THRUSTER_DATASHEET = "",
                               std::vector<int> THRUSTER_MAPPING = {7, 6, 5, 4, 3, 2, 1, 0},
                               std::vector<int> THRUSTER_DIRECTION = {1, 1, 1, 1, 1, 1, 1, 1},
                               std::vector<int> THRUSTER_PWM_OFFSET = {0, 0, 0, 0, 0, 0, 0, 0},
                               std::vector<int> PWM_MIN = {1100, 1100, 1100, 1100, 1100, 1100, 1100, 1100},
                               std::vector<int> PWM_MAX = {1900, 1900, 1900, 1900, 1900, 1900, 1900, 1900},
                               std::map<int, std::map<std::string, Eigen::VectorXd>> coeffs = {});

    std::vector<int> drive_thrusters(const std::vector<double>& thruster_forces_array);

private:
    int i2c_fd;
    int PICO_I2C_ADDRESS;
    std::vector<int> THRUSTER_MAPPING;
    std::vector<int> THRUSTER_DIRECTION;
    std::vector<int> THRUSTER_PWM_OFFSET;
    std::vector<int> PWM_MIN;
    std::vector<int> PWM_MAX;
    double SYSTEM_OPERATIONAL_VOLTAGE;
    std::string ROS2_PACKAGE_NAME_FOR_THRUSTER_DATASHEET;
    std::map<int, std::map<std::string, Eigen::VectorXd>> coeffs;

    std::vector<int> _interpolate_forces_to_pwm(const std::vector<double>& thruster_forces_array);
    void _send_data_to_escs(const std::vector<int>& thruster_pwm_array);
};

#endif // THRUSTER_INTERFACE_AUV_DRIVER_HPP
