#ifndef THRUSTER_INTERFACE_AUV_DRIVER_HPP
#define THRUSTER_INTERFACE_AUV_DRIVER_HPP

#include <vector>
#include <string>
#include <map>
#include <cstdint>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>

class ThrusterInterfaceAUVDriver {
public:

    ThrusterInterfaceAUVDriver(
        int I2C_BUS = 1,
        int PICO_I2C_ADDRESS = 0xAA,
        double SYSTEM_OPERATIONAL_VOLTAGE = 12.1,
        const std::vector<int>& THRUSTER_MAPPING = {},
        const std::vector<int>& THRUSTER_DIRECTION = {},
        const std::vector<int>& THRUSTER_PWM_OFFSET = {},
        const std::vector<int>& PWM_MIN = {},
        const std::vector<int>& PWM_MAX = {},
        const std::map<int, std::map<std::string, std::vector<double>>>& COEFFS = {}
    );

    ~ThrusterInterfaceAUVDriver();

    std::vector<int16_t> drive_thrusters(const std::vector<double>& thruster_forces_array);

private:

    int bus_fd;
    int I2C_BUS;
    int PICO_I2C_ADDRESS;
    int SYSTEM_OPERATIONAL_VOLTAGE;

    std::vector<int> THRUSTER_MAPPING;
    std::vector<int> THRUSTER_DIRECTION;
    std::vector<int> THRUSTER_PWM_OFFSET;
    std::vector<int> PWM_MIN;
    std::vector<int> PWM_MAX;

    std::map<int, std::map<std::string, std::vector<double>>> COEFFS;

    std::vector<int16_t> interpolate_forces_to_pwm(const std::vector<double>& thruster_forces_array);
    void send_data_to_escs(const std::vector<int16_t>& thruster_pwm_array);
    double polyval(const std::vector<double>& coeffs, double x);
};

#endif // THRUSTER_INTERFACE_AUV_DRIVER_HPP
