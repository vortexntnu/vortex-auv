#ifndef THRUSTER_INTERFACE_AUV_DRIVER_HPP
#define THRUSTER_INTERFACE_AUV_DRIVER_HPP

#include <vector>
#include <string>
#include <map>
#include <cstdint>

class ThrusterInterfaceAUVDriver {
public:
    ThrusterInterfaceAUVDriver(
        int I2C_BUS = 1,
        int PICO_I2C_ADDRESS = 0x21,
        double SYSTEM_OPERATIONAL_VOLTAGE = 16.0,
        const std::string& ROS2_PACKAGE_NAME_FOR_THRUSTER_DATASHEET = "",
        const std::vector<int>& THRUSTER_MAPPING = {7,6,5,4,3,2,1,0},
        const std::vector<int>& THRUSTER_DIRECTION = {1,1,1,1,1,1,1,1},
        const std::vector<int>& THRUSTER_PWM_OFFSET = {0,0,0,0,0,0,0,0},
        const std::vector<int>& PWM_MIN = {1100,1100,1100,1100,1100,1100,1100,1100},
        const std::vector<int>& PWM_MAX = {1900,1900,1900,1900,1900,1900,1900,1900},
        const std::map<int, std::map<std::string, std::vector<double>>>& coeffs = {}
    );

    ~ThrusterInterfaceAUVDriver();

    std::vector<int16_t> drive_thrusters(const std::vector<double>& thruster_forces_array);

private:
    int bus_fd;
    int PICO_I2C_ADDRESS;
    int SYSTEM_OPERATIONAL_VOLTAGE;

    std::vector<int> THRUSTER_MAPPING;
    std::vector<int> THRUSTER_DIRECTION;
    std::vector<int> THRUSTER_PWM_OFFSET;
    std::vector<int> PWM_MIN;
    std::vector<int> PWM_MAX;

    std::map<int, std::map<std::string, std::vector<double>>> coeffs;

    std::vector<int16_t> interpolate_forces_to_pwm(const std::vector<double>& thruster_forces_array);
    void send_data_to_escs(const std::vector<int16_t>& thruster_pwm_array);
    double polyval(const std::vector<double>& coeffs, double x);
};

#endif // THRUSTER_INTERFACE_AUV_DRIVER_HPP
