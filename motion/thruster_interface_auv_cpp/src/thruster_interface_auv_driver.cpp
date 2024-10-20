#include "thruster_interface_auv_cpp/thruster_interface_auv_driver.hpp"

ThrusterInterfaceAUVDriver::ThrusterInterfaceAUVDriver(int I2C_BUS, int PICO_I2C_ADDRESS, double SYSTEM_OPERATIONAL_VOLTAGE,
                                                       std::string ROS2_PACKAGE_NAME_FOR_THRUSTER_DATASHEET,
                                                       std::vector<int> THRUSTER_MAPPING, std::vector<int> THRUSTER_DIRECTION,
                                                       std::vector<int> THRUSTER_PWM_OFFSET, std::vector<int> PWM_MIN,
                                                       std::vector<int> PWM_MAX,
                                                       std::map<int, std::map<std::string, Eigen::VectorXd>> coeffs)
    : PICO_I2C_ADDRESS(PICO_I2C_ADDRESS), THRUSTER_MAPPING(THRUSTER_MAPPING), THRUSTER_DIRECTION(THRUSTER_DIRECTION),
      THRUSTER_PWM_OFFSET(THRUSTER_PWM_OFFSET), PWM_MIN(PWM_MIN), PWM_MAX(PWM_MAX), SYSTEM_OPERATIONAL_VOLTAGE(SYSTEM_OPERATIONAL_VOLTAGE),
      ROS2_PACKAGE_NAME_FOR_THRUSTER_DATASHEET(ROS2_PACKAGE_NAME_FOR_THRUSTER_DATASHEET), coeffs(coeffs) {
    
    // Open I2C device
    char i2c_device[20];
    snprintf(i2c_device, 19, "/dev/i2c-%d", I2C_BUS);
    i2c_fd = open(i2c_device, O_RDWR);
    if (i2c_fd < 0) {
        std::cerr << "ERROR: Failed to open I2C bus" << std::endl;
    }
    
    if (ioctl(i2c_fd, I2C_SLAVE, PICO_I2C_ADDRESS) < 0) {
        std::cerr << "ERROR: Failed to set I2C address" << std::endl;
    }

    // Convert SYSTEM_OPERATIONAL_VOLTAGE to the nearest valid value
    if (SYSTEM_OPERATIONAL_VOLTAGE < 11.0) {
        SYSTEM_OPERATIONAL_VOLTAGE = 10;
    } else if (SYSTEM_OPERATIONAL_VOLTAGE < 13.0) {
        SYSTEM_OPERATIONAL_VOLTAGE = 12;
    } else if (SYSTEM_OPERATIONAL_VOLTAGE < 15.0) {
        SYSTEM_OPERATIONAL_VOLTAGE = 14;
    } else if (SYSTEM_OPERATIONAL_VOLTAGE < 17.0) {
        SYSTEM_OPERATIONAL_VOLTAGE = 16;
    } else if (SYSTEM_OPERATIONAL_VOLTAGE < 19.0) {
        SYSTEM_OPERATIONAL_VOLTAGE = 18;
    } else {
        SYSTEM_OPERATIONAL_VOLTAGE = 20;
    }
}

std::vector<int> ThrusterInterfaceAUVDriver::_interpolate_forces_to_pwm(const std::vector<double>& thruster_forces_array) {
    std::vector<int> interpolated_pwm;
    Eigen::VectorXd left_coeffs = coeffs[SYSTEM_OPERATIONAL_VOLTAGE]["LEFT"];
    Eigen::VectorXd right_coeffs = coeffs[SYSTEM_OPERATIONAL_VOLTAGE]["RIGHT"];

    for (size_t i = 0; i < thruster_forces_array.size(); ++i) {
        double force = thruster_forces_array[i] / 9.80665;  // Convert Newtons to Kg
        double pwm = 0;

        if (force < 0) {
            pwm = left_coeffs(0) + left_coeffs(1) * force + left_coeffs(2) * std::pow(force,2) + left_coeffs(3) * std::pow(force,3);
        } else if (force == 0.0) {
            pwm = 1500 - THRUSTER_PWM_OFFSET[i];
        } else {
            pwm = right_coeffs(0) + right_coeffs(1) * force + right_coeffs(2) * std::pow(force,2) + right_coeffs(3) * std::pow(force,3);
        }

        interpolated_pwm.push_back(static_cast<int>(pwm));
    }
    return interpolated_pwm;
}

void ThrusterInterfaceAUVDriver::_send_data_to_escs(const std::vector<int>& thruster_pwm_array) {
    std::vector<uint8_t> i2c_data_array;

    for (size_t i = 0; i < thruster_pwm_array.size(); ++i) {
        uint8_t msb = (thruster_pwm_array[i] >> 8) & 0xFF;
        uint8_t lsb = thruster_pwm_array[i] & 0xFF;
        i2c_data_array.push_back(msb);
        i2c_data_array.push_back(lsb);
    }

    if (write(i2c_fd, i2c_data_array.data(), i2c_data_array.size()) != static_cast<ssize_t>(i2c_data_array.size())) {
        std::cerr << "ERROR: Failed to send data via I2C" << std::endl;
    }
}

std::vector<int> ThrusterInterfaceAUVDriver::drive_thrusters(const std::vector<double>& thruster_forces_array) {
    std::vector<double> thruster_forces_modified(thruster_forces_array.size());

    for (size_t i = 0; i < thruster_forces_array.size(); ++i) {
        thruster_forces_modified[i] = thruster_forces_array[THRUSTER_MAPPING[i]] * THRUSTER_DIRECTION[i];
    }

    std::vector<int> thruster_pwm_array = _interpolate_forces_to_pwm(thruster_forces_modified);

    for (size_t i = 0; i < thruster_pwm_array.size(); ++i) {
        thruster_pwm_array[i] += THRUSTER_PWM_OFFSET[i];

        if (thruster_pwm_array[i] < PWM_MIN[i]) {
            thruster_pwm_array[i] = PWM_MIN[i];
        } else if (thruster_pwm_array[i] > PWM_MAX[i]) {
            thruster_pwm_array[i] = PWM_MAX[i];
        }
    }

    try {
        _send_data_to_escs(thruster_pwm_array);
    } catch (const std::exception& e) {
        std::cerr << "ERROR: Failed to send PWM values: " << e.what() << std::endl;
    }

    return thruster_pwm_array;
}
