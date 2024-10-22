#include "thruster_interface_auv_driver.hpp"

ThrusterInterfaceAUVDriver::ThrusterInterfaceAUVDriver(
    int I2C_BUS,
    int PICO_I2C_ADDRESS,
    double SYSTEM_OPERATIONAL_VOLTAGE,
    const std::vector<int>& THRUSTER_MAPPING,
    const std::vector<int>& THRUSTER_DIRECTION,
    const std::vector<int>& THRUSTER_PWM_OFFSET,
    const std::vector<int>& PWM_MIN,
    const std::vector<int>& PWM_MAX,
    const std::map<int, std::map<std::string, std::vector<double>>>& COEFFS
) : I2C_BUS(I2C_BUS),
    PICO_I2C_ADDRESS(PICO_I2C_ADDRESS),
    SYSTEM_OPERATIONAL_VOLTAGE(SYSTEM_OPERATIONAL_VOLTAGE),
    THRUSTER_MAPPING(THRUSTER_MAPPING),
    THRUSTER_DIRECTION(THRUSTER_DIRECTION),
    THRUSTER_PWM_OFFSET(THRUSTER_PWM_OFFSET),
    PWM_MIN(PWM_MIN),
    PWM_MAX(PWM_MAX),
    COEFFS(COEFFS)
{



    //DEBUGGING ONLY-----------------------------------------------------------------------
    std::cout << "I2C_BUS: " << I2C_BUS << std::endl;
    std::cout << "PICO_I2C_ADDRESS: " << PICO_I2C_ADDRESS << std::endl;
    std::cout << "SYSTEM_OPERATIONAL_VOLTAGE: " << SYSTEM_OPERATIONAL_VOLTAGE << std::endl;
    std::cout << "THRUSTER_MAPPING: ";
    for (int i = 0; i < THRUSTER_MAPPING.size(); i++) {
        std::cout << THRUSTER_MAPPING[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "THRUSTER_DIRECTION: ";
    for (int i = 0; i < THRUSTER_DIRECTION.size(); i++) {
        std::cout << THRUSTER_DIRECTION[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "THRUSTER_PWM_OFFSET: ";
    for (int i = 0; i < THRUSTER_PWM_OFFSET.size(); i++) {
        std::cout << THRUSTER_PWM_OFFSET[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "PWM_MIN: ";
    for (int i = 0; i < PWM_MIN.size(); i++) {
        std::cout << PWM_MIN[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "PWM_MAX: ";
    for (int i = 0; i < PWM_MAX.size(); i++) {
        std::cout << PWM_MAX[i] << " ";
    }
    std::cout << std::endl;
    for (auto const& [key, val] : COEFFS) {
        std::cout << "COEFFS[" << key << "]: " << std::endl;
        for (auto const& [key2, val2] : val) {
            std::cout << "COEFFS[" << key << "][" << key2 << "]: ";
            for (int i = 0; i < val2.size(); i++) {
                std::cout << val2[i] << " ";
            }
            std::cout << std::endl;
        }
    }
    //---------------------------------------------------------------------------------------



    // Open the I2C bus
    std::string i2c_filename = "/dev/i2c-" + std::to_string(I2C_BUS);
    bus_fd = open(i2c_filename.c_str(), O_RDWR);
    if (bus_fd < 0) {
        std::cerr << "ERROR: Failed to open I2C bus " << I2C_BUS << std::endl;
    }

    // Convert SYSTEM_OPERATIONAL_VOLTAGE passed as argument to integer field variable
    if (SYSTEM_OPERATIONAL_VOLTAGE < 11.0) {
        this->SYSTEM_OPERATIONAL_VOLTAGE = 10;
    } else if (SYSTEM_OPERATIONAL_VOLTAGE < 13.0) {
        this->SYSTEM_OPERATIONAL_VOLTAGE = 12;
    } else if (SYSTEM_OPERATIONAL_VOLTAGE < 15.0) {
        this->SYSTEM_OPERATIONAL_VOLTAGE = 14;
    } else if (SYSTEM_OPERATIONAL_VOLTAGE < 17.0) {
        this->SYSTEM_OPERATIONAL_VOLTAGE = 16;
    } else if (SYSTEM_OPERATIONAL_VOLTAGE < 19.0) {
        this->SYSTEM_OPERATIONAL_VOLTAGE = 18;
    } else {
        this->SYSTEM_OPERATIONAL_VOLTAGE = 20;
    }
}

ThrusterInterfaceAUVDriver::~ThrusterInterfaceAUVDriver() {
    if (bus_fd >= 0) {
        close(bus_fd);
    }
}

double ThrusterInterfaceAUVDriver::polyval(const std::vector<double>& coeffs, double x) {
    double result = 0.0;
    for (size_t i = 0; i < coeffs.size(); ++i) {
        result += coeffs[i] * std::pow(x, coeffs.size() - i - 1);
    }
    return result;
}

std::vector<int16_t> ThrusterInterfaceAUVDriver::interpolate_forces_to_pwm(const std::vector<double>& thruster_forces_array) {
    // Convert Newtons to Kg (since the thruster datasheet is in Kg)
    std::vector<double> forces_in_kg(thruster_forces_array.size());
    for (size_t i = 0; i < thruster_forces_array.size(); ++i) {
        forces_in_kg[i] = thruster_forces_array[i] / 9.80665;
    }

    // Select the appropriate coefficients based on the operational voltage
    auto left_coeffs = COEFFS[SYSTEM_OPERATIONAL_VOLTAGE]["LEFT"];
    auto right_coeffs = COEFFS[SYSTEM_OPERATIONAL_VOLTAGE]["RIGHT"];

    std::vector<int16_t> interpolated_pwm;
    for (size_t i = 0; i < forces_in_kg.size(); ++i) {
        double force = forces_in_kg[i];
        double pwm = 0.0;
        if (force < 0) {
            pwm = polyval(left_coeffs, force);
        } else if (force == 0.0) {
            pwm = 1500 - THRUSTER_PWM_OFFSET[i];
        } else {
            pwm = polyval(right_coeffs, force);
        }
        interpolated_pwm.push_back(static_cast<int16_t>(pwm));
    }

    return interpolated_pwm;
}

void ThrusterInterfaceAUVDriver::send_data_to_escs(const std::vector<int16_t>& thruster_pwm_array) {
    uint8_t i2c_data_array[16];
    for (size_t i = 0; i < thruster_pwm_array.size(); ++i) {
        i2c_data_array[2*i] = (thruster_pwm_array[i] >> 8) & 0xFF; // MSB
        i2c_data_array[2*i + 1] = thruster_pwm_array[i] & 0xFF;    // LSB
    }

    // Set the I2C slave address
    if (ioctl(bus_fd, I2C_SLAVE, PICO_I2C_ADDRESS) < 0) {
        std::cerr << "ERROR: Failed to set I2C slave address" << std::endl;
        return;
    }

    // Write data to the I2C device
    if (write(bus_fd, i2c_data_array, 16) != 16) {
        std::cerr << "ERROR: Failed to write to I2C device" << std::endl;
    }
}

std::vector<int16_t> ThrusterInterfaceAUVDriver::drive_thrusters(const std::vector<double>& thruster_forces_array) {
    // Apply thruster mapping and direction
    std::vector<double> mapped_forces(thruster_forces_array.size());
    for (size_t i = 0; i < THRUSTER_MAPPING.size(); ++i) {
        mapped_forces[i] = thruster_forces_array[THRUSTER_MAPPING[i]] * THRUSTER_DIRECTION[i];
    }

    // Convert forces to PWM
    std::vector<int16_t> thruster_pwm_array = interpolate_forces_to_pwm(mapped_forces);

    // Apply thruster offset and limit PWM if needed
    for (size_t i = 0; i < thruster_pwm_array.size(); ++i) {
        thruster_pwm_array[i] += THRUSTER_PWM_OFFSET[i];

        // Clamp the PWM signal
        if (thruster_pwm_array[i] < PWM_MIN[i]) {
            thruster_pwm_array[i] = PWM_MIN[i];
        } else if (thruster_pwm_array[i] > PWM_MAX[i]) {
            thruster_pwm_array[i] = PWM_MAX[i];
        }
    }

    // Send data through I2C
    try {
        send_data_to_escs(thruster_pwm_array);
    } catch (...) {
        std::cerr << "ERROR: Failed to send PWM values" << std::endl;
    }

    return thruster_pwm_array;
}
