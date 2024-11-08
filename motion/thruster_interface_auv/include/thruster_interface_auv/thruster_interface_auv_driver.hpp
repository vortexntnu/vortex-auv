#ifndef THRUSTER_INTERFACE_AUV_DRIVER_HPP
#define THRUSTER_INTERFACE_AUV_DRIVER_HPP

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#define IDLE_PWM_VALUE 1500
struct ThrusterParameters {
    std::vector<uint8_t> mapping;
    std::vector<int8_t> direction;
    std::vector<uint16_t> pwm_min;
    std::vector<uint16_t> pwm_max;
};
struct PolyCoeffs {
    std::vector<double> left;
    std::vector<double> right;
};

/**
 * @brief class instantiated by ThrusterInterfaceAUVNode to control the
 * thrusters, takes the thruster forces and converts them to PWM signals to be
 * sent via I2C to the ESCs (PCA9685 Adafruit 16-Channel 12-bit PWM/Servo
 * Driver)
 *
 * @details Based on the datasheets found in /resources, approximate the map
 * with a piecewise (>0 and <0) third order polynomial.
 *
 * @note The coefficients are found in the config.yaml for all the possible
 * SYSTEM.OPERATIONAL_VOLTAGE values, but we use only 16V for now, so: removed
 * all the handling of the other voltages to save resources. Could be
 * re-implemented in the future for more flexibility if we ever need it to
 * operate at different voltages in different situations.
 */
class ThrusterInterfaceAUVDriver {
   public:
    /**
     * @brief empty default constructor called when declaring the object in the
     * .hpp file. Doesn't initialize any params.
     */
    ~ThrusterInterfaceAUVDriver();

    /**
     * @brief actually used constructor. Called from ThrusterInterfaceAUVNode
     * .cpp when instantiating the object, initializes all the params.
     *
     * @param i2c_bus             bus number used to communicate
     * @param pico_i2c_address    i2c address of the ESC that drive the
     * thrusters
     * @param thruster_mapping    pin to motor mapping for the thrusters i.e. if
     * thruster_to_pin = [7,6 ...] then thruster 0 is pin 1 etc..
     * @param thruster_direction  physical mounting direction of the thrusters,
     * (+-1)
     * @param pwm_min             minimum clamping pwm values
     * @param pwm_max             maximum clamping pwm values
     * @param left_coeffs         third order polynomial coefficients when force
     * < 0
     * @param right_coeffs        third order polynomial coefficients when force
     * > 0
     */
    ThrusterInterfaceAUVDriver(
        short i2c_bus,
        int pico_i2c_address,
        const std::vector<ThrusterParameters>& thruster_parameters,
        const std::vector<PolyCoeffs>& approx_poly_coeffs);
    /**
     * @brief [PUBLIC] method that calls 1) interpolate_forces_to_pwm() to
     * convert the thruster forces to PWM values 2) send_data_to_escs() to send
     * them to the ESCs via I2C
     *
     * @param thruster_forces_array vector of forces for each thruster
     *
     * @return std::vector<int16_t> vector of pwm values sent to each thruster
     */
    std::vector<int16_t> drive_thrusters(
        const std::vector<double>& thruster_forces_array);

   private:
    int bus_fd_;  ///< file descriptor for the I2C bus (integer >0 that uniquely
                  ///< identifies the device. -1 if it fails)

    int i2c_bus_;
    int pico_i2c_address_;
    std::vector<ThrusterParameters> thruster_parameters_;
    std::vector<PolyCoeffs> poly_coeffs_;

    /**
     * @brief [private] method that just take the thruster forces and return PWM
     * values
     *
     * @param thruster_forces_array vector of forces for each thruster
     *
     * @return std::vector<int16_t> vector of pwm values sent to each thruster
     * if we want to publish them for debug purposes
     */
    std::vector<int16_t> interpolate_forces_to_pwm(
        const std::vector<double>& thruster_forces_array);

    std::int16_t force_to_pwm(double force,
                              const std::vector<PolyCoeffs>& coeffs);

    std::int16_t interpolate_pwm(double force,
                                 const std::vector<double>& coeffs);

    /**
     * @brief [private] method that takes the pwm values computed and sends them
     * to the ESCs via I2C
     *
     * @param thruster_pwm_array vector of pwm values to send
     */
    void send_data_to_escs(const std::vector<int16_t>& thruster_pwm_array);

    static constexpr double to_kg(double force) { return force / 9.80665; }
    static constexpr std::array<std::uint8_t, 2> pwm_to_i2c_data(
        std::int16_t pwm) {
        return {static_cast<std::uint8_t>((pwm >> 8) & 0xFF),
                static_cast<std::uint8_t>(pwm & 0xFF)};
    }
};

#endif  // THRUSTER_INTERFACE_AUV_DRIVER_HPP
