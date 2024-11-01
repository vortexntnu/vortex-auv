#ifndef THRUSTER_INTERFACE_AUV_DRIVER_HPP
#define THRUSTER_INTERFACE_AUV_DRIVER_HPP

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <vector>

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
    ThrusterInterfaceAUVDriver();
    ~ThrusterInterfaceAUVDriver();

    /**
     * @brief actually used constructor. Called from ThrusterInterfaceAUVNode
     * .cpp when instantiating the object, initializes all the params.
     *
     * @param I2C_BUS             bus number used to communicate
     * @param PICO_I2C_ADDRESS    i2c address of the ESC that drive the
     * thrusters
     * @param THRUSTER_MAPPING    pin to motor mapping for the thrusters i.e. if
     * thruster_to_pin = [7,6 ...] then thruster 0 is pin 1 etc..
     * @param THRUSTER_DIRECTION  physical mounting direction of the thrusters,
     * (+-1)
     * @param PWM_MIN             minimum clamping pwm values
     * @param PWM_MAX             maximum clamping pwm values
     * @param LEFT_COEFFS         third order polynomial coefficients when force
     * < 0
     * @param RIGHT_COEFFS        third order polynomial coefficients when force
     * > 0
     */
    ThrusterInterfaceAUVDriver(short I2C_BUS,
                               int PICO_I2C_ADDRESS,
                               const std::vector<short>& THRUSTER_MAPPING,
                               const std::vector<short>& THRUSTER_DIRECTION,
                               const std::vector<int>& PWM_MIN,
                               const std::vector<int>& PWM_MAX,
                               const std::vector<double>& LEFT_COEFFS,
                               const std::vector<double>& RIGHT_COEFFS);

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
    int bus_fd;  ///< file descriptor for the I2C bus (integer >0 that uniquely
                 ///< identifies the device. -1 if it fails)
    short I2C_BUS;
    int PICO_I2C_ADDRESS;

    std::vector<short> THRUSTER_MAPPING;
    std::vector<short> THRUSTER_DIRECTION;
    std::vector<int> PWM_MIN;
    std::vector<int> PWM_MAX;
    std::vector<double> LEFT_COEFFS;
    std::vector<double> RIGHT_COEFFS;

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
                              const std::vector<double>& left_coeffs,
                              const std::vector<double>& right_coeffs);

    std::int16_t interpolate_pwm(double force,
                                 const std::vector<double>& coeffs);

    /**
     * @brief [private] method that takes the pwm values computed and sends them
     * to the ESCs via I2C
     *
     * @param thruster_pwm_array vector of pwm values to send
     */
    void send_data_to_escs(const std::vector<int16_t>& thruster_pwm_array);

    static constexpr auto to_kg = [](double force) { return force / 9.80665; };
};

#endif  // THRUSTER_INTERFACE_AUV_DRIVER_HPP
