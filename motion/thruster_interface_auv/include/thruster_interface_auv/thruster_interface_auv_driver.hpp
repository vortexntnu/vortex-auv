#ifndef THRUSTER_INTERFACE_AUV__THRUSTER_INTERFACE_AUV_DRIVER_HPP_
#define THRUSTER_INTERFACE_AUV__THRUSTER_INTERFACE_AUV_DRIVER_HPP_

#include <asio.hpp>
#include <utility>

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <thread>
#include <vector>

/**
 * @brief struct to hold the parameters for a single thruster
 */
struct ThrusterParameters {
    std::uint8_t mapping;
    std::int8_t direction;
    std::uint16_t pwm_min;
    std::uint16_t pwm_max;
};

enum PolySide {
    LEFT = 0,
    RIGHT = 1
};  // vector index for the position of the coefficients in the coeff vector

using FaultEventCallback =
    std::function<void(std::uint8_t channel, std::uint8_t code)>;

using PGoodEventCallback =
    std::function<void(std::uint8_t channel, std::uint8_t code)>;

using KillswitchEventCallback = std::function<void()>;

using CurrentMeasurementsCallback =
    std::function<void(const std::array<float, 8>& currents)>;

/**
 * @brief class instantiated by ThrusterInterfaceAUVNode to control the
 * thrusters, takes the thruster forces and converts them to PWM signals to be
 * sent via UART to the ESC controller.
 *
 * @details Based on the datasheets found in /resources, approximate the map
 * with a piecewise (>0 and <0) third order polynomial.
 *
 * @note The coefficients are found in the config.yaml for all the possible
 * SYSTEM.OPERATIONAL_VOLTAGE values, but we use only 16V for now, so: removed
 * all the handling of the other voltages to save resources. Could be
 * re-implemented in the future for more flexibility if we ever need it to
 * operate at different voltages in different situations.
 *
 * @note Over UART, the PWM values are packed into a framed packet:
 * [magic][id][length][payload][checksum], where the payload is 8 uint16_t.
 */
class ThrusterInterfaceAUVDriver {
   public:
    ~ThrusterInterfaceAUVDriver();

    /**
     * @brief called from ThrusterInterfaceAUVNode .cpp when instantiating the
     * object, initializes all the params.
     *
     * @param serial_device         serial device used to communicate
     *                              (for example /dev/ttyUSB0)
     * @param baud_rate             UART baud rate
     * @param packet_id             packet ID sent in the UART frame
     * @param thruster_parameters   describe mapping, direction, min and max pwm
     *                              value for each thruster
     * @param right_coeffs          RIGHT(>0) third order polynomial
     * coefficients
     * @param left_coeffs           LEFT(<0) third order polynomial coefficients
     */
    ThrusterInterfaceAUVDriver(
        const std::string& serial_device,
        unsigned int baud_rate,
        const std::vector<ThrusterParameters>& thruster_parameters,
        const std::vector<double>& right_coeffs,
        const std::vector<double>& left_coeffs);

    /**
     * @brief initializes UART
     * @return 0 on success, negative number on failure
     */
    int init_uart();

    /**
     * @brief calls both 1) interpolate_forces_to_pwm() to
     * convert the thruster forces to PWM values and 2) send_data_to_escs() to
     * send them over UART
     *
     * @param thruster_forces_array vector of forces for each thruster
     *
     * @return std::optional<std::vector<uint16_t>> vector of pwm values sent to
     * each thruster, or std::nullopt on failure
     */
    std::optional<std::vector<std::uint16_t>> drive_thrusters(
        const std::vector<double>& thruster_forces_array);

    /**
     * @brief Sets the camera light intensity
     *
     * @param[in] percentage float in the range 0-1
     * @return 0 on success -1 on failure
     */
    int set_camera_light(float percentage);

    void set_fault_event_callback(FaultEventCallback callback);
    void set_pgood_event_callback(PGoodEventCallback callback);
    void set_killswitch_event_callback(KillswitchEventCallback callback);
    void set_current_measurements_callback(
        CurrentMeasurementsCallback callback);

   private:
    /**
     * @brief only take the thruster forces and return PWM values
     *
     * @param thruster_forces_array vector of forces for each thruster
     *
     * @return std::vector<uint16_t> vector of pwm values sent to each thruster
     * if we want to publish them for debug purposes
     */
    std::vector<std::uint16_t> interpolate_forces_to_pwm(
        const std::vector<double>& thruster_forces_array);

    /**
     * @brief scalar map from force to pwm x->y. Choose coefficients [LEFT] or
     * [RIGHT] based on sign(force)
     *
     * @param force scalar force value
     *
     * @return std::uint16_t scalar pwm value
     */
    std::uint16_t force_to_pwm(double force);

    /**
     * @brief compute y = a*x^3 + b*x^2 + c*x + d
     *
     * @param force x
     * @param coeffs a,b,c,d
     *
     * @return std::uint16_t pwm value
     */
    std::uint16_t calc_poly(double force, const std::vector<double>& coeffs);

    /**
     * @brief only takes the pwm values computed and sends them
     * over UART as a framed packet
     *
     * @param thruster_pwm_array vector of pwm values to send
     * @return 0 on success, -1 on failure
     */
    int send_data_to_escs(const std::vector<std::uint16_t>& thruster_pwm_array);

    /**
     * @brief create UART packet with format:
     * [magic][id][length][payload][checksum]
     *
     * @param id packet ID
     * @param thruster_pwm_array vector of 8 pwm values to send as payload
     *
     * @return std::vector<uint8_t> serialized packet bytes
     */
    std::vector<std::uint8_t> create_packet(
        std::uint8_t id,
        const std::vector<std::uint16_t>& thruster_pwm_array) const;

    /**
     * @brief convert Newtons to Kg
     *
     * @param force Newtons
     *
     * @return double Kg
     */
    static constexpr double to_kg(double force) { return force / 9.80665; }

    /**
     * @brief start the asynchronous UART receive loop
     */
    void start_receive();

    /**
     * @brief issue one asynchronous read on the UART port
     */
    void do_receive();

    /**
     * @brief process the accumulated receive buffer and extract valid frames
     */
    void process_receive_buffer();

    /**
     * @brief handle one decoded UART frame
     *
     * @param frame_bytes complete frame bytes including header and checksum
     */
    void handle_received_frame(const std::vector<std::uint8_t>& frame_bytes);

    /**
     * @brief compute checksum for framed UART packets
     *
     * @param msg_id message id
     * @param length payload length
     * @param payload pointer to payload bytes
     *
     * @return checksum byte
     */
    static std::uint8_t compute_checksum(std::uint8_t msg_id,
                                         std::uint8_t length,
                                         const std::uint8_t* payload);

   private:
    static constexpr std::uint8_t UART_START_BYTE = 0xAA;
    static constexpr std::size_t MAX_PAYLOAD_SIZE = 64;
    static constexpr std::size_t READ_CHUNK_SIZE = 256;

    // Outgoing
    static constexpr std::uint8_t MSG_TURN_THRUSTERS_OFF = 0x01U;
    static constexpr std::uint8_t MSG_TURN_LIGHTS_OFF = 0x02U;
    static constexpr std::uint8_t MSG_RESET = 0x03;
    static constexpr std::uint8_t MSG_SET_THRUSTER_PWM = 0x04;
    static constexpr std::uint8_t MSG_SET_LIGHT_PWM = 0x05;
    // Incoming
    static constexpr std::uint8_t MSG_FLT_EVENT = 0x10;
    static constexpr std::uint8_t MSG_PGOOD_EVENT = 0x11;
    static constexpr std::uint8_t MSG_KILLSWITCH_EVENT = 0x12;
    static constexpr std::uint8_t MSG_CURRENT_MEASUREMENTS = 0x13;

    std::string serial_device_;
    unsigned int baud_rate_;

    asio::io_context io_;
    asio::serial_port serial_{io_};
    std::thread io_thread_;

    std::vector<ThrusterParameters> thruster_parameters_;
    std::vector<double> right_coeffs_;
    std::vector<double> left_coeffs_;
    std::uint16_t idle_pwm_value_{1500};

    std::array<std::uint8_t, READ_CHUNK_SIZE> read_buf_{};
    std::vector<std::uint8_t> receive_buffer_;

    FaultEventCallback fault_event_callback_;
    PGoodEventCallback pgood_event_callback_;
    KillswitchEventCallback killswitch_event_callback_;
    CurrentMeasurementsCallback current_measurements_callback_;

};

#endif  // THRUSTER_INTERFACE_AUV__THRUSTER_INTERFACE_AUV_DRIVER_HPP_
