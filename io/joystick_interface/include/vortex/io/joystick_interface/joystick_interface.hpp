#pragma once

#include <boost/asio.hpp>

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>

namespace vortex::io
{

enum class JoystickModeRequest : std::uint8_t
{
    None = 0,
    Manual,
    Reference,
    Autonomous,
};

struct JoystickState
{
    /*
     * Normalized control demand in [-1.0, 1.0].
     *
     * These are continuously updated whenever a valid UDP command arrives.
     */
    double surge {0.0};
    double sway {0.0};
    double heave {0.0};

    double roll {0.0};
    double pitch {0.0};
    double yaw {0.0};

    double gripper_rotate {0.0};
    double gripper_pitch {0.0};
    double gripper_grip {0.0};

    /*
     * Metadata useful to the control/safety layer.
     */
    std::uint16_t sequence {0U};
    bool link_alive {false};

    std::chrono::steady_clock::time_point last_packet_time {};
};

struct JoystickEvent
{
    JoystickModeRequest mode_request {JoystickModeRequest::None};
    bool toggle_killswitch {false};

    [[nodiscard]] bool empty() const
    {
        return mode_request == JoystickModeRequest::None &&
               !toggle_killswitch;
    }
};

/*
 * Receives joystick commands over UDP and exposes them as normal,
 * normalized C++ structs.
 *
 * Thread-safe:
 * - Boost.Asio may run on one thread.
 * - Your control loop may read state() from another thread.
 */
class JoystickInterface
{
public:
    using udp = boost::asio::ip::udp;

    explicit JoystickInterface(
        boost::asio::io_context& io_context,
        std::uint16_t listen_port,
        std::chrono::milliseconds timeout =
            std::chrono::milliseconds(200));

    void start();
    void stop();

    /*
     * Returns the latest continuous joystick state.
     *
     * This does not clear anything. It is safe to call every control tick.
     */
    [[nodiscard]] JoystickState state() const;

    /*
     * Returns and clears accumulated one-shot events.
     *
     * A mode request or killswitch toggle should not disappear merely because
     * another UDP joystick packet arrives before your control loop runs.
     */
    [[nodiscard]] std::optional<JoystickEvent> consume_event();

    /*
     * Optional: lock input to one known joystick sender.
     *
     * Call this once you know the operator laptop's IP address.
     * A port of 0 means "accept any source port from this IP".
     */
    void set_allowed_sender(
        const boost::asio::ip::address& address,
        std::uint16_t port = 0U);

    void clear_allowed_sender();

private:
    static constexpr std::size_t kReceiveBufferSize = 128U;

    static constexpr std::uint32_t kPacketMagic = 0x56584A53U; // VXJS
    static constexpr std::uint16_t kProtocolVersion = 1U;
    static constexpr std::size_t kPacketSize = 28U;

    struct AllowedSender
    {
        boost::asio::ip::address address;
        std::uint16_t port {0U};
    };

    void receive_next();

    void handle_receive(
        const boost::system::error_code& error,
        std::size_t bytes_received);

    void arm_watchdog();

    void handle_watchdog(
        const boost::system::error_code& error);

    [[nodiscard]] bool sender_is_allowed(
        const udp::endpoint& sender) const;

    /*
     * Parses and validates the UDP packet.
     *
     * Returns false for malformed packets, wrong packet versions,
     * unsupported modes, bad magic, or unexpected packet size.
     */
    [[nodiscard]] bool parse_packet(
        const std::uint8_t* bytes,
        std::size_t size,
        JoystickState& parsed_state,
        JoystickEvent& parsed_event) const;

    static std::uint16_t read_u16_be(const std::uint8_t* bytes);

    static std::uint32_t read_u32_be(const std::uint8_t* bytes);

    static double normalize_axis(std::int16_t value);

    boost::asio::io_context& io_context_;

    udp::socket socket_;
    udp::endpoint sender_endpoint_;
    boost::asio::steady_timer watchdog_timer_;

    std::chrono::milliseconds timeout_;

    std::array<std::uint8_t, kReceiveBufferSize> receive_buffer_ {};

    mutable std::mutex mutex_;

    JoystickState latest_state_ {};
    JoystickEvent pending_event_ {};
    std::optional<AllowedSender> allowed_sender_;

    bool running_ {false};
};

}  // namespace vortex::io
