#include "vortex/io/joystick_interface/joystick_interface.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <utility>

namespace vortex::io
{

JoystickInterface::JoystickInterface(
    boost::asio::io_context& io_context,
    std::uint16_t listen_port,
    std::chrono::milliseconds timeout)
    : io_context_(io_context),
      socket_(
          io_context_,
          udp::endpoint(udp::v4(), listen_port)),
      watchdog_timer_(io_context_),
      timeout_(timeout)
{
    socket_.set_option(boost::asio::socket_base::reuse_address(true));
}

void JoystickInterface::start()
{
    if (running_) {
        return;
    }

    running_ = true;
    receive_next();
}

void JoystickInterface::stop()
{
    if (!running_) {
        return;
    }

    running_ = false;

    boost::system::error_code ignored_error {};

    socket_.cancel(ignored_error);
    socket_.close(ignored_error);
    watchdog_timer_.cancel();

    std::scoped_lock lock(mutex_);

    latest_state_.link_alive = false;

    latest_state_.surge = 0.0;
    latest_state_.sway = 0.0;
    latest_state_.heave = 0.0;

    latest_state_.roll = 0.0;
    latest_state_.pitch = 0.0;
    latest_state_.yaw = 0.0;

    latest_state_.gripper_rotate = 0.0;
    latest_state_.gripper_pitch = 0.0;
    latest_state_.gripper_grip = 0.0;
}

JoystickState JoystickInterface::state() const
{
    std::scoped_lock lock(mutex_);
    return latest_state_;
}

std::optional<JoystickEvent> JoystickInterface::consume_event()
{
    std::scoped_lock lock(mutex_);

    if (pending_event_.empty()) {
        return std::nullopt;
    }

    const JoystickEvent event = pending_event_;
    pending_event_ = {};

    return event;
}

void JoystickInterface::set_allowed_sender(
    const boost::asio::ip::address& address,
    std::uint16_t port)
{
    std::scoped_lock lock(mutex_);

    allowed_sender_ = AllowedSender {
        .address = address,
        .port = port,
    };
}

void JoystickInterface::clear_allowed_sender()
{
    std::scoped_lock lock(mutex_);
    allowed_sender_.reset();
}

void JoystickInterface::receive_next()
{
    socket_.async_receive_from(
        boost::asio::buffer(receive_buffer_),
        sender_endpoint_,
        [this](
            const boost::system::error_code& error,
            std::size_t bytes_received) {
            handle_receive(error, bytes_received);
        });
}

void JoystickInterface::handle_receive(
    const boost::system::error_code& error,
    std::size_t bytes_received)
{
    if (!running_) {
        return;
    }

    if (error) {
        if (error != boost::asio::error::operation_aborted) {
            std::cerr << "Joystick UDP receive failed: "
                      << error.message() << '\n';
        }

        if (running_) {
            receive_next();
        }

        return;
    }

    if (!sender_is_allowed(sender_endpoint_)) {
        receive_next();
        return;
    }

    JoystickState parsed_state {};
    JoystickEvent parsed_event {};

    if (!parse_packet(
            receive_buffer_.data(),
            bytes_received,
            parsed_state,
            parsed_event)) {
        receive_next();
        return;
    }

    parsed_state.link_alive = true;
    parsed_state.last_packet_time = std::chrono::steady_clock::now();

    {
        std::scoped_lock lock(mutex_);

        latest_state_ = parsed_state;

        if (parsed_event.mode_request != JoystickModeRequest::None) {
            pending_event_.mode_request = parsed_event.mode_request;
        }

        if (parsed_event.toggle_killswitch) {
            pending_event_.toggle_killswitch = true;
        }
    }

    arm_watchdog();
    receive_next();
}

void JoystickInterface::arm_watchdog()
{
    watchdog_timer_.cancel();

    watchdog_timer_.expires_after(timeout_);

    watchdog_timer_.async_wait(
        [this](const boost::system::error_code& error) {
            handle_watchdog(error);
        });
}

void JoystickInterface::handle_watchdog(
    const boost::system::error_code& error)
{
    if (!running_ ||
        error == boost::asio::error::operation_aborted) {
        return;
    }

    if (error) {
        std::cerr << "Joystick watchdog error: "
                  << error.message() << '\n';
        return;
    }

    std::scoped_lock lock(mutex_);

    latest_state_.link_alive = false;

    latest_state_.surge = 0.0;
    latest_state_.sway = 0.0;
    latest_state_.heave = 0.0;

    latest_state_.roll = 0.0;
    latest_state_.pitch = 0.0;
    latest_state_.yaw = 0.0;

    latest_state_.gripper_rotate = 0.0;
    latest_state_.gripper_pitch = 0.0;
    latest_state_.gripper_grip = 0.0;
}

bool JoystickInterface::sender_is_allowed(
    const udp::endpoint& sender) const
{
    std::scoped_lock lock(mutex_);

    if (!allowed_sender_.has_value()) {
        return true;
    }

    if (sender.address() != allowed_sender_->address) {
        return false;
    }

    return allowed_sender_->port == 0U ||
           sender.port() == allowed_sender_->port;
}

bool JoystickInterface::parse_packet(
    const std::uint8_t* bytes,
    std::size_t size,
    JoystickState& parsed_state,
    JoystickEvent& parsed_event) const
{
    if (size != kPacketSize) {
        return false;
    }

    if (read_u32_be(bytes + 0U) != kPacketMagic) {
        return false;
    }

    if (read_u16_be(bytes + 4U) != kProtocolVersion) {
        return false;
    }

    const std::uint8_t requested_mode = bytes[8U];
    const std::uint8_t flags = bytes[9U];

    if (requested_mode >
        static_cast<std::uint8_t>(
            JoystickModeRequest::Autonomous)) {
        return false;
    }

    parsed_state.sequence = read_u16_be(bytes + 6U);

    parsed_state.surge = normalize_axis(
        static_cast<std::int16_t>(read_u16_be(bytes + 10U)));

    parsed_state.sway = normalize_axis(
        static_cast<std::int16_t>(read_u16_be(bytes + 12U)));

    parsed_state.heave = normalize_axis(
        static_cast<std::int16_t>(read_u16_be(bytes + 14U)));

    parsed_state.roll = normalize_axis(
        static_cast<std::int16_t>(read_u16_be(bytes + 16U)));

    parsed_state.pitch = normalize_axis(
        static_cast<std::int16_t>(read_u16_be(bytes + 18U)));

    parsed_state.yaw = normalize_axis(
        static_cast<std::int16_t>(read_u16_be(bytes + 20U)));

    parsed_state.gripper_rotate = normalize_axis(
        static_cast<std::int16_t>(read_u16_be(bytes + 22U)));

    parsed_state.gripper_pitch = normalize_axis(
        static_cast<std::int16_t>(read_u16_be(bytes + 24U)));

    parsed_state.gripper_grip = normalize_axis(
        static_cast<std::int16_t>(read_u16_be(bytes + 26U)));

    parsed_event.mode_request =
        static_cast<JoystickModeRequest>(requested_mode);

    constexpr std::uint8_t toggle_killswitch_flag = 1U << 0U;

    parsed_event.toggle_killswitch =
        (flags & toggle_killswitch_flag) != 0U;

    return true;
}

std::uint16_t JoystickInterface::read_u16_be(
    const std::uint8_t* bytes)
{
    return static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(bytes[0]) << 8U) |
        static_cast<std::uint16_t>(bytes[1]));
}

std::uint32_t JoystickInterface::read_u32_be(
    const std::uint8_t* bytes)
{
    return
        (static_cast<std::uint32_t>(bytes[0]) << 24U) |
        (static_cast<std::uint32_t>(bytes[1]) << 16U) |
        (static_cast<std::uint32_t>(bytes[2]) << 8U) |
        static_cast<std::uint32_t>(bytes[3]);
}

double JoystickInterface::normalize_axis(std::int16_t value)
{
    constexpr double maximum = 32767.0;

    return std::clamp(
        static_cast<double>(value) / maximum,
        -1.0,
        1.0);
}

}  // namespace vortex::io
