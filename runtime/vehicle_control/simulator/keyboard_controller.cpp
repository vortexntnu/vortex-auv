#include "simulator/keyboard_controller.hpp"

#include <algorithm>
#include <iostream>
#include <sstream>
#include <string>

#include <Eigen/Geometry>

#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace vortex::runtime::vehicle_control {

namespace {

constexpr double force_step_n = 5.0;
constexpr double torque_step_nm = 1.0;

constexpr double maximum_force_n = 50.0;
constexpr double maximum_torque_nm = 10.0;

class TerminalRawMode {
public:
    TerminalRawMode()
    {
        tcgetattr(STDIN_FILENO, &original_);

        termios raw = original_;

        raw.c_lflag &=
            static_cast<tcflag_t>(~(ICANON | ECHO));

        tcsetattr(
            STDIN_FILENO,
            TCSANOW,
            &raw);
    }

    ~TerminalRawMode()
    {
        tcsetattr(
            STDIN_FILENO,
            TCSANOW,
            &original_);
    }

private:
    termios original_{};
};

double clamp_force(double value)
{
    return std::clamp(
        value,
        -maximum_force_n,
        maximum_force_n);
}

double clamp_torque(double value)
{
    return std::clamp(
        value,
        -maximum_torque_nm,
        maximum_torque_nm);
}

}  // namespace

KeyboardController::KeyboardController() = default;

KeyboardController::~KeyboardController()
{
    stop();
}

void KeyboardController::start()
{
    if (running_.exchange(true)) {
        return;
    }

    thread_ =
        std::thread{
            &KeyboardController::run,
            this,
        };
}

void KeyboardController::stop()
{
    running_.store(false);

    if (thread_.joinable()) {
        thread_.join();
    }
}

SimulatorInput KeyboardController::input() const
{
    std::scoped_lock lock{mutex_};
    return input_;
}

void KeyboardController::run()
{
    TerminalRawMode terminal_raw_mode;

    std::cout
        << "\nManual simulator controls:\n"
        << "  W/S: surge forward/backward\n"
        << "  A/D: sway left/right\n"
        << "  R/F: heave up/down\n"
        << "  Q/E: yaw left/right\n"
        << "  X: stop all movement\n"
        << "  Space: toggle manual command\n"
        << "  M: toggle manual/reference mode\n"
        << "  G: type reference x y z roll pitch yaw (radians)\n"
        << "  Esc: stop keyboard controller\n\n";

    while (running_.load()) {
        pollfd descriptor{
            .fd = STDIN_FILENO,
            .events = POLLIN,
            .revents = 0,
        };

        const int result =
            poll(&descriptor, 1, 50);

        if (result <= 0) {
            continue;
        }

        char key = '\0';

        if (read(STDIN_FILENO, &key, 1) == 1) {
            process_key(key);
        }
    }
}

void KeyboardController::process_key(char key)
{
    if ((key == 'g' || key == 'G')) {
        read_reference();
        return;
    }

    std::scoped_lock lock{mutex_};

    ManualCommand& command = input_.manual_command;

    switch (key) {
    case 'w':
    case 'W':
        command.wrench[0] =
            clamp_force(
                command.wrench[0] +
                force_step_n);
        break;

    case 's':
    case 'S':
        command.wrench[0] =
            clamp_force(
                command.wrench[0] -
                force_step_n);
        break;

    case 'a':
    case 'A':
        command.wrench[1] =
            clamp_force(
                command.wrench[1] +
                force_step_n);
        break;

    case 'd':
    case 'D':
        command.wrench[1] =
            clamp_force(
                command.wrench[1] -
                force_step_n);
        break;

    case 'r':
    case 'R':
        command.wrench[2] =
            clamp_force(
                command.wrench[2] -
                force_step_n);
        break;

    case 'f':
    case 'F':
        command.wrench[2] =
            clamp_force(
                command.wrench[2] +
                force_step_n);
        break;

    case 'q':
    case 'Q':
        command.wrench[5] =
            clamp_torque(
                command.wrench[5] +
                torque_step_nm);
        break;

    case 'e':
    case 'E':
        command.wrench[5] =
            clamp_torque(
                command.wrench[5] -
                torque_step_nm);
        break;

    case 'x':
    case 'X':
        command.wrench.setZero();
        break;

    case ' ':
        command.active = !command.active;

        if (!command.active) {
            command.wrench.setZero();
        }

        break;

    case 'm':
    case 'M':
        input_.operation_mode =
            input_.operation_mode == vortex::utils::types::Mode::manual
                ? vortex::utils::types::Mode::autonomous
                : vortex::utils::types::Mode::manual;
        command.active = false;
        command.wrench.setZero();
        break;

    case 27:
        command.active = false;
        command.wrench.setZero();
        running_.store(false);
        break;

    default:
        break;
    }

    std::cout
        << "\rMode: "
        << (input_.operation_mode == vortex::utils::types::Mode::manual
                ? "MANUAL   " : "REFERENCE")
        << " manual: "
        << (command.active ? "ON " : "OFF")
        << " wrench = ["
        << command.wrench.transpose()
        << "]          "
        << std::flush;
}

void KeyboardController::read_reference()
{
    std::cout << "\nReference x y z roll pitch yaw [rad]: " << std::flush;

    std::string line;
    while (running_.load()) {
        pollfd descriptor{
            .fd = STDIN_FILENO,
            .events = POLLIN,
            .revents = 0,
        };
        if (poll(&descriptor, 1, 50) <= 0) {
            continue;
        }

        char character = '\0';
        if (read(STDIN_FILENO, &character, 1) != 1) {
            continue;
        }
        if (character == '\n' || character == '\r') {
            break;
        }
        if ((character == 127 || character == '\b') && !line.empty()) {
            line.pop_back();
            std::cout << "\b \b" << std::flush;
        } else if (character >= 32 && character < 127) {
            line.push_back(character);
            std::cout << character << std::flush;
        }
    }

    double x, y, z, roll, pitch, yaw;
    std::istringstream values{line};
    if (!(values >> x >> y >> z >> roll >> pitch >> yaw)) {
        std::cout << "\nInvalid reference; expected six numbers.\n";
        return;
    }

    const Eigen::Quaterniond orientation =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    std::scoped_lock lock{mutex_};
    input_.reference.x = x;
    input_.reference.y = y;
    input_.reference.z = z;
    input_.reference.qw = orientation.w();
    input_.reference.qx = orientation.x();
    input_.reference.qy = orientation.y();
    input_.reference.qz = orientation.z();
    ++input_.reference_revision;
    input_.operation_mode = vortex::utils::types::Mode::autonomous;
    input_.manual_command.active = false;
    input_.manual_command.wrench.setZero();

    std::cout << "\nReference accepted; mode set to REFERENCE.\n";
}

}  // namespace vortex::runtime::vehicle_control
