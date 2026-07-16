#include "simulator/keyboard_controller.hpp"

#include <algorithm>
#include <iostream>

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

ManualCommand KeyboardController::command() const
{
    std::scoped_lock lock{mutex_};
    return command_;
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
    std::scoped_lock lock{mutex_};

    switch (key) {
    case 'w':
    case 'W':
        command_.wrench[0] =
            clamp_force(
                command_.wrench[0] +
                force_step_n);
        break;

    case 's':
    case 'S':
        command_.wrench[0] =
            clamp_force(
                command_.wrench[0] -
                force_step_n);
        break;

    case 'a':
    case 'A':
        command_.wrench[1] =
            clamp_force(
                command_.wrench[1] +
                force_step_n);
        break;

    case 'd':
    case 'D':
        command_.wrench[1] =
            clamp_force(
                command_.wrench[1] -
                force_step_n);
        break;

    case 'r':
    case 'R':
        command_.wrench[2] =
            clamp_force(
                command_.wrench[2] -
                force_step_n);
        break;

    case 'f':
    case 'F':
        command_.wrench[2] =
            clamp_force(
                command_.wrench[2] +
                force_step_n);
        break;

    case 'q':
    case 'Q':
        command_.wrench[5] =
            clamp_torque(
                command_.wrench[5] +
                torque_step_nm);
        break;

    case 'e':
    case 'E':
        command_.wrench[5] =
            clamp_torque(
                command_.wrench[5] -
                torque_step_nm);
        break;

    case 'x':
    case 'X':
        command_.wrench.setZero();
        break;

    case ' ':
        command_.active =
            !command_.active;

        if (!command_.active) {
            command_.wrench.setZero();
        }

        break;

    case 27:
        command_.active = false;
        command_.wrench.setZero();
        running_.store(false);
        break;

    default:
        break;
    }

    std::cout
        << "\rManual: "
        << (command_.active ? "ON " : "OFF")
        << " wrench = ["
        << command_.wrench.transpose()
        << "]          "
        << std::flush;
}

}  // namespace vortex::runtime::vehicle_control
