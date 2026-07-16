#pragma once

#include "common/manual_command.hpp"

#include <atomic>
#include <mutex>
#include <thread>

namespace vortex::runtime::vehicle_control {

class KeyboardController {
public:
    KeyboardController();
    ~KeyboardController();

    KeyboardController(const KeyboardController&) = delete;
    KeyboardController& operator=(const KeyboardController&) = delete;

    void start();
    void stop();

    [[nodiscard]]
    ManualCommand command() const;

private:
    void run();
    void process_key(char key);

    mutable std::mutex mutex_;
    ManualCommand command_;

    std::atomic_bool running_{false};
    std::thread thread_;
};

}  // namespace vortex::runtime::vehicle_control
