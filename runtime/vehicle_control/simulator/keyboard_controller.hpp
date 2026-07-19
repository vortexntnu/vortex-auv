#pragma once

#include "common/manual_command.hpp"
#include "vortex/utils/types.hpp"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <thread>

namespace vortex::runtime::vehicle_control {

struct SimulatorInput {
    ManualCommand manual_command{};
    vortex::utils::types::Mode operation_mode =
        vortex::utils::types::Mode::manual;
    vortex::utils::types::Pose reference{};
    std::uint64_t reference_revision = 0;
};

class KeyboardController {
public:
    KeyboardController();
    ~KeyboardController();

    KeyboardController(const KeyboardController&) = delete;
    KeyboardController& operator=(const KeyboardController&) = delete;

    void start();
    void stop();

    [[nodiscard]]
    SimulatorInput input() const;

private:
    void run();
    void process_key(char key);
    void read_reference();

    mutable std::mutex mutex_;
    SimulatorInput input_;

    std::atomic_bool running_{false};
    std::thread thread_;
};

}  // namespace vortex::runtime::vehicle_control
