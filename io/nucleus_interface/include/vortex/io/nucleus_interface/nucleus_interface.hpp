#pragma once

#include <mutex>
#include <optional>
#include <thread>

#include <asio.hpp>

#include <nortek_nucleus_driver.hpp>
#include <vortex/utils/types.hpp>

namespace vortex::io {

struct NucleusInterfaceConfig {
    std::string remote_ip;
    std::uint16_t data_remote_port{};
    std::string password;

    bool enable_imu{true};
    bool enable_dvl{true};
    bool enable_altimeter{true};
    bool enable_pressure{true};

    int imu_frequency_hz{100};
    int ahrs_frequency_hz{100};

    AhrsMode ahrs_mode{};
    BottomTrackMode bottom_track_mode{};

    int bottom_track_velocity_range{};
    bool enable_watertrack{false};
    int altimeter_power_level{};

    double rotxy{};
    double rotyz{};
    double rotxz{};
};

struct DvlVelocity {
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d variance{Eigen::Vector3d::Zero()};

    std::chrono::steady_clock::time_point timestamp{};
    bool valid{false};
};

struct NucleusState {
    vortex::utils::types::Pose pose{};
    vortex::utils::types::Twist twist{};

    double altitude_m{};
    double pressure_pa{};

    std::chrono::steady_clock::time_point timestamp{};
    bool valid{false};
};

class NucleusInterface {
   public:
    explicit NucleusInterface(const NucleusInterfaceConfig& config);
    ~NucleusInterface();

    bool start();
    void stop();

    [[nodiscard]]
    std::optional<NucleusState> latest_state() const;

   private:
    void nucleus_callback(NortekNucleusFrame frame);

    void handle_ahrs(const AhrsDataV2& data);
    void handle_ins(const InsDataV2& data);
    void handle_bottom_track(const BottomTrackData& data);
    void handle_altimeter(const AltimeterData& data);

    NucleusInterfaceConfig config_;

    asio::io_context io_;
    std::unique_ptr<NortekNucleusDriver> driver_;
    std::jthread io_thread_;

    mutable std::mutex state_mutex_;
    NucleusState latest_state_{};
    DvlVelocity latest_dvl_{};
    // Cached because the INS packet and AHRS packet arrive separately.

    double latest_qw_{1.0};
    double latest_qx_{0.0};
    double latest_qy_{0.0};
    double latest_qz_{0.0};

    bool have_orientation_{false};
};

}  // namespace vortex::io
