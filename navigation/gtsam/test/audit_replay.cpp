// Offline audit boundary: accepts ONLY timestamped IMU and DVL measurements.
// Truth and simulation trajectory parameters never enter this process.
#include "gtsam_navigation/estimator.hpp"

#include <iomanip>
#include <iostream>

int main() {
    gtsam_navigation::Config config;
    config.imu_p_dvl =
        gtsam::Pose3(gtsam::Rot3::Rz(3.14159), {-0.030, 0.004, 0.1564});
    gtsam_navigation::Estimator estimator(config);
    double time;
    gtsam::Vector3 acceleration, gyro, velocity;
    int has_dvl, sample = 0;
    std::cout << std::setprecision(17);
    while (std::cin >> time >> acceleration.x() >> acceleration.y() >>
           acceleration.z() >> gyro.x() >> gyro.y() >> gyro.z() >> has_dvl >>
           velocity.x() >> velocity.y() >> velocity.z()) {
        if (has_dvl) {
            estimator.add_dvl({time, velocity, gtsam::I_3x3 * 2.5e-5});
        }
        if (!estimator.add_imu({time, acceleration, gyro})) {
            std::cerr << estimator.status().detail << '\n';
            return 1;
        }
        if (sample++ % 20 != 0)
            continue;
        const auto result = estimator.latest();
        if (!result)
            continue;
        const auto q = result->pose.rotation().toQuaternion();
        const auto covariance = odometry_covariances(*result, config).first;
        std::cout << result->time << ' '
                  << result->pose.translation().transpose() << ' ' << q.x()
                  << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << ' '
                  << result->velocity.transpose();
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                std::cout << ' ' << covariance(row, col);
            }
        }
        std::cout << ' ' << estimator.status().last_dvl_time << ' '
                  << estimator.status().rejected_dvl << '\n';
    }
    return estimator.status().initialized ? 0 : 2;
}
