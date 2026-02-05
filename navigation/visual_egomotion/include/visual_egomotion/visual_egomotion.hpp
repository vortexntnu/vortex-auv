#pragma once

#include <array>
#include <deque>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <rclcpp/time.hpp>

namespace visual_egomotion {

/**
 * Sliding-window smoother for the relative pose T_ref<-base measured from a
 * visual/marker.
 *
 * Notes:
 * - Keeps a time/size bounded buffer.
 * - Uses robust iterative averaging on SO(3) (log/exp maps) + arithmetic mean
 * in R^3.
 * - Outputs a diagonal 6x6 covariance (x,y,z, roll,pitch,yaw) as a cheap
 * confidence proxy.
 *
 * This class has no ROS publishers/subscribers and is unit-testable.
 */
class SlidingWindowSO3Mean {
   public:
    struct Config {
        int win_size = 20;
        double win_age_sec = 1.0;
        double huber_threshold_deg = 2.5;
        double gate_threshold_deg = 15.0;
    };

    struct Sample {
        rclcpp::Time stamp;
        tf2::Transform T_ref_base;
        double weight = 1.0;
    };

    explicit SlidingWindowSO3Mean(Config cfg);

    void clear();
    bool empty() const { return buf_.empty(); }
    size_t size() const { return buf_.size(); }

    void addSample(const rclcpp::Time& stamp,
                   const tf2::Transform& T_ref_base,
                   double weight = 1.0);
    void prune(const rclcpp::Time& now);

    bool estimate(const rclcpp::Time& now,
                  tf2::Transform& T_est,
                  std::array<double, 36>& cov6x6) const;

   private:
    static tf2::Vector3 logMapSO3(const tf2::Quaternion& q);
    static tf2::Quaternion expMapSO3(const tf2::Vector3& w);
    static double angularDist(const tf2::Quaternion& q1,
                              const tf2::Quaternion& q2);

    template <typename T>
    static T median(std::vector<T> v);

    Config cfg_;
    std::deque<Sample> buf_;
};

}  // namespace visual_egomotion