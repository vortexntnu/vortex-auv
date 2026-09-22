#pragma once
#include <memory>
#include <optional>
#include <string>
#include "eskf/estimator.hpp"

struct GtsamNavigationParams {
    double lag_seconds = 2.0;
    double keyframe_interval = 0.2;
    double integration_covariance = 1e-8;
};
class GtsamNavigation : public NavigationEstimator {
   public:
    explicit GtsamNavigation(const EskfParams& params,
                             const GtsamNavigationParams& smoothing = {},
                             const NominalState& initial = {});
    ~GtsamNavigation() override;
    bool imu_update(const ImuMeasurement&, double dt) override;
    bool dvl_update(const SensorDVL&) override;
    bool depth_update(const SensorDepth&) override;
    NominalState get_nominal_state() const override;
    ErrorState get_error_state() const override;
    double get_nis_dvl() const override;
    double get_nis_depth() const override;
    bool healthy() const override;
    std::string error_message() const override;
    std::optional<NominalState> keyframe_state(size_t index) const;
    size_t keyframe_count() const;
    size_t active_state_count() const;

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
