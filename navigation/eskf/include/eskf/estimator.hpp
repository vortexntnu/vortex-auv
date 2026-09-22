#pragma once
#include <string>
#include "eskf/typedefs.hpp"

// Shared measurement/output boundary; implementations own their inference
// state.
class NavigationEstimator {
   public:
    virtual ~NavigationEstimator() = default;
    virtual bool imu_update(const ImuMeasurement&, double dt) = 0;
    virtual bool dvl_update(const SensorDVL&) = 0;
    virtual bool depth_update(const SensorDepth&) = 0;
    virtual NominalState get_nominal_state() const = 0;
    virtual ErrorState get_error_state() const = 0;
    virtual double get_nis_dvl() const = 0;
    virtual double get_nis_depth() const = 0;
    virtual bool healthy() const { return true; }
    virtual std::string error_message() const { return {}; }
};
