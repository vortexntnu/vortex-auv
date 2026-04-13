#include <gtest/gtest.h>
#include <stdexcept>

#include "los_guidance/lib/adaptive_los.hpp"
#include "los_guidance/lib/integral_los.hpp"
#include "los_guidance/lib/proportional_los.hpp"
#include "los_guidance/lib/vector_field_los.hpp"

namespace vortex::guidance::los {

TEST(LosInvalidParamsTest, AdaptiveLosRejectsNegativeLookaheadDistance) {
    AdaptiveLosParams params;
    params.lookahead_distance_h = -0.9;
    params.lookahead_distance_v = 1.4;
    params.adaptation_gain_h = 0.03;
    params.adaptation_gain_v = 0.02;
    params.time_step = 0.01;

    EXPECT_THROW(
        { AdaptiveLOSGuidance guidance(params); }, std::invalid_argument);
}

TEST(LosInvalidParamsTest, ProportionalLosRejectsZeroLookaheadDistance) {
    ProportionalLosParams params;
    params.lookahead_distance_h = 0.0;
    params.lookahead_distance_v = 0.8;

    EXPECT_THROW(
        { ProportionalLOSGuidance guidance(params); }, std::invalid_argument);
}

TEST(LosInvalidParamsTest, IntegralLosRejectsZeroTimeStep) {
    IntegralLosParams params;
    params.proportional_gain_h = 0.5;
    params.proportional_gain_v = 0.5;
    params.integral_gain_h = 0.1;
    params.integral_gain_v = 0.1;
    params.time_step = 0.0;

    EXPECT_THROW(
        { IntegralLOSGuidance guidance(params); }, std::invalid_argument);
}

TEST(LosInvalidParamsTest, VectorFieldLosRejectsNegativeApproachAngle) {
    VectorFieldLosParams params;
    params.max_approach_angle_h = -1.0;
    params.max_approach_angle_v = 1.0;
    params.proportional_gain_h = 1.5;
    params.proportional_gain_v = 0.9;
    params.time_step = 0.01;

    EXPECT_THROW(
        { VectorFieldLOSGuidance guidance(params); }, std::invalid_argument);
}

}  // namespace vortex::guidance::los
