#include <gtest/gtest.h>

#include "reference_filter_dp/eigen_typedefs.hpp"
#include "reference_filter_dp/reference_filter.hpp"

namespace vortex::guidance {

class ReferenceFilterTests : public ::testing::Test {
   protected:
    ReferenceFilterTests() : reference_filter_{get_filter_params()} {}

    ReferenceFilterParams get_filter_params() {
        ReferenceFilterParams params;
        params.omega = Eigen::Vector6d::Ones();
        params.zeta = Eigen::Vector6d::Ones();
        return params;
    }

    ReferenceFilter reference_filter_;
};

TEST_F(ReferenceFilterTests, T01_positive_command_positive_xdot) {
    Eigen::Vector18d x = Eigen::Vector18d::Zero();
    Eigen::Vector6d r = Eigen::Vector6d::Ones();
    Eigen::Vector18d xdot = reference_filter_.calculate_x_dot(x, r);

    for (int i = 0; i < xdot.size(); ++i) {
        EXPECT_GE(xdot(i), 0.0);
    }
}

TEST_F(ReferenceFilterTests, T02_negative_command_negative_xdot) {
    Eigen::Vector18d x = Eigen::Vector18d::Zero();
    Eigen::Vector6d r = -Eigen::Vector6d::Ones();
    Eigen::Vector18d xdot = reference_filter_.calculate_x_dot(x, r);

    for (int i = 0; i < xdot.size(); ++i) {
        EXPECT_LE(xdot(i), 0.0);
    }
}

TEST_F(ReferenceFilterTests, T03_zero_command_zero_xdot) {
    Eigen::Vector18d x = Eigen::Vector18d::Zero();
    Eigen::Vector6d r = Eigen::Vector6d::Zero();
    Eigen::Vector18d xdot = reference_filter_.calculate_x_dot(x, r);

    for (int i = 0; i < xdot.size(); ++i) {
        EXPECT_DOUBLE_EQ(xdot(i), 0.0);
    }
}

}  // namespace vortex::guidance

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
