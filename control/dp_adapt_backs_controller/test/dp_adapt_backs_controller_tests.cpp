#include <gtest/gtest.h>

#include "dp_adapt_backs_controller/typedefs.hpp"
#include "dp_adapt_backs_controller/dp_adapt_backs_controller_utils.hpp"
#include "dp_adapt_backs_controller/dp_adapt_backs_controller.hpp"

class DPAdaptBacksControllerTests : public ::testing::Test
{
    protected:
     DPAdaptBacksControllerTests() : dp_adapt_backs_controller_{ get_dp_params() }
     {
     }

     dp_types::DPAdaptParams get_dp_params()
     {
        dp_types::DPAdaptParams params;
        params.adap_param = dp_types::Vector12d::Ones() * 0.1;
        params.d_gain = dp_types::Vector6d::Ones() * 0.6;
        params.K1 = dp_types::Vector6d::Ones() * 5.0;
        params.K2 = dp_types::Vector6d::Ones() * 5.0;
        params.r_b_bg = dp_types::Vector3d(0.01, 0.0, 0.02);
        params.I_b = dp_types::Vector3d(0.68, 3.32, 3.34);
        params.mass_matrix = dp_types::Matrix6d::Identity() * 30.0;
        params.m = 30.0;
        return params;
     }

     dp_types::Eta generate_current_pose(const double north_pos,
                                         const double east_pos,
                                         const double down_pos,
                                         const double roll_angle,
                                         const double pitch_angle,
                                         const double yaw_angle)
     {
        dp_types::Eta current_pose;
        current_pose.pos = dp_types::Vector3d(north_pos, east_pos, down_pos);
        current_pose.ori = dp_types::Vector3d(roll_angle, pitch_angle, yaw_angle);
        return current_pose;
     }

     dp_types::Eta generate_reference_pose(const double north_pos,
                                           const double east_pos,
                                           const double down_pos,
                                           const double roll_angle,
                                           const double pitch_angle,
                                           const double yaw_angle)
     {
        dp_types::Eta reference_pose;
        reference_pose.pos = dp_types::Vector3d(north_pos, east_pos, down_pos);
        reference_pose.ori = dp_types::Vector3d(roll_angle, pitch_angle, yaw_angle);
        return reference_pose;
     }

     dp_types::Nu generate_current_velocity(const double surge_vel,
                                            const double sway_vel,
                                            const double heave_vel,
                                            const double roll_rate,
                                            const double pitch_rate,
                                            const double yaw_rate)
     {
        dp_types::Nu current_velocity;
        current_velocity.linear_speed = dp_types::Vector3d(surge_vel, sway_vel, heave_vel);
        current_velocity.angular_speed = dp_types::Vector3d(roll_rate, pitch_rate, yaw_rate);
        return current_velocity;
     }

     DPAdaptBacksController dp_adapt_backs_controller_;
};

TEST_F(DPAdaptBacksControllerTests, T01_north_error_with_zero_heading_gives_surge_only_command)
{
    EXPECT_EQ(1, 2);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}