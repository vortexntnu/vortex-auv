#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <vortex/utils/math.hpp>
#include <velocity_controller/PID_setup.hpp>
#include <velocity_controller/LQR_setup.hpp>


TEST(PID,BASIC){
    PID_controller foo (1,1,0);
    ASSERT_NO_THROW(foo.set_output_limits(-10,100));
    ASSERT_NO_THROW(foo.calculate_thrust(1,1));
    EXPECT_TRUE(foo.output()>0);
    EXPECT_NEAR(foo.output(),1,0.01);
    ASSERT_NO_THROW(foo.calculate_thrust(1,1));
    EXPECT_NEAR(foo.output(),2,0.01);
    ASSERT_NO_THROW(foo.calculate_thrust(1000000,1));
    EXPECT_EQ(foo.output(),100);
    ASSERT_NO_THROW(foo.calculate_thrust(-100000,1));
    EXPECT_EQ(foo.output(),-10);
}




int main(int argc,char** argv){
    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}