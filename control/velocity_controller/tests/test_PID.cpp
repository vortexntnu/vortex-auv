#include <gtest/gtest.h>
//#include <Eigen/Dense>
//#include <vortex/utils/math.hpp>
#include <velocity_controller/PID_setup.hpp>
//#include <velocity_controller/LQR_setup.hpp>

class PID_test : public ::testing::Test{
    protected:
    double delta=0.0001;
    PID_controller PID;
    void SetUp() override{
        PID.set_parameters(0,0,0);
        PID.reset_controller();
    }
    void TearDown() override{
        
    }
        
};
/*
class Node_test : public ::testing:Test{
    protected:
    static void SetUpTestSuite(){
            int argc=0;
            char ** argv==nullptr;
            rclcpp::init();
        }
    static void TearDownTestSuite(){
        rclcpp::shutdown();
    }

};


*/
TEST_F(PID_test,reset_controller){
    PID.set_parameters(0,1,0);
    PID.calculate_thrust(100,100);
    PID.calculate_thrust(0,1);
    PID.reset_controller();
    SCOPED_TRACE("Scenario: reset");
    EXPECT_NEAR(PID.get_output(),0,delta);
    PID.calculate_thrust(0,1);
    SCOPED_TRACE("Scenario: reset2"); 
    EXPECT_NEAR(PID.get_output(),0,delta);
}
TEST_F(PID_test,P){
    PID.set_parameters(1,0,0);
    EXPECT_NEAR(PID.calculate_thrust(1,1),1,delta);
    EXPECT_NEAR(PID.calculate_thrust(2,1),2,delta);
    PID.set_parameters(1.2,0,0);
    EXPECT_NEAR(PID.calculate_thrust(-2.2,1),-2.64,delta);
    EXPECT_NEAR(PID.calculate_thrust(-1.5,1),-1.8,delta);
}
TEST_F(PID_test,I){
    PID.set_parameters(0,1.1,0);
    PID.calculate_thrust(1,1);
    EXPECT_NEAR(PID.get_output(),0,delta);
    PID.calculate_thrust(1,1);
    EXPECT_NEAR(PID.get_output(),1.1,delta);
    PID.calculate_thrust(-1,1);
    EXPECT_NEAR(PID.get_output(),2.2,delta);
    EXPECT_NEAR(PID.calculate_thrust(0,1),1.1,delta);
    PID.set_output_limits(-101,101);
    PID.reset_controller();
    PID.set_parameters(1,1,0);
    EXPECT_NEAR(PID.calculate_thrust(1000,10),101,delta);
    EXPECT_NEAR(PID.calculate_thrust(0,1),0,delta);
    PID.reset_controller();
    EXPECT_NEAR(PID.calculate_thrust(-10000,1),-101,delta);
    PID.calculate_thrust(-50,1);
    EXPECT_NEAR(PID.calculate_thrust(1,1),-49,delta);
}
TEST_F(PID_test,D){

}
TEST_F(PID_test,illegal_inputs){
    double temp=PID.get_output();
    EXPECT_FALSE(PID.calculate_thrust(1,0));
    EXPECT_NEAR(PID.get_output(),temp,delta);
    EXPECT_FALSE(PID.set_output_limits(1,-1));
    EXPECT_FALSE(PID.calculate_thrust(1,-1));
}
/*
TEST(PID,BASIC){
    PID_controller foo (1,1,0);
    ASSERT_NO_THROW(foo.set_output_limits(-10,100));
    ASSERT_NO_THROW(foo.calculate_thrust(1,1));
    EXPECT_TRUE(foo.get_output()>0);
    EXPECT_NEAR(foo.get_output(),1,0.01);
    ASSERT_NO_THROW(foo.calculate_thrust(1,1));
    EXPECT_NEAR(foo.get_output(),2,0.01);
    ASSERT_NO_THROW(foo.calculate_thrust(1000000,1));
    EXPECT_EQ(foo.get_output(),100);
    ASSERT_NO_THROW(foo.calculate_thrust(-100000,1));
    EXPECT_EQ(foo.get_output(),-10);
}
*/



int main(int argc,char** argv){
    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}