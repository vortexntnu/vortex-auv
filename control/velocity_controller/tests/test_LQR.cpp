#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <vortex/utils/math.hpp>
#include <velocity_controller/LQR_setup.hpp>
#include "velocity_controller/utilities.hpp"
#include <yaml-cpp/yaml.h>

class LQR_test : public ::testing::Test{
    protected:
    double delta=0.0001;
    static inline YAML::Node cfg;
    LQRController controller;
    static void SetUpTestSuite() {  
        try {
            cfg = YAML::LoadFile(YAML_PATH);
        } 
        catch (const YAML::Exception& e) {
            FAIL() << "Failed to load YAML from '" << YAML_PATH << "': " << e.what();
        }
        
    };
    void SetUp() override{
        controller.set_matrices(cfg["/**"]["ros__parameters"]["LQR_params"]["Q"].as<std::vector<double>>(),cfg["/**"]["ros__parameters"]["LQR_params"]["R"].as<std::vector<double>>(),cfg["/**"]["ros__parameters"]["inertia_matrix"].as<std::vector<double>>(),cfg["/**"]["ros__parameters"]["max_force"].as<double>(),cfg["/**"]["ros__parameters"]["dampening_matrix_low"].as<std::vector<double>>(),cfg["/**"]["ros__parameters"]["dampening_matrix_high"].as<std::vector<double>>());
        controller.reset_controller();
        controller.set_interval(0.01);
    }
    void TearDown() override{
        
    }
};
/*
TEST(LQR,setup){
    LQRController controller;
    controller.set_interval(1);
    YAML::Node cfg;
    ASSERT_NO_THROW(cfg=YAML::LoadFile(YAML_PATH));
    controller.set_matrices(cfg["LQR"]["Q"],cfg["LQR"]["Q"],cfg["LQR"]["Q"],cfg["LQR"]["Q"],100);
};
*/
TEST_F(LQR_test,wrong_setup){
    //LQRController controller;
    std::vector<double> eight={1,2,3,4,5,6,7,8};
    std::vector<double> six={1,2,3,4,5,6};
    std::vector<double> thirty_six={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36};
    std::vector<double> three={1,2,3};
    EXPECT_TRUE(controller.set_matrices(eight,three,thirty_six,100,thirty_six,thirty_six));
    EXPECT_FALSE(controller.set_matrices(eight,eight,thirty_six,100,thirty_six,thirty_six));
    EXPECT_FALSE(controller.set_matrices(three,three,thirty_six,100,thirty_six,thirty_six));
    EXPECT_FALSE(controller.set_matrices(eight,three,eight,100,thirty_six,thirty_six));
    EXPECT_FALSE(controller.set_matrices(eight,three,thirty_six,100,eight,thirty_six));
    EXPECT_FALSE(controller.set_matrices(eight,three,thirty_six,100,thirty_six,eight));
    EXPECT_FALSE(controller.set_matrices(eight,three,thirty_six,-100,thirty_six,thirty_six));
};
/*
TEST_F(LQR_test,solve){
    State states{1,1,1,2,2,2,1,2,1};
    Guidance_data value{1,3,2};
    Eigen::Vector <double,3> result=controller.calculate_thrust(states,value);
    EXPECT_NEAR(result(0),0,delta);
    EXPECT_NEAR(result(1),0,delta);
    EXPECT_NEAR(result(2),0,delta);
};*/
TEST_F(LQR_test,Direction){
    Guidance_data value;
    State state{};
    value.surge=0.2;
    Eigen::Vector<double, 3> result=controller.calculate_thrust(state,value);
    EXPECT_TRUE(result(0)>0);
    
}
TEST_F(LQR_test,zero_input){
    State states{};
    states.surge=1;
    states.yaw=0.2;
    states.pitch=0.3;
    Guidance_data value;
    value.surge=1.0;
    value.yaw=0.2;
    value.pitch=0.3;
    Eigen::Vector <double,3> result=controller.calculate_thrust(states,value);
    EXPECT_NEAR(result(0),0,delta);
    EXPECT_NEAR(result(1),0,delta);
    EXPECT_NEAR(result(2),0,delta);
    controller.reset_controller();
    states.surge=0;
    states.pitch=0;
    states.yaw=0;
    value.surge=0;
    value.pitch=0;
    value.yaw=0;
    result=controller.calculate_thrust(states, value);
    EXPECT_NEAR(result(0),0,delta);
    EXPECT_NEAR(result(1),0,delta);
    EXPECT_NEAR(result(2),0,delta);
}

int main(int argc,char** argv){
    testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
