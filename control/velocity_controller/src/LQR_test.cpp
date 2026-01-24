#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include "velocity_controller/LQR_setup.hpp"
#include "ct/optcon/lqr/LQR.hpp" 

class test_LQR_node : public rclcpp::Node{
    public:
    double q_surge =75.0, q_pitch= 175.0, q_yaw= 175.0, r_surge= 0.3, r_pitch= 0.4, r_yaw= 0.4, i_surge= 0.3,
    i_pitch= 0.4, i_yaw= 0.3,  i_weight= 0.5, dt= 0.1;
    LQRparameters param={q_surge, q_pitch, q_yaw, r_surge, r_pitch, r_yaw, i_surge, i_pitch, i_yaw, i_weight};
    LQRController controller;
    test_LQR_node():Node("test_LQR_node"), controller(){
        RCLCPP_INFO(this->get_logger(),"LQR test node started");
        Eigen::Matrix<double,8,8> Q;
        Eigen::VectorXd qdiag(8);
        qdiag << 75, 175, 175, 1, 0.3, 0.4, 0.3, 1;
        Q.diagonal() = qdiag;
        Eigen::Matrix<double,3,3> R;
        Eigen::VectorXd rdiag(3);
        rdiag << 0.3, 0.4, 0.4;
        R.diagonal() = rdiag;
        Eigen::Matrix<double,8,8> A=(Eigen::Matrix<double,8,8>()<<5,7,23,0,0,0,1,1,0,45,21,4,3,4,3,2,0,23,1,7,6,5,5,7,5,7,6,3,5,7,0,9,2,2,3,2,1,0,3,7,0,0,8,7,6,5,8,5,4,7,1,5,6,2,4,7,6,2,4,5,2,12,5,6).finished();
        Eigen::Matrix<double,8,3> B=(Eigen::Matrix<double,8,3>()<<2,0,0,3,0,2,0,3,0,1,2,0,3,4,0,3,5,0,6,3,4,1,2,3).finished();
        /*Eigen::Matrix3d inertia_matrix=(Eigen::Matrix3d()<<30.0, 0.6, 0.0, 0.6, 1.629, 0.0, 0.0, 0.0, 1.729).finished();
        Eigen::Matrix3d inertia_matrix_inv=inertia_matrix.inverse();
        Eigen::Matrix3d coriolis_matrix=(Eigen::Matrix3d()<<0.2,-30*2*0.01,-30*2*0.0,30 * 2*0.01,0,1.629 * 2,30 * 2*0.01,1.769 * 2,0).finished();
        Eigen::Matrix3d system_matrix=inertia_matrix.inverse()*coriolis_matrix;
        Eigen::Matrix<double,6,6> augmented_system_matrix =(Eigen::Matrix<double,6,6>()<<system_matrix(0,0),system_matrix(0,1),system_matrix(0,2),0,0,0,
                               system_matrix(1,0),system_matrix(1,1),system_matrix(1,2),0,0,0,
                               system_matrix(2,0),system_matrix(2,1),system_matrix(2,2),0,0,0,
                               -1,0,0,0,0,0,
                               0,-1,0,0,0,0,
                               0,0,-1,0,0,0).finished();
        Eigen::Matrix<double,6,3> augmented_input_matrix=(Eigen::Matrix<double,6,3>()<< inertia_matrix_inv(0,0),inertia_matrix_inv(0,1),inertia_matrix_inv(0,2),0,0,0,
                              inertia_matrix_inv(1,0),inertia_matrix_inv(1,1),inertia_matrix_inv(1,2),0,0,0,
                              inertia_matrix_inv(2,0),inertia_matrix_inv(2,1),inertia_matrix_inv(2,2),0,0,0).finished();*/
        ct::optcon::LQR<8,3> lqr_solver;
        Eigen::Matrix<double,3,8> K;
        lqr_solver.compute(Q,R,A,B,K,true,false);
        RCLCPP_INFO(this->get_logger(),"LQR Gain K matrix:");
        RCLCPP_INFO(this->get_logger(),"\n%f %f %f %f %f %f %f %f\n%f %f %f %f %f %f %f %f\n%f %f %f %f %f %f %f %f",
        K(0,0),K(0,1),K(0,2),K(0,3),K(0,4),K(0,5),K(0,6),K(0,7),
        K(1,0),K(1,1),K(1,2),K(1,3),K(1,4),K(1,5),K(1,6),K(1,7),
        K(2,0),K(2,1),K(2,2),K(2,3),K(2,4),K(2,5),K(2,6),K(2,7));

    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<test_LQR_node>());
  rclcpp::shutdown();
    return 0;
}
