#ifndef VELOCITY_CONTROLLER__CONTROLLER_HPP_
#define VELOCITY_CONTROLLER__CONTROLLER_HPP_
#include <memory>
#include "velocity_controller/utilities.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "vortex/utils/math.hpp"

//TODO(henrimha): implement a saturate tau function/anti wind up, that uses the normalize_wrench_vector and compute max wrench, maybe some other
//TODO(henrimha): change the way anti wind up works, seperate the calculation and put up flag for saturation, then anti wind up calculation

struct controller_params{
    int num_dimensions;
    int num_thrusters;
    Eigen::MatrixXd thruster_position;
    Eigen::MatrixXd thruster_force_direction;
    Eigen::Vector3d center_of_mass;
    double min_thrust;
    double max_thrust;
    
};
class controller{
    public:
        controller(const controller_params& params);
        virtual geometry_msgs::msg::WrenchStamped calculate_thrust(State state, State error_state) = 0;
        virtual void reset_controller(int nr=0) = 0;
        bool get_validity(){return valid;};
        ~controller()=default;
        geometry_msgs::msg::WrenchStamped saturate_thrust_direction(const geometry_msgs::msg::WrenchStamped& thrust_wrench);
        geometry_msgs::msg::WrenchStamped saturate_thrust_block(const geometry_msgs::msg::WrenchStamped& thrust_wrench);

    protected:
        bool valid=false;
        double saturated[6];

        controller_params params_;
        Eigen::MatrixXd thrust_configuration_;
        Eigen::VectorXd min_force_vec;
        Eigen::VectorXd max_force_vec;
        Eigen::Vector<double,6> tau_max;

};

#endif // VELOCITY_CONTROLLER__CONTROLLER_HPP_