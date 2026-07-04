#include "velocity_controller/lib/controller.hpp"
#include <algorithm>
#include "vortex/utils/math.hpp"
#include "vortex/utils/types.hpp"
#include "thrust_allocator_auv/thrust_allocator_utils.hpp"
#include "velocity_controller/utilities.hpp"



geometry_msgs::msg::WrenchStamped controller::saturate_thrust_direction(const geometry_msgs::msg::WrenchStamped& thrust_wrench){
    Eigen::Vector<double,6> tau= wrench_to_vector(thrust_wrench);    
    Eigen::Vector<double,6> saturated_tau=normalize_wrench_vector(tau, tau_max);
    geometry_msgs::msg::WrenchStamped saturated_wrench=vector_to_wrench(saturated_tau);
    for (int i=0; i<6; i++){
        if(saturated_tau[i]!=tau[i]){
            saturated[i] = true;
        }
        else {
            saturated[i] = false;
        }
    }
    return saturated_wrench;

}
geometry_msgs::msg::WrenchStamped controller::saturate_thrust_block(const geometry_msgs::msg::WrenchStamped& thrust_wrench){
    Eigen::Vector<double,6> tau= wrench_to_vector(thrust_wrench);
    Eigen::Vector<double,6> saturated_tau;
    for (int i = 0; i < 6; i++) {
        saturated_tau[i] = std::clamp(tau[i], -tau_max[i], tau_max[i]);
        if(saturated_tau[i] != tau[i]){
            saturated[i] = true;
        }
        else {
            saturated[i] = false;
        }
    }
    geometry_msgs::msg::WrenchStamped saturated_wrench= vector_to_wrench(saturated_tau);
    return saturated_wrench;
}


controller::controller(const controller_params& params):params_(params){
    thrust_configuration_ = vortex::utils::math::build_thrust_configuration_matrix(
        params_.thruster_force_direction, params_.thruster_position, params_.center_of_mass);
    min_force_vec = Eigen::VectorXd::Constant(params_.num_thrusters, params_.min_thrust);
    max_force_vec = Eigen::VectorXd::Constant(params_.num_thrusters, params_.max_thrust);
    tau_max = vortex::utils::math::calculate_valid_thrust_region_polyhedron(
        thrust_configuration_, min_force_vec, max_force_vec);
}

//TODO(henrimha): consider using checking not wether the value is equal but very tiny instead abs<1e-6 