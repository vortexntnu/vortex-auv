#include "thrust_allocator_auv/qp_allocator.hpp"
#include "thrust_allocator_auv/allocator_config.hpp"

QPAllocator::QPAllocator(const AllocatorConfig& cfg) {
    formulate_as_qp(cfg);
};

// n = 6, r = 8
void QPAllocator::formulate_as_qp(const AllocatorConfig& cfg){
    // TODO: Fossen page 324-326
};

Eigen::VectorXd QPAllocator::calculate_allocated_thrust(const Eigen::VectorXd& tau) {
    // TODO: look into static implementations perhaps lookup table of precomputed vals
    return tau;
};