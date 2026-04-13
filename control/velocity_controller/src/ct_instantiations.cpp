// src/ct_instantiations.cpp
// This file exists ONLY to emit Control Toolbox symbols

//#include <Eigen/Dense>
#include <ct/optcon/optcon.h>
//#define CT_OPTCON_ENABLE_LQR
//#define CT_CORE_ENABLE_CARE

//#include <ct/optcon/lqr/LQR.hpp>
//#include <ct/optcon/lqr/LQR-impl.hpp>

//#include <ct/optcon/lqr/riccati/CARE.hpp>
//#include <ct/optcon/lqr/riccati/CARE-impl.hpp>

template class ct::optcon::LQR<8, 3>; 
template class ct::optcon::CARE<8, 3>;