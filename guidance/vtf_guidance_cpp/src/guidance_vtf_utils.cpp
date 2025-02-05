#include "guidance_vtf/guidance_vtf_utils.hpp"

Matrix3d R_z(double rotation) {
    Matrix3d R;
    R << std::cos(rotation), -std::sin(rotation), 0, std::sin(rotation),
        std::cos(rotation), 0, 0, 0, 1;
    return R;
};

double ssa(double angle) {
    double angle_ssa = fmod(angle + M_PI, 2 * M_PI) - M_PI;
    return angle_ssa;
};
