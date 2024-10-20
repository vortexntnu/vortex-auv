#ifndef REFERENCE_FILTER_HPP
#define REFERENCE_FILTER_HPP

#include "reference_filter_dp/eigen_typedefs.hpp"

class ReferenceFilter {
    public:
        explicit ReferenceFilter();

        Vector18d calculate_x_dot(const Vector18d &x, const Vector6d &r);

        void calculate_Ad();

        void calculate_Bd();

        void set_omega(const Vector6d &omega);

        void set_delta(const Vector6d &zeta);

    private:
        Matrix18d Ad_;
        Matrix18x6d Bd_;
        Matrix6d Omega_;
        Matrix6d Delta_;
        Matrix6d identity_matrix_;
};

#endif