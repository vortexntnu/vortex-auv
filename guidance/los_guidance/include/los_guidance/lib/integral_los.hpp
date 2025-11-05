#ifndef INTEGRAL_LOS_GUIDANCE_HPP
#define INTEGRAL_LOS_GUIDANCE_HPP

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "los_guidance/lib/types.hpp"
#include <cmath>

namespace vortex::guidance::los {

    struct IntergalLosParams {
        double lookahead_distance_h{};
        double lookahead_distance_v{};
        double gamma_h{};
        double gamma_v{};
        double time_step{};
    };

    class IntergalLOSGuidance {
        public:
            IntergalLOSGuidance(const IntergalLosParams& params);
            ~IntergalLOSGuidance() = default;  
        
        private:
            void update_angles(const types::Inputs& inputs);
            types::CrossTrackError calculate_crosstrack_error(const tyoes::Inputs& inputs);

            IntergalLosParams m_params{};
        };
}
 
#endif  // INTEGRAL_LOS_GUIDANCE_HPP