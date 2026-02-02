#ifndef LANDMARK_SERVER__TYPEDEFS_HPP_
#define LANDMARK_SERVER__TYPEDEFS_HPP_

#include <vortex/utils/types.hpp>

namespace vortex::mission {

struct Landmark {
    uint16_t type{};
    uint16_t subtype{};
    vortex::utils::types::Pose pose{};
};

}  // namespace vortex::mission

#endif  // LANDMARK_SERVER__TYPEDEFS_HPP_
