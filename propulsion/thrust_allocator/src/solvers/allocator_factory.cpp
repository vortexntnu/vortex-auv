#include "vortex/propulsion/thrust_allocator/allocator_factory.hpp"
#include "vortex/propulsion/thrust_allocator/pseudoinverse_allocator.hpp"

#ifdef VORTEX_THRUST_ALLOCATOR_HAS_CASADI
#include "vortex/propulsion/thrust_allocator/qp_allocator.hpp"
#endif

#include <stdexcept>

std::unique_ptr<Allocator> Factory::make_allocator(
    const std::string& allocator_type,
    const AllocatorConfig& config)
{
    if (allocator_type == "pseudoinverse") {
        return std::make_unique<PseudoinverseAllocator>(config);
    }

    if (allocator_type == "qp") {
#ifdef VORTEX_THRUST_ALLOCATOR_HAS_CASADI
        return std::make_unique<QPAllocator>(config);
#else
        throw std::runtime_error(
            "QP allocator requested, but thrust_allocator was built "
            "without CasADi support");
#endif
    }

    throw std::invalid_argument(
        "Unknown allocator type: '" + allocator_type +
        "'. Expected: pseudoinverse or qp.");
}
