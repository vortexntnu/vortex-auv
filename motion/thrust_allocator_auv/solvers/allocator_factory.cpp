#include "thrust_allocator_auv/allocator_factory.hpp"
#include "thrust_allocator_auv/allocator.hpp"
#include "thrust_allocator_auv/pseudoinverse_allocator.hpp"

std::unique_ptr<Allocator> Factory::make_allocator(
    const std::string& allocator_type,
    const AllocatorConfig& config) {
    if (allocator_type == "pseudoinverse") {
        return std::make_unique<PseudoinverseAllocator>(config);
    }

    else if (allocator_type == "qp") {
        return std::make_unique<QPAllocator>(config);
    }

    else {
        throw std::invalid_argument("Unknown allocator_type: '" +
                                    allocator_type +
                                    "'. Expected: pseudoinverse or qp.");
    }
};
