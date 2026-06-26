#pragma once

#include <memory>
#include <string>

#include "vortex/propulsion/thrust_allocator/allocator.hpp"
#include "vortex/propulsion/thrust_allocator/allocator_config.hpp"

class Factory {
public:
    static std::unique_ptr<Allocator> make_allocator(
        const std::string& allocator_type,
        const AllocatorConfig& config);
};
