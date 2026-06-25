/**
 * @file allocator_factory.hpp
 * @brief Contains the Allocator Factory, which makes it easier
 * to choose allocator at runtime i.e imported through ROS params.
 */

#ifndef THRUST_ALLOCATOR_AUV__ALLOCATOR_FACTORY_HPP_
#define THRUST_ALLOCATOR_AUV__ALLOCATOR_FACTORY_HPP_

#include <memory>
#include <string>
#include "thrust_allocator_auv/allocator_config.hpp"
#include "thrust_allocator_auv/qp_allocator.hpp"

/**
 * @brief The Allocator Factory that will generate a specific Allocator type.
 */
class Factory {
   public:
    /**
     * @brief A function that makes an allocator based on a string
     * defining the desired type.
     *
     * @param allocator_type String of the desired allocator type.
     * @return unique pointer to the created allocator.
     */
    static std::unique_ptr<Allocator> make_allocator(
        const std::string& allocator_type,
        const AllocatorConfig& config);
};

#endif  // THRUST_ALLOCATOR_AUV__ALLOCATOR_FACTORY_HPP_
