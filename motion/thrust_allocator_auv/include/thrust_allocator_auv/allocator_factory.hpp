/**
 * @file allocator_factory.hpp
 * @brief Contains the Allocator Factory, which makes it easier
 * to choose allocator at runtime i.e imported through ROS params.
 */

#ifndef THRUST_ALLOCATOR_AUV__ALLOCATOR_FACTORY_HPP_
#define THRUST_ALLOCATOR_AUV__ALLOCATOR_FACTORY_HPP_

#include <string>
#include <memory>
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
        static std::unique_ptr<Allocator> make_allocator(const std::string& allocator_type);
    };
    
#endif // THRUST_ALLOCATOR_AUV__ALLOCATOR_FACTORY_HPP_
