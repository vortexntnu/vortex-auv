#include "thrust_allocator_auv/allocator.hpp"
#include "thrust_allocator_auv/allocator_factory.hpp"
#include "thrust_allocator_auv/pseudoinverse_allocator.hpp"

std::unique_ptr<Allocator> Factory::make_allocator(const std::string& allocator_type){
    if (allocator_type == "pseudoinverse") {
        return std::make_unique<PseudoinverseAllocator>();
    } 
    
    else if (allocator_type == "QP") {
        return std::make_unique<QPAllocator>();
    } 
    
    else {
        throw std::invalid_argument(
      "Unknown allocator_type: '" + allocator_type + "'. Expected: pseudoinverse/pinv or qp.");
    }
};