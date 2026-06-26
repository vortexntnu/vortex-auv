cmake_minimum_required(VERSION 3.22)

project(vortex_auv
    VERSION 0.1.0
    LANGUAGES CXX
)

add_subdirectory(io/can_interface)

add_subdirectory(propulsion/thrust_allocator)
add_subdirectory(propulsion/thruster_interface)

# enable_testing()
# add_subdirectory(tests)
