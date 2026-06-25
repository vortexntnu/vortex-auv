cmake_minimum_required(VERSION 3.22)

project(thrust_allocator
    VERSION 0.1.0
    LANGUAGES CXX
)

find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)

# Keep only if qp_allocator.cpp uses CasADi.
find_package(casadi REQUIRED)

add_library(thrust_allocator
    src/thrust_allocator.cpp

    src/solvers/allocator_factory.cpp
    src/solvers/pseudoinverse_allocator.cpp
    src/solvers/qp_allocator.cpp
)

add_library(vortex::thrust_allocator ALIAS thrust_allocator)

target_compile_features(thrust_allocator
    PUBLIC
        cxx_std_20
)

target_include_directories(thrust_allocator
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(thrust_allocator
    PUBLIC
        Eigen3::Eigen
        spdlog::spdlog
        casadi
)

target_compile_options(thrust_allocator
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)
