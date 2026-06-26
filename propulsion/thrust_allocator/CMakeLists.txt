cmake_minimum_required(VERSION 3.22)

project(thrust_allocator
    VERSION 0.1.0
    LANGUAGES CXX
)

find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(casadi QUIET)

set(THRUST_ALLOCATOR_SOURCES
    src/thrust_allocator.cpp
    src/solvers/allocator_factory.cpp
    src/solvers/pseudoinverse_allocator.cpp
)

if(casadi_FOUND)
    list(APPEND THRUST_ALLOCATOR_SOURCES
        src/solvers/qp_allocator.cpp
    )
endif()

add_library(thrust_allocator
    ${THRUST_ALLOCATOR_SOURCES}
)

add_library(vortex::thrust_allocator ALIAS thrust_allocator)

target_compile_features(thrust_allocator
    PUBLIC
        cxx_std_20
)

target_include_directories(thrust_allocator
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(thrust_allocator
    PUBLIC
        Eigen3::Eigen
        spdlog::spdlog
)

if(casadi_FOUND)
    target_link_libraries(thrust_allocator
        PRIVATE
            casadi
    )

    target_compile_definitions(thrust_allocator
        PRIVATE
            VORTEX_THRUST_ALLOCATOR_HAS_CASADI=1
    )

    message(STATUS "thrust_allocator: CasADi found, QP allocator enabled")
else()
    message(STATUS "thrust_allocator: CasADi not found, building pseudoinverse allocator only")
endif()

target_compile_options(thrust_allocator
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)
