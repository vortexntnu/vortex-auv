cmake_minimum_required(VERSION 3.22)

project(vortex_auv
  VERSION 0.1.0
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(VORTEX_BUILD_TESTS "Build unit tests" OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

# ------------------------------------------------------------
# Shared third-party dependencies.
#
# These can instead live in each package CMakeLists.txt. Keeping
# them here is useful while validating the whole repository build.
# ------------------------------------------------------------

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)

# ------------------------------------------------------------
# Internal libraries.
#
# Add low-level dependencies before libraries that depend on them.
# ------------------------------------------------------------

add_subdirectory(io/can_interface)

add_subdirectory(propulsion/thrust_allocator)
add_subdirectory(propulsion/thruster_interface)

add_subdirectory(control/dp_adapt_backs_controller_quat)
add_subdirectory(guidance/reference_filter_dp_quat)
add_subdirectory(runtime/vehicle_control)
# ------------------------------------------------------------
# Tests
# ------------------------------------------------------------

if(VORTEX_BUILD_TESTS)
  include(CTest)
  enable_testing()

  find_package(GTest REQUIRED)

  # Uncomment when you have a root-level test folder.
  # add_subdirectory(tests)
endif()
