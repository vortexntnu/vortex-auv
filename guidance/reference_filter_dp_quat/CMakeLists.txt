cmake_minimum_required(VERSION 3.20)

project(reference_filter_dp_quat
  LANGUAGES CXX
)

add_library(${PROJECT_NAME}
  src/reference_filter.cpp
  src/waypoint_follower.cpp
  src/waypoint_guidance_manager.cpp
)

add_library(vortex::reference_filter_dp_quat ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Eigen3::Eigen
    vortex_utils
  PRIVATE
    fmt::fmt
    spdlog::spdlog
)

if(VORTEX_BUILD_TESTS)
  add_subdirectory(test)
endif()
