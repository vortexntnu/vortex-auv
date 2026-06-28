cmake_minimum_required(VERSION 3.20)

project(dp_adapt_backs_controller_quat
  LANGUAGES CXX
)

add_library(${PROJECT_NAME}
  src/dp_adapt_backs_controller.cpp
  src/dp_adapt_backs_controller_utils.cpp
)

add_library(vortex::dp_adapt_backs_controller_quat ALIAS ${PROJECT_NAME})

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
