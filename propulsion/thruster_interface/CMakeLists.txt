find_package(Eigen3 REQUIRED)

add_library(thruster_interface
  src/thruster_interface.cpp
)

add_library(vortex::thruster_interface ALIAS thruster_interface)

target_include_directories(thruster_interface
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(thruster_interface
  PUBLIC
    vortex::can_interface
    Eigen3::Eigen
)

target_compile_features(thruster_interface PUBLIC cxx_std_20)

target_compile_options(thruster_interface
  PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)
