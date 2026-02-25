cmake_minimum_required(VERSION 3.8)
include(GoogleTest)
find_package(ament_cmake_gtest REQUIRED)
#find_package(GTest REQUIRED)
find_package(yaml-cpp REQUIRED)
  #find_package(tf2 REQUIRED)

#Unit test
set(TEST_BINARY_NAME PID_test)

ament_add_gtest(${TEST_BINARY_NAME}
test_PID.cpp
../src/utilities.cpp
../src/ct_instantiations.cpp
../src/PID_setup.cpp
)

ament_add_gtest(LQR_test
test_LQR.cpp
../src/utilities.cpp
../src/ct_instantiations.cpp
../src/LQR_setup.cpp
)

#To DO need to clean up the include directories/dependencies and such
target_include_directories(${TEST_BINARY_NAME} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
${EIGEN3_INCLUDE_DIR}
)
target_include_directories(LQR_test PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
${EIGEN3_INCLUDE_DIR}
) #Need to fix
ament_target_dependencies(${TEST_BINARY_NAME}
rclcpp
std_msgs
vortex_msgs
geometry_msgs
nav_msgs
vortex_utils
)
ament_target_dependencies(LQR_test
  rclcpp
  vortex_utils
  std_msgs
  geometry_msgs
  vortex_msgs
  nav_msgs
)
target_compile_definitions(LQR_test PRIVATE 
YAML_PATH="${PROJECT_SOURCE_DIR}/config/parameters.yaml")

target_link_libraries(${TEST_BINARY_NAME}
Eigen3::Eigen
ct_optcon
ct_core
)
target_link_libraries(LQR_test
  Eigen3::Eigen
  ct_optcon
  ct_core
  yaml-cpp
)


#System tests


target_link_libraries(
    ${TEST_BINARY_NAME}
    ${LIB_NAME}
    #yaml-cpp
    #GTest::GTest
    #spdlog::spdlog
    Eigen3::Eigen 
    ct_optcon 
    ct_core
)


#gtest_discover_tests(${TEST_BINARY_NAME})