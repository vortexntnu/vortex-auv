cmake_minimum_required(VERSION 3.8)
project(velocity_controller)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(vortex_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CasADi REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ct_optcon REQUIRED)
find_package(ct_core REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  include
)

add_executable(velocity_controller_node
  src/velocity_controller.cpp
  src/PID_setup.cpp
  src/LQR_setup.cpp
  src/utilities.cpp
  src/ct_instantiations.cpp
)



#add_executable(test_LQR_node
#  src/LQR_test.cpp
#  src/LQR_setup.cpp
#  src/utilities.cpp
#  src/ct_instantiations.cpp
#  )
ament_target_dependencies(velocity_controller_node
  rclcpp
  std_msgs
  vortex_msgs
  geometry_msgs
  CasADi
  nav_msgs
  vortex_utils
)

target_link_libraries(velocity_controller_node Eigen3::Eigen ct_optcon ct_core)

install(TARGETS
  velocity_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config 
  DESTINATION share/${PROJECT_NAME}/
)
if(BUILD_TESTING)
  
  #find_package(GTest REQUIRED)
  #find_package(yaml-cpp REQUIRED)
  #find_package(tf2 REQUIRED)
  include(GoogleTest)
  find_package(ament_cmake_gtest REQUIRED)

  set(TEST_BINARY_NAME ${PROJECT_NAME}_test)

  ament_add_gtest(${TEST_BINARY_NAME}
    test/test_PID.cpp
    src/utilities.cpp
    src/ct_instantiations.cpp
    src/LQR_setup.cpp
    src/PID_setup.cpp
  )
  target_include_directories(${TEST_BINARY_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
  )
  ament_target_dependencies(${TEST_BINARY_NAME}
    rclcpp
    std_msgs
    vortex_msgs
    geometry_msgs
    nav_msgs
    vortex_utils
    )
    target_link_libraries(${TEST_BINARY_NAME}
    Eigen3::Eigen
    ct_optcon
    ct_core
    )

  add_executable(test_VC_node
    src/test_VC.cpp
    src/utilities.cpp
    src/ct_instantiations.cpp
  )

  target_include_directories(test_VC_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR} 
  )


  target_link_libraries(test_VC_node Eigen3::Eigen ct_optcon ct_core)


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

  ament_target_dependencies(test_VC_node
    rclcpp
    std_msgs
    vortex_msgs
    geometry_msgs
    nav_msgs
    vortex_utils
  )

  install(TARGETS test_VC_node
  DESTINATION lib/${PROJECT_NAME}
  )

  #gtest_discover_tests(${TEST_BINARY_NAME})

endif()


ament_package()
