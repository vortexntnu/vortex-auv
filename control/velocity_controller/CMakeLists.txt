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
find_package(Eigen3 REQUIRED)
find_package(CasADi REQUIRED)
find_package(LAPACK REQUIRED)
find_package(BLAS REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


include_directories(
  ${EIGEN3_INCLUDE_DIR}
  include
)

add_executable(velocity_controller_node
  src/velocity_controller.cpp
  src/PID_setup.cpp
  src/LQR_setup.cpp
  src/utilities.cpp
)

add_executable(test_VC_node
  src/test_VC.cpp
  src/PID_setup.cpp
  src/LQR_setup.cpp
  src/utilities.cpp
)
add_executable(test_LQR_node
  src/LQR_test.cpp
  src/LQR_setup.cpp
  src/utilities.cpp
)
ament_target_dependencies(velocity_controller_node
  rclcpp
  std_msgs
  vortex_msgs
  geometry_msgs
  Eigen3
  CasADi
  nav_msgs
  
)

ament_target_dependencies(test_VC_node
  rclcpp
  std_msgs
  vortex_msgs
  geometry_msgs
  Eigen3
  nav_msgs
)

ament_target_dependencies(test_LQR_node
  rclcpp
  Eigen3
  vortex_msgs
  geometry_msgs
  std_msgs
  nav_msgs
)

link_directories(/usr/lib/gcc/x86_64-linux-gnu/11)
set(SLICOT_LIB /usr/lib/x86_64-linux-gnu/libslicot.so)
set(GFORTRAN_LIB /usr/lib/gcc/x86_64-linux-gnu/11/libgfortran.so)

target_link_libraries(velocity_controller_node ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES} ${SLICOT_LIB} ${GFORTRAN_LIB})
target_link_libraries(test_VC_node ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES} ${SLICOT_LIB} ${GFORTRAN_LIB})
target_link_libraries(test_LQR_node ${LAPACK_LIBRARIES} ${BLAS_LIBRARIES} ${SLICOT_LIB} ${GFORTRAN_LIB})


install(TARGETS
  velocity_controller_node
  test_VC_node
  test_LQR_node
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
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  #add_subdirectory(test)
endif()

ament_package()
