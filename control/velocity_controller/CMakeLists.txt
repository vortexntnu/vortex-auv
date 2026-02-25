cmake_minimum_required(VERSION 3.8)
project(velocity_controller)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
#find_package(CasADi REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ct_optcon REQUIRED)
find_package(ct_core REQUIRED)
find_package(casadi REQUIRED)

set(ACADOS_ROOT "$ENV{ACADOS_SOURCE_DIR}" CACHE PATH "acados root")
include_directories(
  ${ACADOS_ROOT}/include
  ${ACADOS_ROOT}/include/acados
  ${ACADOS_ROOT}/include/acados/ocp_nlp
  ${ACADOS_ROOT}/include/acados_c
  ${ACADOS_ROOT}/include/blasfeo/include
  ${ACADOS_ROOT}/include/hpipm/include
  ${ACADOS_ROOT}/include/qpOASES_e
  ${CMAKE_CURRENT_SOURCE_DIR}/src/build_auv_solver

  include

)
link_directories(${ACADOS_ROOT}/lib)


# After setting ACADOS_ROOT etc.
file(GLOB_RECURSE GEN_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/src/build_auv_solver/*.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/build_auv_solver/*/*.c
)



add_library(auv_nmpc STATIC
  src/NMPC_acados.cpp
  ${GEN_SRCS}
)


add_executable(velocity_controller_node
  src/velocity_controller.cpp
  src/PID_setup.cpp
  src/LQR_setup.cpp
  src/utilities.cpp
  src/ct_instantiations.cpp
  src/NMPC_setup.cpp
  #src/NMPC_acados.cpp
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
 # CasADi
  nav_msgs
  vortex_utils
)
#target_include_directories(velocity_controller_node PRIVATE casadi Eigen3)
target_link_libraries(auv_nmpc acados hpipm blasfeo qpOASES_e m)
target_link_libraries(velocity_controller_node Eigen3::Eigen casadi::casadi ct_optcon ct_core auv_nmpc)
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
  add_subdirectory(tests) 
endif()

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


ament_package()
