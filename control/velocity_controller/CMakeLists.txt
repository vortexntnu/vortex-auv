cmake_minimum_required(VERSION 3.8)
project(velocity_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DRAKE_DIR /ros2_ws_v/src/drake-build/install/lib/cmake/drake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vortex_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(drake CONFIG REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${drake_INCLUDE_DIRS}
  include
)

link_directories("/opt/drake/lib/")

#set(LIB_NAME "${PROJECT_NAME}_component")

#add_library(${LIB_NAME} SHARED
#  src/LQR_setup.cpp
#  src/PID_setup.cpp
#  src/test_VC.cpp
#  src/velocity_controller.cpp
#)
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
#  src/velocity_controller.cpp
)

ament_target_dependencies(velocity_controller_node
  rclcpp
  std_msgs
  vortex_msgs
  geometry_msgs
  Eigen3
  #drake
)

ament_target_dependencies(test_VC_node
  rclcpp
  std_msgs
  vortex_msgs
  geometry_msgs
  Eigen3
)

install(TARGETS
  velocity_controller_node
  test_VC_node
  DESTINATION lib/${PROJECT_NAME}
)


#rclcpp_components_register_node(
#  ${LIB_NAME}
#  PLUGIN "velocity_controller_node"
#  EXECUTABLE ${PROJECT_NAME}_node
#)
#ament_export_targets(export_${LIB_NAME})
#install(TARGETS ${LIB_NAME}
#  EXPORT export_${LIB_NAME}
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
#)
install(TARGETS
  velocity_controller_node
  test_VC_node
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

#add_executable(velocity_controller_node src/velocity_controller.cpp src/PID_setup.cpp)
#add_executable(test_VC_node src/test_VC.cpp src/PID_setup.cpp src/velocity_controller.cpp)

#target_include_directories(velocity_controller_node PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>
#)

#target_link_libraries(velocity_controller_node ${drake_LIBRARIES})
#install(DIRECTORY launch 
#  DESTINATION share/${PROJECT_NAME}/
#)
#install(TARGETS velocity_controller_node
#  DESTINATION lib/${PROJECT_NAME}
#)
#install(TARGETS test_VC_node
#  DESTINATION lib/${PROJECT_NAME}
#)
#install(DIRECTORY config
#  DESTINATION share/${PROJECT_NAME}/
#)


