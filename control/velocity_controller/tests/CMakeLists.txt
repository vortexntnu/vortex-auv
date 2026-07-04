cmake_minimum_required(VERSION 3.8)
include(GoogleTest)
find_package(ament_cmake_gtest REQUIRED)
find_package(yaml-cpp REQUIRED)

# ---------- Felles include-directories, brukt av alle gtest-targets ----------
set(TEST_INCLUDE_DIRS
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# ---------- Felles ROS-avhengigheter for testkode (headere brukt direkte i test_*.cpp) ----------
set(TEST_ROS_DEPS
    rclcpp
    std_msgs
    vortex_msgs
    geometry_msgs
    nav_msgs
    vortex_utils
)

# Hjelpefunksjon for å unngå å gjenta samme 5 linjer for hvert target
function(add_velocity_controller_test TEST_NAME)
    ament_add_gtest(${TEST_NAME} ${ARGN})
    target_include_directories(${TEST_NAME} PUBLIC ${TEST_INCLUDE_DIRS})
    ament_target_dependencies(${TEST_NAME} ${TEST_ROS_DEPS})
    # Lenk mot selve komponentbiblioteket - inneholder ALLE .cpp-filer
    # (utilities, PID_controller, LQR_setup, controller, 3DOF_PID,
    # control_manager, ct_instantiations, velocity_controller_ros) samt
    # riktige include-stier for thrust_allocator_auv/vortex_utils_ros
    # og deres ament_target_dependencies/target_link_libraries transitivt.
    target_link_libraries(${TEST_NAME} ${LIB_NAME} Eigen3::Eigen)
endfunction()

# ================= PID_controller =================
add_velocity_controller_test(PID_test test_PID.cpp)

# ================= LQR_setup =================
add_velocity_controller_test(LQR_test test_LQR.cpp)
target_compile_definitions(LQR_test PRIVATE
    YAML_PATH="${PROJECT_SOURCE_DIR}/config/parameters.yaml")
target_link_libraries(LQR_test yaml-cpp)

# ================= PID_3DOF =================
add_velocity_controller_test(PID_3DOF_test test_3DOF_PID.cpp)

# ================= control_manager =================
add_velocity_controller_test(control_manager_test test_control_manager.cpp)
target_compile_definitions(control_manager_test PRIVATE
    YAML_PATH="${PROJECT_SOURCE_DIR}/config/parameters.yaml")

# ================= utilities =================
add_velocity_controller_test(utilities_test test_utilities.cpp)

# ================= System test (eksisterende, uendret) =================
add_executable(test_vc_node
    test_VC.cpp
)
target_include_directories(test_vc_node PUBLIC ${TEST_INCLUDE_DIRS})
target_link_libraries(test_vc_node ${LIB_NAME} Eigen3::Eigen)
ament_target_dependencies(test_vc_node
    rclcpp
    rclcpp_lifecycle
    ${TEST_ROS_DEPS}
)
install(TARGETS test_vc_node
    DESTINATION lib/${PROJECT_NAME}
)