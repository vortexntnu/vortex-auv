cmake_minimum_required(VERSION 3.3)

# Find ROS build system
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# Describe ROS project
option(ENABLE_ROS "Enable or disable building with ROS (if it is found)" ON)
if (NOT ENABLE_ROS)
    message(FATAL_ERROR "Only ROS 2 builds are supported.")
endif ()
add_definitions(-DROS_AVAILABLE=2)

# Include our header files
include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${Boost_LIBRARIES}
        ${CERES_LIBRARIES}
        ${OpenCV_LIBRARIES}
)
list(APPEND ament_libraries
        rclcpp
        tf2_ros
        tf2_geometry_msgs
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        cv_bridge
        image_transport
)

##################################################
# Fetch OpenVINS from upstream and build ov_core, ov_init and ov_msckf from it
##################################################

# Pinned to the exact upstream commit we tested with (master, 2025-11-30).
# Offline builds: pass -DFETCHCONTENT_SOURCE_DIR_OPEN_VINS=/path/to/open_vins to use a local checkout.
# The patch adds output hooks to ROS2Visualizer that RosVio overrides.
# It resets the file first so re-running the patch step is safe.
include(FetchContent)
FetchContent_Declare(open_vins
        GIT_REPOSITORY https://github.com/rpng/open_vins.git
        GIT_TAG 69488123ed9362dd44b6f28e7f4680abbff1442b
        PATCH_COMMAND git checkout -- ov_msckf/src/ros
              COMMAND git apply ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ros2_visualizer_output_hook.patch
)
# Only download the sources; we build the libraries ourselves below
FetchContent_GetProperties(open_vins)
if (NOT open_vins_POPULATED)
    FetchContent_Populate(open_vins)
endif ()
set(OV_CORE_SRC ${open_vins_SOURCE_DIR}/ov_core/src)
set(OV_INIT_SRC ${open_vins_SOURCE_DIR}/ov_init/src)
set(OV_MSCKF_SRC ${open_vins_SOURCE_DIR}/ov_msckf/src)

# Aruco tags need opencv contrib, we do not use them (keep use_aruco: false in the config)
add_definitions(-DENABLE_ARUCO_TAGS=0)

# ov_core: feature tracking, camera models, types, utils
add_library(ov_core_lib STATIC
        ${OV_CORE_SRC}/cpi/CpiV1.cpp
        ${OV_CORE_SRC}/cpi/CpiV2.cpp
        ${OV_CORE_SRC}/sim/BsplineSE3.cpp
        ${OV_CORE_SRC}/track/TrackBase.cpp
        ${OV_CORE_SRC}/track/TrackAruco.cpp
        ${OV_CORE_SRC}/track/TrackDescriptor.cpp
        ${OV_CORE_SRC}/track/TrackKLT.cpp
        ${OV_CORE_SRC}/track/TrackSIM.cpp
        ${OV_CORE_SRC}/types/Landmark.cpp
        ${OV_CORE_SRC}/feat/Feature.cpp
        ${OV_CORE_SRC}/feat/FeatureDatabase.cpp
        ${OV_CORE_SRC}/feat/FeatureInitializer.cpp
        ${OV_CORE_SRC}/utils/print.cpp
)
set_target_properties(ov_core_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(ov_core_lib PUBLIC ${OV_CORE_SRC})
ament_target_dependencies(ov_core_lib rclcpp cv_bridge)
target_link_libraries(ov_core_lib ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

# ov_init: static and dynamic initialization
add_library(ov_init_lib STATIC
        ${OV_INIT_SRC}/ceres/Factor_GenericPrior.cpp
        ${OV_INIT_SRC}/ceres/Factor_ImageReprojCalib.cpp
        ${OV_INIT_SRC}/ceres/Factor_ImuCPIv1.cpp
        ${OV_INIT_SRC}/ceres/State_JPLQuatLocal.cpp
        ${OV_INIT_SRC}/init/InertialInitializer.cpp
        ${OV_INIT_SRC}/dynamic/DynamicInitializer.cpp
        ${OV_INIT_SRC}/static/StaticInitializer.cpp
        ${OV_INIT_SRC}/sim/SimulatorInit.cpp
)
set_target_properties(ov_init_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(ov_init_lib PUBLIC ${OV_INIT_SRC} ${CERES_INCLUDE_DIRS})
ament_target_dependencies(ov_init_lib rclcpp cv_bridge)
target_link_libraries(ov_init_lib ov_core_lib ${CERES_LIBRARIES})

list(APPEND thirdparty_libraries ov_init_lib ov_core_lib)

##################################################
# ov_msckf: upstream estimator + our RosVio
##################################################

add_library(ov_msckf_lib SHARED
        ${OV_MSCKF_SRC}/sim/Simulator.cpp
        ${OV_MSCKF_SRC}/state/State.cpp
        ${OV_MSCKF_SRC}/state/StateHelper.cpp
        ${OV_MSCKF_SRC}/state/Propagator.cpp
        ${OV_MSCKF_SRC}/core/VioManager.cpp
        ${OV_MSCKF_SRC}/core/VioManagerHelper.cpp
        ${OV_MSCKF_SRC}/update/UpdaterHelper.cpp
        ${OV_MSCKF_SRC}/update/UpdaterMSCKF.cpp
        ${OV_MSCKF_SRC}/update/UpdaterSLAM.cpp
        ${OV_MSCKF_SRC}/update/UpdaterZeroVelocity.cpp
        ${OV_MSCKF_SRC}/ros/ROS2Visualizer.cpp
        ${OV_MSCKF_SRC}/ros/ROSVisualizerHelper.cpp
        src/RosVio.cpp
)
ament_target_dependencies(ov_msckf_lib ${ament_libraries})
target_link_libraries(ov_msckf_lib ${thirdparty_libraries})
target_include_directories(ov_msckf_lib PUBLIC src/ ${OV_MSCKF_SRC})
install(TARGETS ov_msckf_lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        PUBLIC_HEADER DESTINATION include
)
install(DIRECTORY src/ ${OV_MSCKF_SRC}/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
ament_export_include_directories(include)
ament_export_libraries(ov_msckf_lib)

##################################################
# Make binary files!
##################################################

add_executable(run_subscribe_msckf src/run_subscribe_msckf.cpp)
ament_target_dependencies(run_subscribe_msckf ${ament_libraries})
target_link_libraries(run_subscribe_msckf ov_msckf_lib ${thirdparty_libraries})
install(TARGETS run_subscribe_msckf DESTINATION lib/${PROJECT_NAME})

# Install launch and config directories
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/)

# finally define this as the package
ament_package()
