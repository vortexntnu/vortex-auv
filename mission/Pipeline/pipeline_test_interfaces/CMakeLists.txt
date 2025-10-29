cmake_minimum_required(VERSION 3.8)
project(pipeline_test_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/PingerDOA.msg"
  "msg/PipelineStatus.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(
  rosidl_default_runtime
  geometry_msgs
)
ament_package()
