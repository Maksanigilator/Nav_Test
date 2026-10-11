cmake_minimum_required(VERSION 3.8)
project(rail_stage1_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ApproachRail.action"
  "action/EnterRail.action"
  DEPENDENCIES action_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
