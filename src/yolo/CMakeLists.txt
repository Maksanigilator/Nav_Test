cmake_minimum_required(VERSION 3.8)
project(yolo)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Типы, встречающиеся в полях FrameDetections.msg.
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FrameDetections.msg"
  DEPENDENCIES std_msgs geometry_msgs vision_msgs
)

# Узел — обычный скрипт, ставим его как исполняемый под коротким именем:
#   ros2 run yolo yolo_client
# yolo_client_node.py — старый пример, он не устанавливается (копия в notes2.md).
install(
  PROGRAMS yolo_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_client
)

# Модель ищем через get_package_share_directory('yolo'), чтобы узел не
# зависел от того, откуда его запустили.
install(
  FILES dataset_v3_180.pt
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
