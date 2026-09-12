cmake_minimum_required(VERSION 3.8)
project(maze_nav)

find_package(ament_cmake REQUIRED)

# Пакет без кода: только данные. Их надо положить в share/,
# чтобы launch-файлы находили их через get_package_share_directory().
install(
  DIRECTORY launch worlds models maps params scripts rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
