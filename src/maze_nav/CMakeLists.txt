cmake_minimum_required(VERSION 3.8)
project(maze_nav)

find_package(ament_cmake REQUIRED)

# Пакет без кода: только данные. Их надо положить в share/,
# чтобы launch-файлы находили их через get_package_share_directory().
install(
  DIRECTORY launch worlds models maps params scripts behavior_trees rviz
  DESTINATION share/${PROJECT_NAME}
)

# Узлы, которые запускаются из launch через Node(executable=...), должны
# лежать в lib/, а не только в share/: ros2 ищет исполняемые файлы именно там.
# Остальные скрипты в scripts/ запускаются по полному пути (см. RUN.md),
# им это не нужно.
install(PROGRAMS
  scripts/imu_noise.py
  scripts/depth_noise.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
