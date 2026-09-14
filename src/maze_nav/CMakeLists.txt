cmake_minimum_required(VERSION 3.8)
project(maze_nav)

find_package(ament_cmake REQUIRED)
# Питоновский модуль route_graph нужен и узлам, и редактору маршрутов,
# поэтому пакет ставится в site-packages, а не лежит рядом со скриптами.
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Пакет без кода: только данные. Их надо положить в share/,
# чтобы launch-файлы находили их через get_package_share_directory().
install(
  DIRECTORY launch worlds models maps params scripts rviz
  DESTINATION share/${PROJECT_NAME}
)

# Скрипты — исполняемые, чтобы работал ros2 run maze_nav <имя>.py
install(
  PROGRAMS
    scripts/gen_maze.py
    scripts/gen_city_map.py
    scripts/scan_fov.py
    scripts/enc_check.py
    scripts/calib_odom.py
    scripts/odom_compare.py
    scripts/yaw_check.py
    scripts/dir_check.py
    scripts/scan_sectors.py
    scripts/route_editor.py
    scripts/route_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
