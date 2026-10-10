cmake_minimum_required(VERSION 3.8)
project(maze_nav_interfaces)

# ОТДЕЛЬНЫЙ ПАКЕТ, А НЕ ЧАСТЬ maze_nav — намеренно.
#
# Генерируемые интерфейсы можно объявить и в maze_nav, он тоже ament_cmake.
# Но тогда в одном пакете окажутся и сгенерированный питоновский модуль
# maze_nav.action, и наши скрипты, и импорты начинают перекрывать друг
# друга непредсказуемым образом. Разделение — стандартная практика ROS 2
# ровно по этой причине.
#
# Практическое следствие: правка .action требует настоящей пересборки
# (colcon build), тогда как скрипты в maze_nav лежат по символической
# ссылке и подхватываются без неё.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/RailEntry.action"
  "action/RailExit.action"
  DEPENDENCIES action_msgs
)

ament_package()
