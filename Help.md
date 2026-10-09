
# В контейнере после ./run.sh запуск базы со сбором камеры
ros2 launch maze_nav robot_nav.launch.py \
    nav2:=true imu:=false dropoff:=true force_2d:=true

# На хосте, нужно поменять путь. Запуск сегментации
cd ~/roshack/Roshack-yolodocker && docker compose up rail_mask

# В контейнере после ./run.sh визуализация сегментации, уже то что пришло в основной контейнер
python3 /root/ros_ws/yolo_transport_test/mask_view.py


# В контейнере после ./run.sh Запуск заезда на рельсы 
ros2 run maze_nav rail_entry.py --ros-args \
    -r odom:=/odom -r imu:=/imu/data -r obstacles:=/rails/map \
    -p scan:=false -p drive_distance:=1.5