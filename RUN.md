# Команды запуска

Всё запускается внутри контейнера. Поднять его — `./run.sh` (второй и третий
терминал — та же команда, она подключится к уже работающему).

## Сборка образа

```bash
cd ~/Nav_Test
docker build -f docker/Dockerfile -t nav2:jazzy .
```

Запускать обязательно из `Nav_Test/`, а не из `docker/`: контекст сборки — это `.`,
от него считается путь в `COPY docker/entrypoint.sh`.

## Симулятор с лидаром (исходный стенд)

```bash
ros2 launch maze_nav maze.launch.py                 # лабиринт maze1 + TurtleBot3
ros2 launch maze_nav maze.launch.py gui:=false      # без окна, только физика
ros2 launch maze_nav maze.launch.py world:=maze2.world x_pose:=-3.75 y_pose:=-3.75
```

Мир и позу спавна можно менять: `world:=` ищет файл в `src/maze_nav/worlds/`,
`x_pose:`/`y_pose:` задают точку появления робота. Готовый мир здесь один,
`maze1.world`; как сделать второй — ниже.

## Nav2 в лабиринте

Второй терминал (`./run.sh` подключится к тому же контейнеру):

```bash
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true \
    map:=/root/ros_ws/src/maze_nav/maps/maze1.yaml \
    params_file:=/root/ros_ws/src/maze_nav/params/maze_params.yaml
```

Третий — RViz с готовыми панелями Nav2:

```bash
ros2 launch nav2_bringup rviz_launch.py
```

Цель ставится кнопкой **2D Goal Pose**. Начальную позу подсказывать не нужно:
карта снята с того же мира, и AMCL сходится сам.

Вместо `maze_params.yaml` можно подставить `maze_params_rpp.yaml` (Regulated
Pure Pursuit вместо DWB) или `maze_params_fov180.yaml` (он же плюс AMCL под
урезанный сектор лидара). Разбор — в README, раздел 7.

### Проверить, что всё живо

```bash
ros2 topic hz /scan                 # лидар из Gazebo
ros2 topic echo /amcl_pose --once   # локализация сошлась
ros2 node list | grep -c .          # узлов Nav2 должно быть больше десятка
```

Если `/scan` молчит — не поднялся мост `ros_gz_bridge`, смотреть вывод
`maze.launch.py`. Если `/amcl_pose` пуст, а `/scan` идёт — почти всегда
несовпадение карты и мира.

## Новый лабиринт и карта под него

Лабиринт генерируется на хосте, карта снимается в контейнере.

```bash
python3 src/maze_nav/scripts/gen_maze.py --cells 6 --seed 2 \
    -o src/maze_nav/worlds/maze2.world
```

Генератор строит ИДЕАЛЬНЫЙ лабиринт — между любыми двумя точками ровно один
путь. Все стены складываются в один статический `model` с одним `link`: так
Gazebo обрабатывает их заметно быстрее, чем сотню отдельных моделей.

Дальше карта. Три терминала:

```bash
ros2 launch maze_nav maze.launch.py world:=maze2.world
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Объехать лабиринт целиком, затем сохранить:

```bash
ros2 run nav2_map_server map_saver_cli -f /root/ros_ws/src/maze_nav/maps/maze2
```

Снимать карту обязательно: `maze1.pgm` подходит только к `maze1.world`. При
другом seed геометрия другая, и AMCL со старой картой не сойдётся.

## Сектор обзора лидара

```bash
ros2 run maze_nav scan_fov.py
ros2 run maze_nav scan_fov.py --topic /scan/filtered --count 100
```

Скрипт копит сообщения с `/scan` и для каждого угла считает, как часто луч
возвращает корректную дальность. Лучи, перекрытые конструкцией робота, почти
всегда дают inf/nan/0 — по ним и виден слепой сектор. Нужен, когда AMCL
расходится: по замеру выбирается `maze_params_fov180.yaml` вместо обычного.

## Переходник Twist -> TwistStamped

```bash
ros2 run maze_nav twist_to_stamped.py
ros2 run maze_nav twist_to_stamped.py --ros-args \
    -r in:=/cmd_vel_nav -r out:=/controller/reference
```

Нужен, когда привод ждёт `TwistStamped`, а источник команд говорит простым
`Twist` — так делают и `teleop_twist_keyboard`, и Nav2 в Jazzy по умолчанию
(`enable_stamped_cmd_vel` выключен). Метка времени ставится в момент
пересылки, а не берётся из источника: у цепочечных контроллеров есть
`reference_timeout`, и команду со старым штампом они отбрасывают, обнуляя
колёса.

---

Команды слоёв выше — RTAB-Map и RealSense, заезд на рельсы — лежат в RUN.md
своих ветках, `rgbd-nav` и `agrobot`.
