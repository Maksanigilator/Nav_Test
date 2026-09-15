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

---

# Навигация по RGB-D (ветка rgbd-nav)

## Этап 1. RTAB-Map на датасете TUM RGB-D

Опорная точка перед симулятором: прогон на данных с ground truth. Если здесь
ошибка в пределах нормы, дальнейшие провалы можно списывать на данные и камеру,
а не на сам алгоритм.

### 1. Скачать датасет

```bash
mkdir -p ~/Nav_Test/datasets && cd ~/Nav_Test/datasets
curl -L -C - --retry 20 --retry-all-errors \
  -O https://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_long_office_household.bag
```

1.6 ГБ. `-C -` обязателен: докачка нужна, если соединение рвётся (например,
через локальный прокси). Проверить целостность — размер должен быть
ровно `1700571628` байт:

```bash
stat -c%s rgbd_dataset_freiburg3_long_office_household.bag
```

### 2. Сконвертировать в ROS 2

Штатный рецепт RTAB-Map требует ROS 1 (`rosbag decompress`, `roscore` и два
ROS1-скрипта). Вместо него — свой конвертер, ROS 1 не нужен:

```bash
./run.sh python3 /root/ros_ws/src/maze_nav/scripts/tum_bag_to_ros2.py \
  /datasets/rgbd_dataset_freiburg3_long_office_household.bag
```

Занимает несколько минут. Ключ `--force` перезаписывает результат,
`--keep-all-topics` оставляет `/cortex_marker_array` (по умолчанию он
выбрасывается — RTAB-Map его не читает, а весит он треть бэга).

Проверить результат:

```bash
./run.sh ros2 bag info /datasets/rgbd_dataset_freiburg3_long_office_household
```

Ожидается 5 топиков, 22395 сообщений, 87.3 с. Тип `/tf` должен быть
`tf2_msgs/msg/TFMessage`, а не ROS1-овский `tf/msg/tfMessage`.

### 3. Прогнать RTAB-Map

Собрать пакет и запустить в первом терминале:

```bash
./run.sh
colcon build --symlink-install && source install/setup.bash
ros2 launch maze_nav rtabmap_tum.launch.py
```

Во втором терминале:

```bash
./run.sh
ros2 bag play /datasets/rgbd_dataset_freiburg3_long_office_household --clock
```

`--clock` обязателен: launch поднимает узлы с `use_sim_time:=true`.

Аргументы launch:

* `viz:=true` — включить GUI `rtabmap_viz` (нужен X11);
* `database_path:=...` — куда класть базу, по умолчанию `/datasets/rtabmap_tum.db`;
* `publish_camera_tf:=true` — публиковать статику `kinect -> openni_rgb_optical_frame`.
  **По умолчанию выключено и включать не надо.** Подробности — в README.

### 4. Снять отчёт

После остановки launch по Ctrl+C:

```bash
./run.sh env QT_QPA_PLATFORM=offscreen rtabmap-report --loop /datasets/rtabmap_tum.db
```

`QT_QPA_PLATFORM=offscreen` обязателен: `rtabmap-report` — Qt-приложение,
а в образе выставлен `QT_QPA_PLATFORM=xcb`, и без переопределения оно падает
с «could not connect to display».

Полезные ключи: `--poses` выгружает траектории (одометрия, оптимизированный
граф, ground truth) в формате TUM RGB-D — их можно скормить `evo_ape`.

Сводка по базе без Qt:

```bash
./run.sh rtabmap-info /datasets/rtabmap_tum.db
```

### Что должно получиться

```
rtabmap_tum.db (137, 20.0 m): RMSE= 0.020 m (max=0.053m, odom=0.059 m) ang=0.9 deg,
slam: avg=54 ms (max=173 ms) loops=16 (t_err=0.017m r_err=0.55 deg),
odom: avg=41ms (max=60ms)
```

На что смотреть:

* `loops=16` — замыкания петли срабатывают. Если здесь 0, RTAB-Map работает
  как чистая одометрия, и смысла в нём нет;
* `RMSE=0.020 m` против `odom=0.059 m` — SLAM втрое точнее голой одометрии,
  значит замыкания не просто находятся, но и исправляют траекторию;
* `odom: avg=41ms` — одометрия успевает за 24 Гц, то есть реальное время.

### 5. Посмотреть глазами

Три способа, от самого дешёвого к самому подробному.

**Готовые графики траекторий.** Ничего запускать не надо, картинки уже лежат
в `datasets/`: `traj_trajectories.png` (вид сверху: эталон, SLAM, одометрия),
`traj_xyz.png`, `traj_rpy.png`, `traj_speeds.png`.

Пересобрать их можно так:

```bash
./run.sh bash -c '
  source /opt/ros/jazzy/setup.bash
  export QT_QPA_PLATFORM=offscreen MPLBACKEND=Agg
  cd /datasets
  rtabmap-report --poses /datasets/rtabmap_tum.db
  # rtabmap-report пишет заголовок "#timestamp ..." и девятую колонку id,
  # а evo требует ровно 8 полей без заголовка — приводим к его формату
  for f in gt odom slam; do
    grep -v "^#" rtabmap_tum_$f.txt | awk "{print \$1,\$2,\$3,\$4,\$5,\$6,\$7,\$8}" > evo_$f.txt
  done
  evo_traj tum evo_slam.txt evo_odom.txt --ref evo_gt.txt \
    --plot_mode xy --save_plot /datasets/traj
  evo_ape tum evo_gt.txt evo_slam.txt -a
'
```

**Разбор готовой базы.** `rtabmap-databaseViewer` — отладчик, а не плеер:
нужен, когда что-то не сошлось и надо понять почему.

```bash
xhost +local:docker
./run.sh rtabmap-databaseViewer /datasets/rtabmap_tum.db
```

Первым делом — `View -> Concise Layout`, иначе окно открывает разом девять
панелей. Разбор интерфейса и остальных утилит RTAB-Map — в [RTABMAP.md](RTABMAP.md).

**Живая демонстрация (то, что показывают в роликах).** Картинка с камеры
с отслеживаемыми признаками, накапливающееся 3D-облако и вспыхивающие замыкания
петли. Бэг проигрывается сам, второй терминал не нужен:

```bash
xhost +local:docker          # один раз на хосте
./run.sh
# дальше внутри контейнера:
colcon build --symlink-install && source install/setup.zsh
ros2 launch maze_nav rtabmap_tum.launch.py viz:=true play:=true rate:=0.5
```

Пересборка нужна после любой правки launch-файлов и скриптов: артефакты
`colcon` лежат в именованных томах `nav2_build`/`nav2_install`, а не на хосте,
и сами по себе от изменений в `src/` не обновляются. Оболочка в контейнере —
zsh, поэтому `setup.zsh`, а не `setup.bash`.

* `play:=true` — проигрывать бэг самим, не открывая второй терминал. Старт
  отложен на 8 секунд: узлы поднимаются не мгновенно, а сообщения, пришедшие
  до подписки, просто пропадут, и первые секунды датасета выпадут из карты;
* `rate:=0.5` — вдвое медленнее, чтобы успевать смотреть. `rate:=2.0` наоборот;
* `bag:=...` — другой датасет.

На что смотреть в окне `rtabmap_viz`:

* верхняя панель — кадр с камеры, жёлтым отмечены отслеживаемые признаки.
  Когда их мало (пустая стена, резкий поворот), одометрия близка к срыву —
  именно это будет главной проблемой в сером лабиринте на этапе 2;
* центральная 3D-панель — карта, которая достраивается по ходу;
* в момент замыкания петли граф «дёргается»: накопленный дрейф схлопывается,
  и облако выравнивается. На этом датасете такое случится 16 раз.

Если нужны два терминала по отдельности (например, чтобы перезапускать бэг
без перезапуска SLAM), `play` не указывай:

```bash
./run.sh                     # терминал 1
colcon build --symlink-install && source install/setup.zsh
ros2 launch maze_nav rtabmap_tum.launch.py viz:=true

./run.sh                     # терминал 2
ros2 bag play /datasets/rgbd_dataset_freiburg3_long_office_household --clock
```

X11 в `run.sh` уже настроен (проброс `$XAUTHORITY` и `/tmp/.X11-unix`),
отдельно ничего доставлять не нужно.

---

## Этап 2. Навигация по RGB-D в Gazebo, без лидара и колёсной одометрии

```bash
xhost +local:docker          # один раз на хосте
./run.sh
colcon build --symlink-install && source install/setup.zsh
ros2 launch maze_nav rgbd_sim.launch.py nav2:=true
```

Аргументы:

* `nav2:=true` — поднять стек Nav2 (по умолчанию только симулятор и RTAB-Map);
* `viz:=true` — окно `rtabmap_viz` с живой картой и признаками;
* `gui:=false` — без окна Gazebo, только физика;
* `world:=...` — другой мир. По умолчанию `turtlebot3_house.world`: у него есть
  настоящие текстуры. **Серый `maze1.world` не годится** — в нём ни одной
  текстуры, и визуальная одометрия слепа;
* `x_pose:`, `y_pose:` — точка спавна.

### Отправить цель

Во втором терминале:

```bash
./run.sh
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.5}, orientation: {w: 1.0}}}}"
```

**Внимание про координаты.** Начало координат карты RTAB-Map — это стартовая
поза робота, а не начало мира Gazebo. Если робот заспавнен в `world(-2.0, 0.5)`,
то цель `map(0.0, 0.5)` окажется в `world(-2.0, 1.0)`. Не перепутать при сверке
с ground truth.

### Посмотреть в RViz

```bash
./run.sh rviz2
```

Что добавить: `Map` на `/map`, `PointCloud2` на `/cloud`, костмапы на
`/local_costmap/costmap` и `/global_costmap/costmap`, `TF`.

### Проверить, что всё живо

```bash
ros2 topic hz /camera/image_raw /odom /cloud /local_costmap/costmap
ros2 run tf2_ros tf2_echo map odom          # даёт RTAB-Map
ros2 run tf2_ros tf2_echo odom base_footprint   # даёт визуальная одометрия
```

Если `odom -> base_footprint` нет — одометрия не поехала. Колёсной одометрии
в этой сборке нет намеренно, подменить её некому.

### Ожидаемый результат

```
Goal finished with status: SUCCEEDED
одометрия: quality около 500, потерь 0
карта: примерно 200x150 клеток после короткого проезда
```
