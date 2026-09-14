# Запуск: все команды

Устройство проекта — в [README.md](README.md). Здесь только команды.

Есть два режима, и различаются они тем, **где считается навигация**.

| | Режим А: Nav2 на ноутбуке | Режим Б: всё на роботе |
|---|---|---|
| когда | отладка, правка конфигов | заезд |
| ноутбук | Nav2, RViz, редактор, детектор | только экран (ssh -X) |
| по сети идёт | сканы, TF, одометрия | картинка окон |
| обрыв Wi-Fi | робот встаёт | робот едет дальше |

Обязательное условие в обоих: везде `ROS_DOMAIN_ID=42` и
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`. В образах прописано, на роботе —
в его окружении.

---

# 1. Найти робота

Адрес меняется при каждом подключении к другой сети.

```bash
ip -4 addr show | grep -oP 'inet \K[\d./]+'        # своя подсеть
for i in $(seq 1 254); do (ping -c1 -W1 192.168.1.$i >/dev/null 2>&1 \
  && echo "живой: 192.168.1.$i") & done; wait      # обход подсети
ip neigh | grep -i '2c:cf:67'                      # MAC Raspberry Pi
ssh pi@<адрес> hostname                            # должно ответить raspberrypi
```

Если ssh ругается на смену ключа хоста — это не атака, адрес просто
переехал к другому устройству:

```bash
ssh-keygen -R <адрес>
```

**Время на роботе и ноутбуке должно совпадать.** Расхождение в секунды
ломает TF: Nav2 сыплет `Transform data too old`, а потом рапортует
`Reached the goal!`, не тронувшись с места. Мы на это потратили час.

```bash
ssh pi@<IP> 'timedatectl status'    # ждём System clock synchronized: yes
ssh pi@<IP> 'sudo timedatectl set-ntp true'
```

---

# 2. Робот: базовый запуск

Нужен всегда, в обоих режимах.

```bash
ssh pi@<IP робота>
ros2 launch frob_bringup bringup.launch.py
```

Именно launch, а не голый `arduino_bridge`: bringup поднимает лидар,
`lidar_filter`, IMU, `robot_state_publisher` (TF по URDF) и одометрию
(энкодерная нода плюс EKF `robot_localization`). Один мост даёт только
`/cmd_vel` и сырые тики — без `/odom` и TF Nav2 не тронется с места.

Камера для детектора знаков нужна **только в режиме А**, вторым терминалом:

```bash
python3 ~/sign_camera.py --fps 5
```

Проверить поток с ноутбука:

```bash
curl -s --noproxy '*' --max-time 5 http://<IP робота>:8080/snapshot -o /tmp/snap.jpg
```

`--noproxy` обязателен при поднятом VPN: иначе запрос к роботу уйдёт
в прокси и вернётся 502.

---

# 3. Режим А: Nav2 на ноутбуке

## 3.1 Поднять контейнеры

```bash
cd ~/Nav_Test
docker compose up -d              # оба контейнера разом
docker compose exec nav2 zsh      # войти в навигационный
docker compose exec yolo bash     # войти в детектор
docker compose down               # остановить
```

Либо по отдельности, тем же эффектом:

```bash
./run.sh          # nav2:jazzy; повторный вызов подключается к работающему
./run_yolo.sh     # yolo:latest
```

Собрать образ ноутбука, если его ещё нет:

```bash
docker build --network host -f docker/Dockerfile -t nav2:jazzy .
```

Запускать именно из корня: контекст сборки `.` нужен, чтобы `COPY` нашёл
`docker/entrypoint.sh`. Первый раз — минут десять.

## 3.2 Навигация

Внутри контейнера `nav2`:

```bash
colcon build --packages-select maze_nav && source install/setup.zsh
ros2 launch maze_nav city.launch.py
```

Поднимается всё сразу: Nav2 по карте полигона с AMCL, RViz,
`route_follower` и окно редактора с графом и кнопкой «Старт».

```bash
ros2 launch maze_nav city.launch.py rviz:=false      # без RViz
ros2 launch maze_nav city.launch.py editor:=false    # без окна редактора
ros2 launch maze_nav city.launch.py map:=/root/ros_ws/src/maze_nav/maps/city.yaml
ros2 launch maze_nav city.launch.py params_file:=/путь/к/своему.yaml
```

`colcon build` нужен только после добавления файлов или правки
`CMakeLists.txt`: сборка идёт с `--symlink-install`, и правки в
существующих скриптах подхватываются сразу.

Только стек Nav2, без интерфейса и исполнителя:

```bash
ros2 launch maze_nav nav2_frob.launch.py
```

## 3.3 Детектор знаков

В контейнере `yolo`:

```bash
python3 /root/ros_ws/src/yolo/sign_detector.py --ros-args \
  -p stream_url:=http://<IP робота>:8080/stream \
  -p device:=cuda:0
```

Детектор молчит, пока его не спросят. Запрос — из любого контейнера
того же домена:

```bash
ros2 service call /detect_signs std_srvs/srv/Trigger
```

```
success=True, message='no_right (класс 5, уверенность 0.98, в 5 из 5 кадров, 0.58 с)'
```

---

# 4. Режим Б: всё на роботе

## 4.1 Собрать на ноутбуке

```bash
./docker/robot/build.sh                # оба образа
./docker/robot/build.sh --only nav2    # только навигация
./docker/robot/build.sh --only yolo    # только детектор
./docker/robot/build.sh --sync         # только обновить копию src/
```

Сборка идёт через эмуляцию qemu и занимает десятки минут: почти всё время
уходит на `apt` и `pip` внутри arm64. На самой плате было бы в разы дольше.

Результат складывается в ту же папку: образы архивами, копия `src/`,
`BUILD_INFO` с датой и коммитом.

## 4.2 Доставить

```bash
./docker/robot/build.sh --send pi@<IP робота>
```

или руками — папка самодостаточна:

```bash
rsync -a --partial --progress docker/robot/ pi@<IP робота>:nav2_robot/
```

`--partial` не для красоты: архивы весят гигабайты, и при обрыве Wi-Fi
передача продолжится, а не начнётся заново.

## 4.3 Запустить

```bash
ssh -X pi@<IP робота>          # -X обязателен, иначе окон не будет
cd ~/nav2_robot
./run.sh
```

Первый запуск сам сделает `docker load` из архива — пара минут. Внутри:

```bash
colcon build --packages-select maze_nav && source install/setup.bash
ros2 launch maze_nav city.launch.py
```

Детектор знаков — вторым терминалом, отдельным контейнером:

```bash
ssh pi@<IP робота>
cd ~/nav2_robot && ./run_yolo.sh
```

Кадры берутся прямо с `/dev/video0`. Поэтому **`sign_camera.py`
одновременно работать не может**: камера отдаётся одному процессу, второй
получит `Device or resource busy`.

Без графики, если ноутбук не нужен вовсе:

```bash
ros2 launch maze_nav city.launch.py rviz:=false editor:=false
```

## 4.4 Правка конфига без пересборки

```bash
scp src/maze_nav/params/frob_params.yaml \
    pi@<IP>:nav2_robot/src/maze_nav/params/
```

Образ при этом не трогается: код монтируется с хоста. Узлы читают
параметры при старте, поэтому launch надо перезапустить.

---

# 5. Редактор маршрута

Работает сам по себе, без робота и Nav2 — удобно репетировать заезд.

```bash
ros2 run maze_nav route_editor.py \
  --map   /root/ros_ws/src/maze_nav/maps/city.yaml \
  --graph /root/ros_ws/src/maze_nav/maps/city_routes.yaml
```

Клавиши работают в любой раскладке:

| Клавиша | Действие |
|---|---|
| `G` | назначить цель под курсором |
| `P` | поставить робота в точку под курсором |
| `M` | расставить знаки: 8 направляющих, 2 остановки, 1 стоянка |
| `1` `2` `3` | повесить знак вручную: нет налево / направо / прямо |
| `T` | переключить «стоп» у точки |
| `0` | снять все знаки |
| пробел | старт / пауза проезда |
| `Backspace` | сброс проезда, знаки снова нераспознанные |
| `I` | показать номера точек |
| `U` | пересчитать нумерацию |
| `R` | перегенерировать граф из регламента |
| `Ctrl+S` | сохранить граф |

Знаки серые, пока робот до них не доехал; после распознавания загораются:
красный ободок — запрет, синий — предписание. У места остановки робот
стоит 2 секунды, у места стоянки заезд завершается.

---

# 6. Куда смотреть

| Что | Где |
|---|---|
| кадр с рамками | файл `src/yolo/last_detection.jpg` |
| он же в RViz | Add → By topic → `/sign_image` |
| имя знака | топик `/sign_detected` (`std_msgs/String`) |
| подробности детекции | `/sign_detections` — JSON с классами, уверенностью, рамками |
| карта, скан, план, курс робота | RViz |
| проезд по графу | окно редактора: пройденные точки зеленеют, цель синяя |

---

# 7. Диагностика

| Симптом | Команда |
|---|---|
| контейнер не видит робота | `ros2 node list` — ждём `sllidar_node`, `ekf_filter_node`, `arduino_bridge` |
| нет TF до робота | `ros2 run tf2_ros tf2_echo map base_footprint` |
| положение лидара | `ros2 run tf2_ros tf2_echo robot_base lidar` → ждём `0.080, 0.000, 0.020` |
| сектор обзора лидара | `ros2 run maze_nav scan_fov.py --topic /scan --count 50` |
| что видит лидар по секторам | `ros2 run maze_nav scan_sectors.py` |
| частота данных | `ros2 topic hz /scan/filtered` · `/odom` · `/odometry/filtered` |
| узлы Nav2 не активны | `ros2 lifecycle get /amcl` и далее, ждём `active [3]` |
| робот крутится, но не едет | проверить часы обеих машин и что лидар вообще подключён: `ssh pi@<IP> lsusb \| grep CP210` |
| робот не реагирует | закрыт ли `teleop_twist_keyboard` — он публикует в `/cmd_vel` наравне с навигацией |
| сверить одометрию | `ros2 run maze_nav odom_compare.py`, проехать метр, Ctrl+C |
| сверить угол | `ros2 run maze_nav yaw_check.py`, повернуть на 180°, Ctrl+C |
| знаки направления | `ros2 run maze_nav dir_check.py` |
| энкодеры | `ros2 run maze_nav enc_check.py --time 5` |

**Если робот крутится на месте и не едет вперёд** — сначала проверяй не
конфиг Nav2, а три вещи: подключён ли лидар физически, сходятся ли часы,
и не ушёл ли ноутбук в другую Wi-Fi сеть. Все три раза, когда мы это
ловили, виноват был не Nav2.

---

# 8. Отдельные задачи

**Езда с клавиатуры:**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Сбор карты помещения:**

```bash
ros2 launch maze_nav frob_slam.launch.py
# поездить телеопом, затем сохранить:
ros2 run nav2_map_server map_saver_cli -f /root/ros_ws/src/maze_nav/maps/<имя>
```

**Карта полигона** снимать лидаром не нужно — она строится из регламента:

```bash
ros2 run maze_nav gen_city_map.py
```

**Калибровка одометрии:**

```bash
ros2 run maze_nav calib_odom.py --distance 1.6   # проехать 1.6 м руками
ros2 run maze_nav calib_odom.py --angle 360      # прокрутить на месте
```

Коэффициенты применяются на роботе в
`src/frob_odometry/config/odometry_params.yaml`; после правки нужны
`colcon build --packages-select frob_odometry` и перезапуск bringup —
параметры читаются один раз при старте.

**Симулятор без робота:**

```bash
ros2 launch maze_nav maze.launch.py               # лабиринт + TurtleBot3
ros2 launch maze_nav maze.launch.py gui:=false    # без окна, только физика
```

---

# 9. Что помнить

**Высота лидара — между 0.030 и 0.085 м от пола.** Ниже в скан лезут
бордюры, которых нет на карте; выше постаменты 0.80 м подменяются
зданиями 0.40 м. Целиться в 0.05.

**Ноутбук должен держаться одной сети.** Дважды уходил на соседнюю точку
доступа и давал задержку в полсекунды, которую я принимал за перегрузку
канала. Закрепить приоритет автоподключения нужной сети.

**Классы модели** совпадают с Приложением Б регламента: `1` прямо,
`2` налево, `3` направо, `4` налево запрещён, `5` направо запрещён,
`6` место остановки, `7` место стоянки, `8` опасный объект — маршрут
не меняет, его и так объедет костмапа.

**Цепочка команд скорости** в Nav2 Jazzy, менять её нельзя — на ней
завязаны параметры:

```
controller_server → /cmd_vel_nav → velocity_smoother
    → /cmd_vel_smoothed → collision_monitor → /cmd_vel → arduino_bridge
```

Тип сообщения `geometry_msgs/Twist`, как и ждёт мост. Поэтому в
`frob_params.yaml` не включён `enable_stamped_cmd_vel` — в отличие от
`maze_params*.yaml`, где мост TurtleBot3 требует `TwistStamped`.

**Опорные числа робота** из URDF и `ros2_arduino_bridge`: корпус — цилиндр
R=0.11 м, база колёс 0.18 м, потолок 0.4863 м/с и 5.4 рад/с. Базовый фрейм
`base_footprint`; фрейма `base_link` у Frob нет вовсе.
