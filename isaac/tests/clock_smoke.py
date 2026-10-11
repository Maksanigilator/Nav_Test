#!/usr/bin/env python3
"""Тест 2: Isaac Sim публикует /clock, контейнер его видит.

Что именно проверяется — и почему это отдельный тест, а не часть сцены.
Тест 1 (isaac/tests/dds_probe.py) доказал, что транспорт между хостом
и контейнером работает, но пользовался библиотеками ROS 2 в обход Isaac:
голый python грузил их из потрохов пакета isaacsim. Здесь впервые запускается
САМ симулятор, и проверяется другое:

1. ros2_bridge вообще поднялся и нашёл дистрибутив. Настройка расширения
   ros_distro по умолчанию "system_default" = «взять то, что засорсено
   в терминале». Мы сорсим isaac/env.sh, значит подхватиться должен jazzy;
2. граф OmniGraph крутится и публикует наружу;
3. время симуляции доезжает до контейнера.

Почему именно часы, а не сразу камера. Весь ROS-стек проекта поднимается
с use_sim_time:=true (см. SetParameter в launch/rgbd_sim.launch.py). Без /clock
узлы молча зависнут на ожидании трансформов, и это будет выглядеть как поломка
TF или камеры, а не как отсутствие часов. Дешевле убедиться заранее.

Сцена намеренно пустая: ни пола, ни моделей. add_default_ground_plane() тянет
ассет с сервера Nucleus, и на этом тест мог бы зависнуть по причине, к часам
отношения не имеющей. Физическая сцена, которую заводит World, сама по себе
даёт идущее время симуляции — больше для часов ничего не нужно.

Запуск. На хосте:

    source isaac/env.sh
    "$ISAAC_PYTHON" isaac/tests/clock_smoke.py

В контейнере:

    ./run.sh ros2 topic hz /clock
    ./run.sh ros2 topic echo --once /clock

Первый запуск долгий: Kit поднимается и компилирует шейдеры, это минуты,
а не зависание.

Грабля, которая тут заложена с самого начала: **публикация идёт только
во время Play.** Узлы ROS в Isaac активны лишь когда таймлайн играет —
если забыть timeline.play(), граф построится, топик объявится, а сообщений
не будет ни одного. Симптом неотличим от сломанного транспорта.
"""
import argparse

# SimulationApp обязан быть создан ДО любых импортов omni/isaacsim: он поднимает
# Kit и регистрирует расширения. Импорт omni.graph выше по файлу упадёт.
# Поэтому и argparse здесь же, до него, а не в main().
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--gui', action='store_true',
                    help='показать окно Isaac. По умолчанию headless: '
                         'для часов картинка не нужна, а VRAM на этой машине '
                         'всего 10 ГБ')
parser.add_argument('--duration', type=float, default=0.0,
                    help='сколько секунд крутить, 0 = до Ctrl+C')
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({'headless': not args.gui})

import omni.graph.core as og  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

# Расширение моста не включено по умолчанию — без этой строки узлы
# isaacsim.ros2.bridge.* просто не существуют, и og.Controller.edit упадёт
# на неизвестном типе узла.
enable_extension('isaacsim.ros2.bridge')
simulation_app.update()

world = World(stage_units_in_meters=1.0)

# Граф собран ровно так же, как в штатном тесте самого расширения
# (isaacsim/ros2/bridge/tests/test_clock.py) — это надёжнее, чем по памяти.
# OnPlaybackTick вместо OnTick: первый срабатывает только во время Play,
# и часы не «идут» на остановленном симуляторе.
og.Controller.edit(
    {'graph_path': '/ClockGraph', 'evaluator_name': 'execution'},
    {
        og.Controller.Keys.CREATE_NODES: [
            ('OnPlaybackTick', 'omni.graph.action.OnPlaybackTick'),
            ('ReadSimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
            ('PublishClock', 'isaacsim.ros2.bridge.ROS2PublishClock'),
        ],
        og.Controller.Keys.CONNECT: [
            ('OnPlaybackTick.outputs:tick', 'PublishClock.inputs:execIn'),
            ('ReadSimTime.outputs:simulationTime', 'PublishClock.inputs:timeStamp'),
        ],
        og.Controller.Keys.SET_VALUES: [
            ('PublishClock.inputs:topicName', 'clock'),
        ],
    },
)

world.reset()

timeline = omni.timeline.get_timeline_interface()
timeline.play()

print('[clock_smoke] публикую /clock. Проверяй из контейнера:', flush=True)
print('[clock_smoke]     ./run.sh ros2 topic hz /clock', flush=True)

step = 0
try:
    while simulation_app.is_running():
        world.step(render=True)
        step += 1
        # Печатаем время симуляции со стороны Isaac, чтобы отличить
        # «часы не идут» от «часы идут, но не долетают».
        if step % 120 == 0:
            print(f'[clock_smoke] sim time = {world.current_time:.1f} c, '
                  f'шагов {step}', flush=True)
        if args.duration and world.current_time >= args.duration:
            break
except KeyboardInterrupt:
    pass
finally:
    timeline.stop()
    simulation_app.close()
