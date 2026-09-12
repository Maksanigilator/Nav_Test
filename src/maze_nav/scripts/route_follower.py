#!/usr/bin/env python3
"""Проезд по графу маршрутов: точка за точкой, с решением на каждой.

Логика цикла ровно такая, как задумано:
  1. строим маршрут по графу от текущей точки до цели;
  2. отправляем в Nav2 ОДНУ следующую точку через NavigateToPose;
  3. дождавшись приезда, смотрим знаки и пересчитываем маршрут;
  4. если впереди «стоп» — стоим положенное время;
  5. повторяем, пока не доедем до цели.

Пересчёт на каждом шаге и есть смысл затеи: увидев знак, робот меняет
план, не прерывая движения по маршруту целиком.

Запуск:
    ros2 run maze_nav route_follower.py --graph .../city_routes.yaml

Отправить цель (id точки из графа):
    ros2 topic pub --once /route_goal std_msgs/String "{data: 'n_4_4_E'}"

Распознанный знак подаётся снаружи, строкой "точка:знак":
    ros2 topic pub --once /route_sign std_msgs/String "{data: 'n_2_2_N:no_left'}"

ДОРАБОТАТЬ, когда появится распознавание знаков:
  * заменить топик /route_sign на подписку к детектору (vision_msgs) и
    самим определять, к какой точке знак относится, по позе робота;
  * сейчас знак действует бессрочно — см. reset_signs() в route_graph;
  * действие «стоп» — простая пауза; для полигона может требоваться
    выдержка именно на стоп-линии, а не в центре полосы;
  * при недостижимости цели узел сдаётся; разумно добавить объезд или
    повторную попытку с другой полосы.
"""
import argparse
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from maze_nav.route_graph import RouteGraph, DIR_NAME, NAME_DIR


class RouteFollower(Node):
    def __init__(self, graph_path, stop_seconds):
        super().__init__('route_follower')
        self.g = RouteGraph.load(graph_path)
        self.stop_seconds = stop_seconds
        self.goal_id = None
        self.busy = False

        # Реентрантная группа: маршрут крутится в своём потоке и ждёт
        # результата от Nav2, пока исполнитель продолжает обрабатывать
        # остальные callbacks. Без этого вложенный spin падает с
        # "Executor is already spinning".
        self.cb_group = ReentrantCallbackGroup()
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                callback_group=self.cb_group)
        self.tf_buf = Buffer()
        self.tf_lis = TransformListener(self.tf_buf, self)

        self.create_subscription(String, '/route_goal', self.on_goal, 10)
        self.create_subscription(String, '/route_sign', self.on_sign, 10)
        self.create_subscription(String, '/route_start', self.on_start, 10)
        self.pub_state = self.create_publisher(String, '/route_state', 10)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped,
                                              '/initialpose', 10)
        # Рёбра, по которым уже проехали: нужны в режиме исследования, чтобы
        # предпочитать новые направления.
        self.passed_edges = set()
        self.visits = {}

        self.get_logger().info(
            f"граф: {len(self.g.nodes)} точек, {len(self.g.edges)} рёбер. "
            f"цель слать в /route_goal, знаки в /route_sign")

    # ---------------- входные топики ----------------
    def on_goal(self, msg):
        if self.busy:
            self.get_logger().warn("маршрут уже выполняется, новая цель отклонена")
            return
        if msg.data not in self.g.nodes:
            self.get_logger().error(f"нет такой точки: {msg.data}")
            return
        self.goal_id = msg.data
        self.get_logger().info(f"цель: {self.goal_id}")
        # редактор подсветит цель синим и сбросит прошлую подсветку
        self._state('reset')
        self._state(f'goal:{self.goal_id}')
        # выполняем в отдельном потоке исполнителя, чтобы не блокировать callbacks
        self._launch()

    def _launch(self):
        """Маршрут исполняется в отдельном потоке.

        Внутри цикла мы ждём завершения целей Nav2, а это блокирующее
        ожидание. В callback исполнителя его делать нельзя.
        """
        t = threading.Thread(target=self.run_route, daemon=True)
        t.start()

    def on_start(self, msg):
        """Команда старта. Пустая строка — исследование, иначе id цели.

        Перед выездом публикуем начальную позу в точке 1: робота ставят на
        полигон у въезда, и AMCL без подсказки может не сойтись — стены
        полигона одинаковы со всех сторон.
        """
        if self.busy:
            self.get_logger().warn("маршрут уже выполняется")
            return
        self.goal_id = msg.data.strip() or None
        self.passed_edges.clear()
        self.visits.clear()
        self.set_initial_pose()
        self._state('reset')
        if self.goal_id:
            self._state(f'goal:{self.goal_id}')
            self.get_logger().info(f"старт, цель {self.goal_id}")
        else:
            self.get_logger().info("старт в режиме исследования (цели нет)")
        self._launch()

    def set_initial_pose(self, node_id: str | None = None):
        """Сказать AMCL, где робот. По умолчанию — точка номер 1."""
        if node_id is None:
            node_id = min(self.g.nodes, key=lambda i: self.g.nodes[i].index)
        n = self.g.nodes[node_id]
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x = n.x
        m.pose.pose.position.y = n.y
        m.pose.pose.orientation.z = math.sin(n.yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(n.yaw / 2.0)
        # разброс по умолчанию: позиция +-0.25 м, курс +-0.25 рад
        m.pose.covariance[0] = m.pose.covariance[7] = 0.25 ** 2
        m.pose.covariance[35] = 0.25 ** 2
        self.pub_init.publish(m)
        self.get_logger().info(
            f"начальная поза: {node_id} (№{n.index}) ({n.x:+.2f}, {n.y:+.2f})")

    def explore_next(self, nid):
        """Куда ехать без цели: сначала непройденные рёбра, прямо важнее поворотов."""
        order = {'straight': 0, 'right': 1, 'left': 2}
        outs = [(b, e) for (a, b), e in self.g.edges.items()
                if a == nid and e.enabled]
        if not outs:
            return None
        fresh = [(b, e) for b, e in outs if (nid, b) not in self.passed_edges]
        pool = fresh or outs
        if fresh:
            return min(pool, key=lambda p: order.get(p[1].kind, 3))[0]
        return min(pool, key=lambda p: (self.visits.get(p[0], 0),
                                        order.get(p[1].kind, 3)))[0]

    def on_sign(self, msg):
        """Формат 'точка:знак', например 'n_2_2_N:no_left'."""
        try:
            nid, sign = msg.data.split(':')
        except ValueError:
            self.get_logger().error(f"ожидался формат 'точка:знак', пришло: {msg.data}")
            return
        changed = self.g.apply_sign(nid, sign)
        self.get_logger().info(f"знак {sign} в {nid}: изменено {len(changed)} рёбер")

    # ---------------- где робот сейчас ----------------
    def current_node(self):
        """Ближайшая точка графа к текущей позе робота, с учётом его курса."""
        try:
            tr = self.tf_buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"нет TF map->base_footprint: {e}")
            return None
        x = tr.transform.translation.x
        y = tr.transform.translation.y
        q = tr.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z), 1 - 2 * q.z * q.z)

        # направление, ближайшее к текущему курсу: выбираем ту точку, чей yaw
        # ближе — иначе робот может «зацепиться» за встречную полосу
        best, bd = None, float('inf')
        for nid, n in self.g.nodes.items():
            d = math.hypot(n.x - x, n.y - y)
            da = abs(math.atan2(math.sin(n.yaw - yaw), math.cos(n.yaw - yaw)))
            score = d + 0.3 * da          # 0.3 м на радиан — вес курса
            if score < bd:
                best, bd = nid, score
        return best

    # ---------------- основной цикл ----------------
    def run_route(self):
        self.busy = True
        try:
            # Nav2 поднимается небыстро: вся цепочка от map_server до
            # bt_navigator занимает под минуту. Если нажать «Старт» раньше,
            # цель просто отклонят — поэтому ждём сервер, а не падаем.
            self.get_logger().info("жду готовности Nav2...")
            if not self.nav.wait_for_server(timeout_sec=60.0):
                self.get_logger().error(
                    "Nav2 (navigate_to_pose) не поднялся за 60 с — "
                    "проверь, активированы ли узлы")
                self._state('failed')
                return
            self.get_logger().info("Nav2 готов, поехали")

            while rclpy.ok():
                cur = self.current_node()
                if cur is None:
                    self.get_logger().error("не понять, где робот — прерываю")
                    return
                if self.goal_id and cur == self.goal_id:
                    self.get_logger().info("цель достигнута")
                    self._state('done')
                    return

                if self.goal_id:
                    path = self.g.find_path(cur, self.goal_id)
                    if not path or len(path) < 2:
                        self.get_logger().error(
                            f"маршрут {cur} -> {self.goal_id} не найден")
                        self._state('unreachable')
                        return
                    nxt = path[1]
                    left = f"осталось точек: {len(path) - 1}"
                else:
                    # Цели пока нет — её увидит камера. До тех пор просто
                    # катаемся, предпочитая неизведанные направления.
                    nxt = self.explore_next(cur)
                    if nxt is None:
                        self.get_logger().error(f"из {cur} ехать некуда")
                        self._state('failed')
                        return
                    left = f"пройдено рёбер: {len(self.passed_edges)}"

                self.passed_edges.add((cur, nxt))
                self.visits[nxt] = self.visits.get(nxt, 0) + 1
                self.get_logger().info(f"{cur} -> {nxt}   ({left})")
                self._state(f"{cur}->{nxt}")

                if not self.goto(self.g.nodes[nxt]):
                    self.get_logger().error(f"Nav2 не довёл до {nxt}")
                    self._state('failed')
                    return

                if self.g.nodes[nxt].stop:
                    self.get_logger().info(f"знак «стоп» в {nxt}: стою {self.stop_seconds} с")
                    time.sleep(self.stop_seconds)
        finally:
            self.busy = False

    def goto(self, wp) -> bool:
        """Отправить одну точку в Nav2 и дождаться результата.

        Ожидание — через threading.Event, который взводится в callback.
        Вложенный spin_until_future_complete здесь недопустим: этот метод
        зовётся из рабочего потока, а исполнитель уже крутится в основном.
        """
        goal = NavigateToPose.Goal()
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = wp.x
        p.pose.position.y = wp.y
        p.pose.orientation.z = math.sin(wp.yaw / 2.0)
        p.pose.orientation.w = math.cos(wp.yaw / 2.0)
        goal.pose = p

        send = self.nav.send_goal_async(goal)
        if not self._wait(send, 10.0):
            self.get_logger().error("Nav2 не принял цель за 10 с")
            return False
        handle = send.result()
        if not handle or not handle.accepted:
            self.get_logger().error("Nav2 отклонил цель")
            return False

        res = handle.get_result_async()
        # Одна точка — это соседняя клетка, 0.8 м. Минуты с запасом хватает
        # даже на объезд и recovery-поведения.
        if not self._wait(res, 120.0):
            self.get_logger().error("Nav2 не ответил за 120 с, отменяю цель")
            handle.cancel_goal_async()
            return False
        result = res.result()
        return result is not None and result.status == 4      # SUCCEEDED

    @staticmethod
    def _wait(future, timeout):
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        return done.wait(timeout)

    def _state(self, text):
        m = String()
        m.data = text
        self.pub_state.publish(m)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--graph', required=True, help='yaml графа маршрутов')
    p.add_argument('--stop-seconds', type=float, default=3.0,
                   help='сколько стоять у знака «стоп» (по умолчанию 3)')
    a, unknown = p.parse_known_args()

    rclpy.init()
    node = RouteFollower(a.graph, a.stop_seconds)
    # Многопоточный: рабочий поток маршрута ждёт результатов Nav2, а
    # исполнитель тем временем продолжает принимать команды и TF.
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
