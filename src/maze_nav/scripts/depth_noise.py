#!/usr/bin/env python3
"""Приводит идеальную глубину из симулятора к поведению настоящего D435.

╔══════════════════════════════════════════════════════════════════════════╗
║  ВНИМАНИЕ: УЗЕЛ ОТКЛЮЧЁН. В launch/isaac_nav.launch.py он закомментирован.║
║  Модель верна по величине шума, но НЕВЕРНА по его временной структуре,    ║
║  и из-за этого ломает одометрию сильнее, чем ломала бы реальная камера.   ║
╚══════════════════════════════════════════════════════════════════════════╝

ЧТО РАБОТАЕТ. Величина разброса воспроизводится точно и проверена замером
против теории по всему диапазону:

    дальность   замер      теория
    0.3-0.7 м   0.23 см    0.20 см
    1.2-2.0 м   2.51 см    2.35 см
    3.0-5.0 м   17.41 см   16.20 см
    5.0-8.0 м   31.91 см   31.38 см

Рост как z^2 получается сам, потому что шум задан в диспаратности. Это
совпадает с паспортом D435 и с литературой (~4 см СКО на 2 м).

ЧТО СЛОМАНО. Шум разыгрывается заново КАЖДЫЙ КАДР, то есть он белый по
времени. У настоящего стерео так не бывает: диспаратность считается
детерминированно по паре кадров, и неподвижная камера на неподвижной сцене
даёт ПОВТОРЯЮЩУЮСЯ глубину. Ошибка там есть, и немалая, но она привязана
к сцене и к алгоритму сопоставления, а не бросается заново каждый раз.
Временная изменчивость у реальной камеры берётся только от шума матрицы
и при хорошем свете мала.

Почему это важно именно здесь. Одометрия работает методом PnP 3D->2D
(Vis/EstimationType=1) по НАКОПЛЕННОЙ локальной карте признаков
(OdometryF2M, frame-to-map). Если у одной и той же физической точки
координаты прыгают от кадра к кадру, карта размазывается и PnP по ней
не сходится.

Замерено на прогоне в офисе (Isaac, склад офисной сцены, ~2200 замеров):

    потерь одометрии            15
    при отказах инлайеров       4.7 из 71 совпадения (7%)

Семьдесят признаков сопоставляются прекрасно — рушится именно геометрия,
и это подпись разъехавшихся 3D-координат, а не нехватки текстуры.

ЧТО НАДО ПОЧИНИТЬ ПЕРЕД ВОЗВРАТОМ:

1. Сделать шум КОРРЕЛИРОВАННЫМ ПО ВРЕМЕНИ: держать поле шума и обновлять
   медленно, а не разыгрывать на каждый кадр. Тогда неподвижная сцена даст
   стабильную, пусть и смещённую, глубину — как у настоящей камеры.
2. Добавить ПРОСТРАНСТВЕННУЮ корреляцию: у реального матчера ошибка идёт
   пятнами по областям сопоставления, а не попиксельно независимо.
3. Откалибровать порог текстуры. Сейчас он взят на глаз (6.0) и даёт 52%
   невалидных пикселей, что вероятно пессимистично. Калибровать есть обо
   что: на реальной записи с рельсами измерено 88% валидных.
4. Заодно подумать про Vis/MaxDepth. Сейчас он 0, то есть без предела,
   и RTAB-Map берёт признаки с восьми метров, где разброс ±30 см — для
   порога Vis/PnPReprojError=2 пикселя это мусор в любом случае.

Отдельно: узел добавляет заметную нагрузку. Глубина начинает ходить по
проводу ДВАЖДЫ (идеальная из сцены и зашумлённая отсюда), это +78 МБ/с,
плюс генерация 407 тысяч гауссовых отсчётов и градиент на каждый кадр.
На прогоне это давало отставание выдачи на 7% (48.4 Гц против 52 на входе)
и рост задержки до 0.37 с. Перед возвратом стоит либо притормозить
симулятор до реального времени, либо считать шум внутри сцены.

───────────────────────────────────────────────────────────────────────────


Почему не «просто шум». Глубина у стереокамеры считается как z = f*B/d,
где d — диспаратность в пикселях. Ошибка возникает НЕ в метрах, а в пикселях
диспаратности, при сопоставлении левого и правого кадра. Из геометрии сразу
следует, что ошибка глубины растёт как z^2:

    dz = z^2 * dd / (f * B)

Поэтому шум здесь добавляется в диспаратности, а не в метрах, и рост ошибки
с дальностью получается сам, без единого подогнанного коэффициента.

Все параметры взяты из паспорта камеры и наших же интринсик:

    база 50 мм            паспорт Intel D435
    фокус 612.31 пикс     наш, сверен с полем k в camera_info
    sigma 0.306 пикс      решение обратной задачи под известные из литературы
                          ~4 см СКО на 2 м (Ahn и др., анализ шума D435)

Что это даёт по дальностям: 1 см на 1 м, 4 см на 2 м, 16 см на 4 м, 1 м на 10 м.
Последнее и есть причина, по которой глубине D435 вдаль доверять нельзя.

Дыры ставятся по двум причинам, обе физические:

1. ЗА ПРЕДЕЛАМИ ДАЛЬНОСТИ. Ближе 0.28 м диспаратность превышает предел поиска,
   дальше 10 м она вырождается в шум. У реальной камеры там просто нет данных.
2. НЕХВАТКА ТЕКСТУРЫ. Стереосопоставление не работает на однородной
   поверхности: сопоставлять нечего.

Важная оговорка про второй пункт. У D435 есть ИК-проектор, который подсвечивает
сцену точечным узором и на гладкой белой стене глубину как раз ВЫТЯГИВАЕТ.
Но его дальность ограничена, а на ярком свету узор смывается. Поэтому дыры
по текстуре ставятся только дальше projector_range (по умолчанию 3 м) —
вблизи считаем, что проектор справился. Без этой оговорки симулятор был бы
заметно пессимистичнее реальности в помещении.

Чего модель не делает: она осевая. Боковой шум и зависимость от угла падения
луча на поверхность, которые есть у Ahn и др., здесь не воспроизводятся —
на сильно наклонных поверхностях реальный разброс будет больше.

    ros2 run maze_nav depth_noise.py --ros-args -p input_topic:=/camera/depth/ideal
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class DepthNoise(Node):
    def __init__(self):
        super().__init__('depth_noise')

        self.declare_parameter('input_topic', '/camera/depth/ideal')
        self.declare_parameter('output_topic', '/camera/depth/image_raw')
        self.declare_parameter('rgb_topic', '/camera/image_raw')
        self.declare_parameter('info_topic', '/camera/camera_info')

        self.declare_parameter('baseline_m', 0.050)
        # 0 означает «взять fx из camera_info». Так параметр не разъезжается
        # с реальными интринсиками камеры, что бы в них ни поменяли.
        self.declare_parameter('focal_pixel', 0.0)
        self.declare_parameter('disparity_sigma_px', 0.306)
        # Систематическое смещение диспаратности. По умолчанию 0: это уже
        # не шум, а ошибка калибровки, и мешать их в одну кучу не стоит.
        self.declare_parameter('disparity_bias_px', 0.0)
        self.declare_parameter('min_range_m', 0.28)
        self.declare_parameter('max_range_m', 10.0)
        # Ниже этого градиента яркости стерео сопоставлять нечего.
        self.declare_parameter('texture_threshold', 6.0)
        self.declare_parameter('projector_range_m', 3.0)
        self.declare_parameter('seed', 0)

        g = self.get_parameter
        self.baseline = g('baseline_m').value
        self.focal = g('focal_pixel').value
        self.sigma = g('disparity_sigma_px').value
        self.bias = g('disparity_bias_px').value
        self.zmin = g('min_range_m').value
        self.zmax = g('max_range_m').value
        self.tex_thr = g('texture_threshold').value
        self.proj = g('projector_range_m').value

        seed = g('seed').value
        self.rng = np.random.default_rng(seed if seed else None)

        qos = QoSProfile(depth=5,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST)

        self.gray = None
        self.pub = self.create_publisher(Image, g('output_topic').value, qos)
        self.create_subscription(Image, g('rgb_topic').value, self.on_rgb, qos)
        self.create_subscription(Image, g('input_topic').value, self.on_depth, qos)
        if self.focal <= 0:
            self.create_subscription(CameraInfo, g('info_topic').value,
                                     self.on_info, qos)

        self.count = 0
        self.dropped = 0.0
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f"{g('input_topic').value} -> {g('output_topic').value}, "
            f'база {self.baseline*1000:.0f} мм, sigma {self.sigma} пикс '
            f'диспаратности, дальность {self.zmin}..{self.zmax} м')

    def on_info(self, msg):
        if self.focal <= 0 and msg.k[0] > 0:
            self.focal = float(msg.k[0])
            self.get_logger().info(f'фокус из camera_info: {self.focal:.2f} пикс')

    def on_rgb(self, msg):
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
        self.gray = a[:, :, :3].mean(axis=2)

    def on_depth(self, msg):
        if self.focal <= 0:
            return  # ждём camera_info
        z = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.width)
        valid = np.isfinite(z) & (z > 0)

        fb = self.focal * self.baseline
        out = np.full(z.shape, np.nan, np.float32)

        # Шум в ДИСПАРАТНОСТИ: отсюда рост ошибки как z^2.
        d = np.divide(fb, z, out=np.zeros_like(z), where=valid)
        d = d + self.bias + self.rng.normal(0.0, self.sigma, z.shape)
        # Отрицательная или нулевая диспаратность физически невозможна.
        ok = valid & (d > 1e-6)
        out[ok] = (fb / d[ok]).astype(np.float32)

        # Границы дальности: ближе минимума диспаратность выходит за предел
        # поиска, дальше максимума вырождается в шум.
        out[~np.isfinite(out)] = np.nan
        out[(out < self.zmin) | (out > self.zmax)] = np.nan

        # Дыры по нехватке текстуры — только дальше проектора.
        if self.gray is not None and self.gray.shape == z.shape:
            gy, gx = np.gradient(self.gray)
            tex = np.abs(gx) + np.abs(gy)
            blind = (tex < self.tex_thr) & np.isfinite(out) & (out > self.proj)
            out[blind] = np.nan

        self.dropped += 1.0 - np.isfinite(out).mean()
        self.count += 1

        msg_out = Image()
        # Заголовок копируем без изменений: весь стек живёт на use_sim_time,
        # и rgbd_odometry сопоставляет глубину с цветом именно по штампу.
        msg_out.header = msg.header
        msg_out.height, msg_out.width = msg.height, msg.width
        msg_out.encoding = '32FC1'
        msg_out.is_bigendian = msg.is_bigendian
        msg_out.step = msg.width * 4
        msg_out.data = out.tobytes()
        self.pub.publish(msg_out)

    def report(self):
        if not self.count:
            self.get_logger().warn('глубина не приходит — запущена ли сцена?')
        else:
            self.get_logger().info(
                f'кадров {self.count}, невалидных пикселей в среднем '
                f'{100*self.dropped/self.count:.1f}%')


def main():
    rclpy.init()
    node = DepthNoise()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
