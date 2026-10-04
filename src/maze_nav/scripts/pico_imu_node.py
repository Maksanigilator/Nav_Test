#!/usr/bin/env python3
"""Приём потока MPU6050 с Pico и публикация sensor_msgs/Imu.

Разбирает двоичные кадры, которые шлёт прошивка `pico_imu.py` в корне
проекта, переводит отсчёты в СИ и публикует в `imu/data_raw`. Ориентацию
здесь НЕ считаем: её дело фильтра (imu_filter_madgwick), который и отдаёт
готовый `imu/data`. Так разделено не из вкуса, а потому что сырые данные
нужны отдельно — по ним видно, врёт ли датчик, а по кватерниону уже нет.

СИСТЕМА КООРДИНАТ. MPU6050 отдаёт оси в своей раскладке, ROS ждёт
правую систему с X вперёд, Y влево, Z вверх. Поворот задаётся параметром
`axes` строкой вида 'x y z' со знаками, например '-y x z'. По умолчанию
оси идут как есть: как датчик реально повёрнут на роботе, знает только
тот, кто его прикручивал.

МЕТКИ ВРЕМЕНИ. Ставить время прихода кадра нельзя: USB отдаёт данные
пачками, и выборки внутри пачки получили бы одинаковую метку, а фильтр
ориентации по ним считает шаг интегрирования. Поэтому метка строится из
счётчика микросекунд самого Pico, а к шкале ROS он привязывается
смещением. Смещение берётся как МИНИМУМ разности за скользящее окно:
задержка доставки всегда положительна, и наименьшая из наблюдённых ближе
всего к истинной.
"""
import math
import struct

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu

# Два формата кадра, и узел узнаёт их сам.
#
# Прошивка этого проекта (pico_imu.py) шлёт 20 байт: акселерометр и
# гироскоп. Но на плате может оказаться прошивка дипломного стенда — она
# шлёт 14 байт, только акселерометр, потому что там мерили вибрацию.
# Различить их просто по шагу между метками синхронизации, и это дешевле,
# чем требовать перепрошивки ради первой же проверки.
FMT_FULL = '<2sHI6h'            # sync seq t_us ax ay az gx gy gz
FMT_ACCEL = '<2sHI3h'           # sync seq t_us ax ay az
LEN_FULL = struct.calcsize(FMT_FULL)
LEN_ACCEL = struct.calcsize(FMT_ACCEL)
SYNC = b'\xAA\x55'

G = 9.80665


class PicoImu(Node):
    def __init__(self):
        super().__init__('pico_imu')
        p = self.declare_parameter
        p('port', '/dev/ttyACM0')
        p('frame_id', 'imu_link')
        p('topic', 'imu/data_raw')
        # Чувствительность зависит от шкалы, заданной в прошивке. У нашей
        # это +-4 g и +-500 град/с; у прошивки стенда было +-2 g, поэтому
        # при 'auto' значения подставляются под опознанный формат кадра.
        p('accel_lsb_per_g', 0.0)   # 0 = выбрать по формату кадра
        p('gyro_lsb_per_dps', 65.5)
        p('frame', 'auto')          # auto | full | accel
        p('axes', 'x y z')
        # Оценка шума для ковариаций. Взято из документации: плотность шума
        # 400 мкg/sqrt(Гц) у акселерометра и 0.005 град/с/sqrt(Гц) у
        # гироскопа, при полосе 44 Гц это примерно 2.7 мg и 0.033 град/с.
        p('accel_stddev', 0.027)
        p('gyro_stddev', 0.00058)
        # Окно, по которому берётся минимум задержки, с.
        p('clock_window', 10.0)
        # Смещения нуля, вычитаются ПОСЛЕ разворота осей, то есть заданы
        # уже в осях ROS. Так измеренное помощником imu_bias.py число
        # подставляется сюда буквально, без пересчёта.
        #
        # Смещение гироскопа — главная беда дешёвой ИНС: оно интегрируется
        # в курс и уводит его на градусы в минуту. Смещение акселерометра
        # безобиднее, оно даёт постоянный завал горизонта.
        p('gyro_bias', [0.0, 0.0, 0.0])
        p('accel_bias', [0.0, 0.0, 0.0])
        # Верхняя частота публикации, Гц. 0 = публиковать всё.
        #
        # Прошивка стенда шлёт 1 кГц: столько нужно было для резонансов, а
        # одометрии хватает двух сотен. Каждое сообщение в rclpy стоит
        # заметно дороже, чем разбор кадра, поэтому лишние выборки бьют по
        # процессору сильнее, чем сам приём. Наша прошивка и так даёт
        # 200 Гц, так что предел срабатывает только на чужой.
        p('max_rate', 200.0)
        # ── Защита от накопления отставания ──
        #
        # Порт читается неблокирующим чтением по таймеру, и пока узел один,
        # буфер пуст. Но когда на том же ноуте идёт весь стек — камера,
        # RTAB-Map, Nav2, детектор края, — таймеру перестаёт хватать
        # процессорного времени, и выборки копятся в буфере ядра.
        #
        # Беда в том, что метки у накопленных выборок ЧЕСТНЫЕ: прошивка
        # ставит время съёма, а не время выдачи. Поэтому отставание не
        # размывается, а целиком переходит в метки, и rgbd_odometry
        # начинает ругаться, что ИНС не новее кадра, и бросать кадры.
        # Наблюдалось именно так: ИНС отставала примерно на 40 мс, и
        # одометрия теряла каждый кадр подряд.
        #
        # Старые выборки одометрии не нужны — ей нужна СВЕЖАЯ ориентация.
        # Поэтому всё, что старше этого окна, выбрасываем, не разбирая.
        p('backlog_max_s', 0.05)

        self.port = self.get_parameter('port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.a_lsb = float(self.get_parameter('accel_lsb_per_g').value)
        self.g_scale = (math.pi / 180.0
                        / float(self.get_parameter('gyro_lsb_per_dps').value))
        self.buf = bytearray()
        self.dropped_old = 0
        self.in_hz = 200.0
        self.flen = LEN_FULL
        self.has_gyro = True
        self.a_scale = G / 8192.0
        self.fmt = None
        want = self.get_parameter('frame').value
        if want == 'full':
            self._set_frame(LEN_FULL)
        elif want == 'accel':
            self._set_frame(LEN_ACCEL)
        self.perm, self.sign = self._parse_axes(
            self.get_parameter('axes').value)

        av = float(self.get_parameter('accel_stddev').value) ** 2
        gv = float(self.get_parameter('gyro_stddev').value) ** 2
        self.gbias = list(self.get_parameter('gyro_bias').value)
        self.abias = list(self.get_parameter('accel_bias').value)
        if any(self.gbias) or any(self.abias):
            self.get_logger().info(
                f'вычитаю смещения: гироскоп {self.gbias}, '
                f'акселерометр {self.abias}')
        self.acov = [av, 0.0, 0.0, 0.0, av, 0.0, 0.0, 0.0, av]
        self.gcov = [gv, 0.0, 0.0, 0.0, gv, 0.0, 0.0, 0.0, gv]

        self.pub = self.create_publisher(
            Imu, self.get_parameter('topic').value,
            QoSPresetProfiles.SENSOR_DATA.value)

        self.ser = None
        self.buckets = [[-1, 1 << 62]]   # [секунда, минимум задержки]
        self.bucket_sec = -1
        self.last_seq = None
        self.lost = 0
        self.count = 0
        self.skipped = 0
        mr = float(self.get_parameter('max_rate').value)
        self.max_rate = mr
        self.min_dt_us = int(1e6 / mr) if mr > 0 else 0
        self.seen = 0
        self.stride = 1
        self.rate_seen = 0
        self.rate_t0 = None
        self.open_port()
        # Читаем часто и малыми порциями: при 200 Гц кадр приходит каждые
        # 5 мс, и опрос раз в 2 мс держит задержку ниже периода выборки.
        self.create_timer(0.002, self.poll)
        self.create_timer(5.0, self.report)

    def _set_frame(self, length):
        self.flen = length
        self.fmt = FMT_FULL if length == LEN_FULL else FMT_ACCEL
        self.has_gyro = length == LEN_FULL
        # Шкала акселерометра: если её не задали явно, берём ту, что стоит
        # в соответствующей прошивке.
        lsb = self.a_lsb or (8192.0 if self.has_gyro else 16384.0)
        self.a_scale = G / lsb
        self.get_logger().info(
            f'кадр {length} байт: '
            + ('акселерометр и гироскоп' if self.has_gyro
               else 'ТОЛЬКО акселерометр (прошивка стенда, '
                    'гироскопа в потоке нет)')
            + f', шкала {lsb:.0f} отсчётов на g')

    def _detect_frame(self):
        """Шаг между метками синхронизации и есть длина кадра."""
        pos = []
        start = 0
        while len(pos) < 4:
            i = self.buf.find(SYNC, start)
            if i < 0:
                return
            pos.append(i)
            start = i + 1
        steps = [pos[i + 1] - pos[i] for i in range(len(pos) - 1)]
        for cand in (LEN_FULL, LEN_ACCEL):
            # Метка 0xAA55 может случайно встретиться и внутри данных,
            # поэтому требуем, чтобы шаг был кратен длине кадра, а не
            # просто ей равен.
            if all(st % cand == 0 for st in steps):
                self._set_frame(cand)
                return

    @staticmethod
    def _parse_axes(spec):
        """'-y x z' -> (перестановка, знаки)."""
        idx = {'x': 0, 'y': 1, 'z': 2}
        perm, sign = [], []
        for tok in spec.split():
            s = -1.0 if tok.startswith('-') else 1.0
            perm.append(idx[tok.lstrip('+-')])
            sign.append(s)
        if sorted(perm) != [0, 1, 2]:
            raise ValueError(f'в axes должны быть все три оси: {spec!r}')
        return perm, sign

    def open_port(self):
        try:
            # timeout=0 — неблокирующее чтение: узел не должен замирать на
            # порту, иначе таймеры перестают срабатывать.
            self.ser = serial.Serial(self.port, timeout=0)
            self.get_logger().info(f'порт {self.port} открыт')
        except (serial.SerialException, OSError) as e:
            self.ser = None
            self.get_logger().error(f'не открыть {self.port}: {e}',
                                    throttle_duration_sec=5.0)

    def now(self):
        return self.get_clock().now().nanoseconds

    def stamp_ns(self, t_us):
        """Метка ROS для выборки с меткой Pico t_us.

        Минимум задержки считается ПО КОРЗИНАМ, а не по всем выборкам.
        Сначала я хранил список всех разностей за окно и брал минимум по
        нему на каждой выборке. При потоке 1 кГц и окне 10 с это десять
        тысяч записей, по которым на каждую выборку шёл и проход фильтром,
        и поиск минимума, — то есть порядка десяти миллионов действий в
        секунду. Узел съедал три четверти ядра на ровном месте, замерено.

        Корзина в одну секунду хранит только свой минимум, и их всего
        десяток. Обновление корзины — одно сравнение, общий минимум — по
        десяти числам.
        """
        now = self.now()
        d = now - t_us * 1000
        sec = now // 1_000_000_000
        if self.bucket_sec != sec:
            self.bucket_sec = sec
            self.buckets.append([sec, d])
            keep = int(float(self.get_parameter('clock_window').value))
            del self.buckets[:-max(2, keep)]
        elif d < self.buckets[-1][1]:
            self.buckets[-1][1] = d
        return t_us * 1000 + min(b[1] for b in self.buckets)

    def poll(self):
        if self.ser is None:
            self.open_port()
            return
        try:
            chunk = self.ser.read(4096)
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f'чтение оборвалось: {e}')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            return
        if not chunk:
            return
        self.buf.extend(chunk)

        # Отставание срезаем ДО разбора: смысла тратить процессор на
        # выборки, которые и так опоздали, нет — а именно нехватка
        # процессора это отставание и создала.
        if self.fmt is not None:
            keep = int(float(self.get_parameter('backlog_max_s').value)
                       * max(1.0, self.in_hz) * self.flen)
            if keep > 0 and len(self.buf) > keep:
                self.dropped_old += (len(self.buf) - keep) // self.flen
                del self.buf[:len(self.buf) - keep]

        if self.fmt is None:
            self._detect_frame()
            if self.fmt is None:
                if len(self.buf) > 4096:
                    del self.buf[:2048]
                return

        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                # Метка не найдена вовсе: держим хвост на случай, если она
                # разорвана между порциями.
                del self.buf[:max(0, len(self.buf) - 1)]
                return
            if i:
                del self.buf[:i]
            if len(self.buf) < self.flen:
                return
            frame = bytes(self.buf[:self.flen])
            del self.buf[:self.flen]
            self.emit(struct.unpack(self.fmt, frame))

    def emit(self, f):
        if self.has_gyro:
            _, seq, t_us, ax, ay, az, gx, gy, gz = f
        else:
            _, seq, t_us, ax, ay, az = f
            gx = gy = gz = 0
        if self.last_seq is not None:
            gap = (seq - self.last_seq - 1) & 0xFFFF
            if gap:
                self.lost += gap
        self.last_seq = seq

        # Прореживаем ПО СЧЁТУ выборок, а не по меткам времени.
        #
        # По меткам было бы естественнее, но прошивка стенда ставит одну
        # метку на всю пачку, и тогда из пачки проходит ровно одна
        # выборка независимо от её длины — замерено, вместо 200 Гц
        # получалось 173. Счёт от длины пачки не зависит.
        #
        # Шаг берётся из наблюдаемой частоты потока и пересчитывается раз
        # в секунду: частота задана прошивкой, а какая на плате лежит,
        # узел заранее не знает.
        self.seen += 1
        if self.min_dt_us:
            if self.stride > 1 and self.seen % self.stride:
                self.skipped += 1
                return
        self.count += 1
        self._update_stride()

        a = [ax * self.a_scale, ay * self.a_scale, az * self.a_scale]
        g = [gx * self.g_scale, gy * self.g_scale, gz * self.g_scale]

        m = Imu()
        ns = self.stamp_ns(t_us)
        m.header.stamp.sec = ns // 1_000_000_000
        m.header.stamp.nanosec = ns % 1_000_000_000
        m.header.frame_id = self.frame_id
        m.linear_acceleration.x = (self.sign[0] * a[self.perm[0]]
                                   - self.abias[0])
        m.linear_acceleration.y = (self.sign[1] * a[self.perm[1]]
                                   - self.abias[1])
        m.linear_acceleration.z = (self.sign[2] * a[self.perm[2]]
                                   - self.abias[2])
        m.angular_velocity.x = self.sign[0] * g[self.perm[0]] - self.gbias[0]
        m.angular_velocity.y = self.sign[1] * g[self.perm[1]] - self.gbias[1]
        m.angular_velocity.z = self.sign[2] * g[self.perm[2]] - self.gbias[2]
        m.linear_acceleration_covariance = self.acov
        # Без гироскопа честно помечаем угловую скорость как отсутствующую,
        # а не выдаём нули за измерение: фильтр ориентации на нулях решил
        # бы, что датчик неподвижен, и намертво привязал бы курс.
        m.angular_velocity_covariance = (
            self.gcov if self.has_gyro
            else [-1.0] + [0.0] * 8)
        # Ориентацию не даём. Минус единица в первом элементе — это
        # соглашение ROS: «поле не заполнено», и потребитель не примет
        # нули за настоящий кватернион.
        m.orientation_covariance[0] = -1.0
        self.pub.publish(m)

    def _update_stride(self):
        """Раз в секунду пересчитывает, через сколько выборок публиковать."""
        now = self.now()
        if self.rate_t0 is None:
            self.rate_t0 = now
            return
        self.rate_seen += 1
        if now - self.rate_t0 < 1_000_000_000:
            return
        # Наблюдаемая частота ПУБЛИКАЦИИ, умноженная на текущий шаг, даёт
        # частоту входного потока.
        out_hz = self.rate_seen * 1e9 / (now - self.rate_t0)
        in_hz = out_hz * self.stride
        self.in_hz = in_hz if in_hz > 1.0 else self.in_hz
        self.rate_t0, self.rate_seen = now, 0
        if self.max_rate > 0 and in_hz > 0:
            st = max(1, int(round(in_hz / self.max_rate)))
            if st != self.stride:
                self.get_logger().info(
                    f'поток {in_hz:.0f} Гц, публикуем каждую {st}-ю '
                    f'выборку -> {in_hz / st:.0f} Гц')
                self.stride = st

    def report(self):
        if not self.count:
            self.get_logger().warn('кадров нет: проверь порт и прошивку')
            return
        old = (f', выброшено как устаревшие {self.dropped_old}'
               if self.dropped_old else '')
        self.get_logger().info(
            f'опубликовано {self.count}, прорежено {self.skipped}, '
            f'потеряно {self.lost}{old}')
        self.count = self.lost = self.skipped = self.dropped_old = 0


def main():
    rclpy.init()
    n = PicoImu()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()


if __name__ == '__main__':
    main()
