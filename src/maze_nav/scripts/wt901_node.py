#!/usr/bin/env python3
"""Чтение ИНС WitMotion WT901C-485 по Modbus RTU и публикация в топик.

ЧЕМ ЭТА ИНС ОТЛИЧАЕТСЯ ОТ ПРЕЖНЕЙ. Самодельная связка Pico с MPU6050
отдавала сырые отсчёты по USB своим кадром, и всё — пересчёт шкал,
смещения нуля, слияние в ориентацию — делалось на хосте. WT901C-485 это
законченный прибор: внутри девятиосевой датчик с фильтром, наружу
промышленный RS485 с Modbus RTU, и ориентация считается им самим.

ПОЧЕМУ ОН ПОКА НЕ ПОДКЛЮЧЁН К ОДОМЕТРИИ. Прибор стоит НА КРЫШЕ робота, а
не на его оси. При раскачке на подвеске крыша смещается вперёд-назад на
сантиметр-другой, и это смещение неотличимо от настоящего движения
робота. Пока не решено, как это учитывать, узел работает сам по себе и
публикует в отдельный топик: ни одометрия, ни карта его не видят.

ПРОТОКОЛ. Читается двенадцать регистров подряд с 0x34 одним запросом,
функция 0x03. Это ускорение, угловая скорость, магнитное поле и углы:

    0x34 0x35 0x36   ускорение      X Y Z
    0x37 0x38 0x39   угловая скор.  X Y Z
    0x3A 0x3B 0x3C   магнитное поле X Y Z
    0x3D 0x3E 0x3F   углы           крен, тангаж, курс

Шкалы взяты из официального SDK WitMotion (WitStandardModbus_WT901C485,
Python/device_model.py), а не из общих соображений:

    ускорение       raw / 32768 * 16     g
    угловая скор.   raw / 32768 * 2000   град/с
    углы            raw / 32768 * 180    град

Магнитное поле прибор отдаёт в СВОИХ единицах, без калибровки в теслах.
Публикуем как есть и честно ставим ковариацию -1, что по соглашению
означает «значение есть, но доверять ему как измерению нельзя».

ПРО АДРЕС И СКОРОСТЬ. Адрес на шине по умолчанию 0x50, скорость 9600.
И то и другое меняется записью в регистры, так что если прибор молчит —
первым делом перебрать скорости, узел это умеет сам (параметр probe).
"""
import math
import struct
import sys
import time

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu, MagneticField

G = 9.80665
REG_START = 0x34
REG_COUNT = 12
BAUDS = (9600, 115200, 19200, 38400, 57600, 230400, 4800)


def crc16(buf):
    """CRC Modbus RTU. Полином 0xA001, начальное значение 0xFFFF."""
    crc = 0xFFFF
    for b in buf:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def s16(v):
    return v - 65536 if v >= 32768 else v


class WT901(Node):
    def __init__(self):
        super().__init__('wt901')
        p = self.declare_parameter
        p('port', '/dev/ttyUSB0')
        p('baud', 9600)
        p('address', 0x50)
        p('frame_id', 'wt901_link')
        p('rate', 50.0)
        # Перебрать скорости, если на заданной прибор не отвечает. Скорость
        # хранится в самом приборе и могла быть изменена прошлым хозяином.
        p('probe', True)
        p('topic', 'imu/wt901/data')
        p('mag_topic', 'imu/wt901/mag')
        # Печатать разобранные значения в лог раз в N секунд. Нужно, чтобы
        # глазами убедиться в разумности чисел, не поднимая RViz.
        p('print_every', 2.0)

        self.addr = int(self.get_parameter('address').value)
        self.frame = str(self.get_parameter('frame_id').value)
        self.req = self.frame_request()
        self.ser = None
        self.ok = self.bad = 0
        self.last_print = 0.0

        qos = QoSPresetProfiles.SENSOR_DATA.value
        self.pub = self.create_publisher(
            Imu, str(self.get_parameter('topic').value), qos)
        self.mag = self.create_publisher(
            MagneticField, str(self.get_parameter('mag_topic').value), qos)

        self.open_port()
        rate = float(self.get_parameter('rate').value)
        self.create_timer(1.0 / max(1.0, rate), self.poll)
        self.create_timer(5.0, self.report)

    def frame_request(self):
        """Запрос на чтение: адрес, функция 0x03, начальный регистр, число."""
        body = struct.pack('>BBHH', self.addr, 0x03, REG_START, REG_COUNT)
        return body + struct.pack('<H', crc16(body))

    def open_port(self):
        port = str(self.get_parameter('port').value)
        bauds = ([int(self.get_parameter('baud').value)]
                 + ([b for b in BAUDS
                     if b != int(self.get_parameter('baud').value)]
                    if bool(self.get_parameter('probe').value) else []))
        for b in bauds:
            try:
                s = serial.Serial(port, b, timeout=0.15)
            except Exception as e:
                self.get_logger().error(f'не открыть {port}: {e}',
                                        throttle_duration_sec=5.0)
                return
            s.reset_input_buffer()
            s.write(self.req)
            time.sleep(0.08)
            r = s.read(64)
            if len(r) >= 5 and r[0] == self.addr and r[1] == 0x03:
                self.ser = s
                self.get_logger().info(
                    f'прибор отвечает: {port}, скорость {b}, '
                    f'адрес 0x{self.addr:02X}')
                return
            s.close()
        self.get_logger().error(
            f'на {port} никто не ответил ни на одной скорости. Проверь '
            f'переходник RS485, полярность A/B и адрес прибора.')

    def read_regs(self):
        """Один обмен. Возвращает двенадцать знаковых слов или None."""
        if self.ser is None:
            self.open_port()
            return None
        try:
            self.ser.reset_input_buffer()
            self.ser.write(self.req)
            # Ответ: адрес, функция, число байт, данные, CRC.
            want = 3 + REG_COUNT * 2 + 2
            r = self.ser.read(want)
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f'обмен оборвался: {e}',
                                   throttle_duration_sec=5.0)
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            return None
        if len(r) != want or r[0] != self.addr or r[1] != 0x03:
            self.bad += 1
            return None
        # CRC проверяем обязательно: на RS485 с длинным шлейфом помеха
        # портит отдельные байты, и без проверки это выглядело бы как
        # скачок ускорения на пустом месте.
        if crc16(r[:-2]) != struct.unpack('<H', r[-2:])[0]:
            self.bad += 1
            return None
        return [s16(v) for v in struct.unpack('>12H', r[3:-2])]

    def poll(self):
        v = self.read_regs()
        if v is None:
            return
        self.ok += 1
        acc = [x / 32768.0 * 16.0 * G for x in v[0:3]]
        gyr = [math.radians(x / 32768.0 * 2000.0) for x in v[3:6]]
        mg = v[6:9]
        ang = [math.radians(x / 32768.0 * 180.0) for x in v[9:12]]

        now = self.get_clock().now().to_msg()
        m = Imu()
        m.header.stamp, m.header.frame_id = now, self.frame
        m.linear_acceleration.x, m.linear_acceleration.y, \
            m.linear_acceleration.z = acc
        m.angular_velocity.x, m.angular_velocity.y, \
            m.angular_velocity.z = gyr
        cr, cp, cy = (math.cos(a / 2) for a in ang)
        sr, sp, sy = (math.sin(a / 2) for a in ang)
        m.orientation.w = cr * cp * cy + sr * sp * sy
        m.orientation.x = sr * cp * cy - cr * sp * sy
        m.orientation.y = cr * sp * cy + sr * cp * sy
        m.orientation.z = cr * cp * sy - sr * sp * cy
        self.pub.publish(m)

        g = MagneticField()
        g.header.stamp, g.header.frame_id = now, self.frame
        g.magnetic_field.x, g.magnetic_field.y, g.magnetic_field.z = \
            (float(x) for x in mg)
        # -1 по соглашению ROS: величина есть, но она не в теслах и не
        # калибрована, принимать её за измерение нельзя.
        g.magnetic_field_covariance[0] = -1.0
        self.mag.publish(g)

        every = float(self.get_parameter('print_every').value)
        t = time.monotonic()
        if every > 0 and t - self.last_print >= every:
            self.last_print = t
            d = [math.degrees(a) for a in ang]
            self.get_logger().info(
                f'ускорение {acc[0]:+6.2f} {acc[1]:+6.2f} {acc[2]:+6.2f} м/с²  '
                f'|a| {math.sqrt(sum(x * x for x in acc)):5.2f}   '
                f'угл.скор {math.degrees(gyr[0]):+6.1f} '
                f'{math.degrees(gyr[1]):+6.1f} {math.degrees(gyr[2]):+6.1f} °/с   '
                f'углы крен {d[0]:+6.1f} тангаж {d[1]:+6.1f} курс {d[2]:+6.1f}')

    def report(self):
        if not self.ok and not self.bad:
            self.get_logger().warn('ответов нет: проверь порт, скорость и адрес')
            return
        total = self.ok + self.bad
        self.get_logger().info(
            f'принято {self.ok} из {total}'
            + (f', брак {self.bad} ({100.0 * self.bad / total:.1f}%)'
               if self.bad else ''))
        self.ok = self.bad = 0


def main():
    rclpy.init()
    n = WT901()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if n.ser is not None:
            try:
                n.ser.close()
            except Exception:
                pass
        n.destroy_node()


if __name__ == '__main__':
    sys.exit(main() or 0)
