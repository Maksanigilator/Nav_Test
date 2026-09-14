"""
Простая API для управления моторами Arduino по Serial вместо ROS2.
Протокол совпадает с ros2_communication.hpp (Arduino-сторона).
"""
import struct
import time
import argparse
from dataclasses import dataclass
from typing import Optional
import math
from dataclasses import dataclass

import serial


# === Коды команд (должны совпадать с enum Commands в прошивке) ===
CMD_SET_VELOCITY       = 0x10
CMD_GET_ODOM_DATA      = 0x11
CMD_TURN_ROBOT         = 0x12
CMD_MOVE_DIST          = 0x13
CMD_HANDSHAKE_RESPONSE = 0x14

HANDSHAKE_REPLY = b"ARDUINO_OK"

# Формат struct EncoderData в прошивке: int16, int16, float, float (AVR little-endian)
ODOM_STRUCT = struct.Struct("<hhff")
ODOM_SIZE = ODOM_STRUCT.size  # = 12 байт


@dataclass
class EncoderData:
    left_ticks: int
    right_ticks: int
    left_speed: float   # м/с
    right_speed: float  # м/с


class ArduinoRobot:
    """
    Клиент для Arduino-контроллера моторов.

    Пример:
        with ArduinoRobot("/dev/ttyUSB0") as bot:
            bot.handshake()
            bot.set_velocity(0.2, 0.0)   # вперёд 0.2 м/с
            time.sleep(1)
            print(bot.get_odom())
            bot.stop()
    """

    def __init__(self, port: str, baudrate: int = 115200,
                 timeout: float = 1.0, reset_delay: float = 2.0):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        # Arduino Uno/Nano ресетится при открытии порта — ждём загрузки
        time.sleep(reset_delay)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    # --- низкоуровневое ---
    def _read_exact(self, n: int) -> bytes:
        buf = bytearray()
        deadline = time.monotonic() + (self.ser.timeout or 1.0)
        while len(buf) < n:
            chunk = self.ser.read(n - len(buf))
            if not chunk:
                if time.monotonic() > deadline:
                    raise TimeoutError(f"got {len(buf)}/{n} bytes")
                continue
            buf.extend(chunk)
        return bytes(buf)

    def _write(self, data: bytes) -> None:
        self.ser.write(data)
        self.ser.flush()

    # --- публичное API ---

    def handshake(self) -> bool:
        """Проверка связи. True если Arduino ответила 'ARDUINO_OK'."""
        self._write(bytes([CMD_HANDSHAKE_RESPONSE]))
        try:
            resp = self._read_exact(len(HANDSHAKE_REPLY))
        except TimeoutError:
            return False
        return resp == HANDSHAKE_REPLY

    def set_velocity(self, linear: float, angular: float) -> None:
        """
        linear  — м/с (положительное = вперёд)
        angular — рад/с (положительное = влево/против часовой)
        """
        self._write(struct.pack("<Bff", CMD_SET_VELOCITY, linear, angular))

    def get_odom(self) -> EncoderData:
        """Запросить одометрию. Блокирующий запрос/ответ."""
        self._write(bytes([CMD_GET_ODOM_DATA]))
        data = self._read_exact(ODOM_SIZE)
        l_ticks, r_ticks, l_spd, r_spd = ODOM_STRUCT.unpack(data)
        return EncoderData(l_ticks, r_ticks, l_spd, r_spd)

    def turn_robot(self, angle_deg: int, speed: int) -> None:
        """
        Повернуть на месте на angle_deg.
        ВАЖНО: в прошивке читается как int8_t, диапазон -128..127.
        """
        if not (-128 <= angle_deg <= 127):
            raise ValueError("angle_deg должен укладываться в int8 (-128..127)")
        if not (-128 <= speed <= 127):
            raise ValueError("speed должен укладываться в int8 (-128..127)")
        self._write(struct.pack("<Bbb", CMD_TURN_ROBOT, angle_deg, speed))

    def move_distance(self, dist_mm: int, speed: int) -> None:
        """Проехать dist_mm миллиметров со скоростью speed (тики/сек)."""
        self._write(struct.pack("<Bii", CMD_MOVE_DIST, dist_mm, speed))

    def stop(self) -> None:
        self.set_velocity(0.0, 0.0)

    def close(self) -> None:
        try:
            self.stop()
        except Exception:
            pass
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


# =======================  CLI для ручного тестирования =======================

def _cmd_monitor(bot: ArduinoRobot, hz: float):
    period = 1.0 / hz
    print("Мониторинг одометрии. Ctrl+C для выхода.")
    try:
        while True:
            d = bot.get_odom()
            print(f"L={d.left_ticks:+6d} R={d.right_ticks:+6d} "
                  f"vL={d.left_speed:+.3f} vR={d.right_speed:+.3f}")
            time.sleep(period)
    except KeyboardInterrupt:
        print()


def _cmd_drive(bot: ArduinoRobot, linear: float, angular: float, sec: float):
    print(f"set_velocity(linear={linear}, angular={angular}) на {sec} с")
    bot.set_velocity(linear, angular)
    try:
        time.sleep(sec)
    finally:
        bot.stop()
        print("stop()")


def _cmd_move(bot: ArduinoRobot, dist: int, speed: int):
    print(f"move_distance(dist_mm={dist}, speed={speed})")
    bot.move_distance(dist, speed)


def _cmd_turn(bot: ArduinoRobot, angle: int, speed: int):
    print(f"turn_robot(angle_deg={angle}, speed={speed})")
    bot.turn_robot(angle, speed)

# ====== Одометрия ======
# !!! Значения ДОЛЖНЫ совпадать с motor_regulator.h !!!
DEFAULT_WHEEL_CIRCUMFERENCE = 0.212   # м, длина окружности колеса
DEFAULT_TICKS_PER_REV       = 1000    # тиков энкодера на один оборот колеса
DEFAULT_WHEEL_BASE          = 0.150   # м, расстояние между колёсами


@dataclass
class Odometry:
    """
    Дифференциальная одометрия по дельтам энкодеров.
    Каждый вызов update(d) прибавляет ровно одну посылку EncoderData
    (где d.left_ticks / d.right_ticks — ДЕЛЬТЫ в тиках с прошлого запроса).
    """
    wheel_circumference: float = DEFAULT_WHEEL_CIRCUMFERENCE
    ticks_per_rev: int = DEFAULT_TICKS_PER_REV
    wheel_base: float = DEFAULT_WHEEL_BASE

    # накопленное
    left_ticks_total: int = 0
    right_ticks_total: int = 0

    # поза робота в мире
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0     # рад, 0 = смотрит по +X

    @property
    def meters_per_tick(self) -> float:
        return self.wheel_circumference / self.ticks_per_rev

    @property
    def left_meters(self) -> float:
        return self.left_ticks_total * self.meters_per_tick

    @property
    def right_meters(self) -> float:
        return self.right_ticks_total * self.meters_per_tick

    @property
    def path_meters(self) -> float:
        """Средний пройденный путь (по центру робота)."""
        return (self.left_meters + self.right_meters) / 2.0

    def reset(self) -> None:
        self.left_ticks_total = 0
        self.right_ticks_total = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def update(self, d: EncoderData) -> None:
        self.left_ticks_total  += d.left_ticks
        self.right_ticks_total += d.right_ticks

        dL = d.left_ticks  * self.meters_per_tick
        dR = d.right_ticks * self.meters_per_tick

        ds     = (dL + dR) / 2.0
        dtheta = (dR - dL) / self.wheel_base

        # полушаг по дуге — так точнее, чем считать по theta «до»
        self.x     += ds * math.cos(self.theta + dtheta / 2.0)
        self.y     += ds * math.sin(self.theta + dtheta / 2.0)
        self.theta += dtheta
def main():
    ap = argparse.ArgumentParser(description="CLI для тестирования Arduino-моторов")
    ap.add_argument("-p", "--port", required=True,
                    help="Например: COM5 (Windows), /dev/ttyUSB0 (Linux), "
                         "/dev/tty.usbserial-* (macOS)")
    ap.add_argument("--baud", type=int, default=115200)
    sub = ap.add_subparsers(dest="action", required=True)

    sub.add_parser("ping", help="handshake с Arduino")

    p_mon = sub.add_parser("monitor", help="печатать одометрию")
    p_mon.add_argument("--hz", type=float, default=10.0)

    p_drv = sub.add_parser("drive", help="задать линейную/угловую скорость")
    p_drv.add_argument("linear", type=float)
    p_drv.add_argument("angular", type=float)
    p_drv.add_argument("--sec", type=float, default=1.0)

    p_mv = sub.add_parser("move", help="проехать N мм")
    p_mv.add_argument("dist_mm", type=int)
    p_mv.add_argument("speed", type=int)

    p_tn = sub.add_parser("turn", help="повернуть на угол")
    p_tn.add_argument("angle_deg", type=int)
    p_tn.add_argument("speed", type=int)

    args = ap.parse_args()

    with ArduinoRobot(args.port, args.baud) as bot:
        if args.action == "ping":
            ok = bot.handshake()
            print("OK" if ok else "FAIL")
            raise SystemExit(0 if ok else 1)

        if args.action == "drive":
            _cmd_drive(bot, args.linear, args.angular, args.sec)
        elif args.action == "monitor":
            _cmd_monitor(bot, args.hz)
        elif args.action == "move":
            _cmd_move(bot, args.dist_mm, args.speed)
        elif args.action == "turn":
            _cmd_turn(bot, args.angle_deg, args.speed)


if __name__ == "__main__":
    main()