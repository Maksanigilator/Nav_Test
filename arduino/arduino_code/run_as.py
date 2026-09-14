import time
from arduino_api import ArduinoRobot

with ArduinoRobot(r"\\.\COM4") as bot:
    assert bot.handshake(), "Arduino не отвечает"

    bot.set_velocity(0.2, 0.0)
    for _ in range(20):
        d = bot.get_odom()
        print(d)
        time.sleep(0.1)

    bot.stop()