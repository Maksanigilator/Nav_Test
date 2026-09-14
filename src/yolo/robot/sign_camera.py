#!/usr/bin/env python3
"""MJPEG-сервер камеры для Raspberry. Запускается на роботе, не в контейнере.

Зачем не ffmpeg напрямую: у ffmpeg с `-listen 1` встроенный HTTP-сервер
обслуживает ровно одного клиента и после его ухода завершается. Нам же
нужно, чтобы детектор подключался много раз за заезд, а камера при этом
оставалась открытой.

Почему камера открыта постоянно, хотя кадры отдаются по запросу:

  * открытие V4L2 плюс отработка автоэкспозиции занимает от полусекунды
    до полутора, и первые кадры после открытия выходят тёмными — а
    решение по ним меняет маршрут робота;
  * драйвер копит кадры в своей очереди, и если из неё не вычитывать, по
    запросу придёт не «сейчас», а то, что легло туда несколько секунд
    назад: робот уже в точке, а на снимке ещё коридор до неё.

Поэтому ffmpeg крутится фоном всегда и пишет MJPEG в конвейер, а поток
кадров непрерывно вычитывается в память. Наружу уходит только последний
кадр и только пока кто-то подключён — по Wi-Fi в покое не идёт ничего.

Камера отдаёт MJPEG аппаратно, поэтому стоит `-c copy`: ни декодирования,
ни перекодирования, процессор Raspberry свободен для лидара и одометрии.

    ./sign_camera.py                       # 640x480, 5 к/с, порт 8080
    ./sign_camera.py --size 1280x720 --fps 10 --port 8080
"""
import argparse
import http.server
import socketserver
import subprocess
import threading
import time

SOI = b'\xff\xd8'      # начало JPEG
EOI = b'\xff\xd9'      # конец JPEG


class Camera:
    """Держит ffmpeg открытым и хранит последний кадр."""

    def __init__(self, device, size, fps):
        self.cmd = [
            'ffmpeg', '-hide_banner', '-loglevel', 'error',
            '-f', 'v4l2', '-input_format', 'mjpeg',
            '-video_size', size, '-framerate', str(fps),
            '-i', device,
            '-c', 'copy', '-f', 'mjpeg', 'pipe:1',
        ]
        self.frame = None
        self.stamp = 0.0
        self.lock = threading.Lock()
        self.frames_read = 0
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            proc = subprocess.Popen(self.cmd, stdout=subprocess.PIPE,
                                    stderr=subprocess.DEVNULL, bufsize=0)
            buf = b''
            try:
                while True:
                    chunk = proc.stdout.read(65536)
                    if not chunk:
                        break
                    buf += chunk
                    # Кадры режем по маркерам JPEG, а не по длине: так
                    # разбор не зависит от того, как лёг буфер конвейера.
                    while True:
                        i = buf.find(SOI)
                        j = buf.find(EOI, i + 2)
                        if i < 0 or j < 0:
                            break
                        with self.lock:
                            self.frame = buf[i:j + 2]
                            self.stamp = time.monotonic()
                            self.frames_read += 1
                        buf = buf[j + 2:]
                    # Защита от разрастания буфера, если поток поехал.
                    if len(buf) > 4_000_000:
                        buf = b''
            except Exception:
                pass
            finally:
                # kill() только посылает сигнал. Без wait() процесс остаётся
                # зомби и продолжает держать /dev/video0, а следующий ffmpeg
                # падает с "Device or resource busy" — и так по кругу, пока
                # процессы не накопятся десятками. Наблюдалось четыре ffmpeg
                # на один сервер камеры.
                try:
                    proc.kill()
                    proc.wait(timeout=5)
                except Exception:
                    pass
            print('ffmpeg упал, перезапуск через 2 с', flush=True)
            time.sleep(2.0)

    def get(self):
        with self.lock:
            return self.frame, self.stamp


class Handler(http.server.BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.0'
    camera = None
    fps = 5

    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path.startswith('/snapshot'):
            self._snapshot()
        else:
            self._stream()

    def _snapshot(self):
        """Один кадр — для быстрой проверки браузером или curl."""
        frame, _ = self.camera.get()
        if frame is None:
            self.send_error(503, 'no frame yet')
            return
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(frame)))
        self.end_headers()
        self.wfile.write(frame)

    def _stream(self):
        self.send_response(200)
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        period = 1.0 / max(self.fps, 1)
        last = 0.0
        try:
            while True:
                frame, stamp = self.camera.get()
                # Отдаём только новые кадры: повторять один и тот же
                # бессмысленно, детектор посчитает его несколько раз.
                if frame is None or stamp == last:
                    time.sleep(0.01)
                    continue
                last = stamp
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n'
                                 b'Content-Length: ' + str(len(frame)).encode()
                                 + b'\r\n\r\n' + frame + b'\r\n')
                self.wfile.flush()
                time.sleep(period)
        except (BrokenPipeError, ConnectionResetError):
            pass       # клиент отключился — это норма, а не ошибка


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--device', default='/dev/video0')
    p.add_argument('--size', default='640x480')
    p.add_argument('--fps', type=int, default=5)
    p.add_argument('--port', type=int, default=8080)
    a = p.parse_args()

    # Второй экземпляр на том же порту не поднимется из-за SO_REUSEADDR не
    # сразу, зато успеет занять камеру и сломать первый. Проверяем заранее.
    import socket
    probe = socket.socket()
    if probe.connect_ex(('127.0.0.1', a.port)) == 0:
        probe.close()
        print(f'порт {a.port} уже занят — сервер камеры запущен, выхожу',
              flush=True)
        return
    probe.close()

    Handler.camera = Camera(a.device, a.size, a.fps)
    Handler.fps = a.fps
    print(f'камера {a.device} {a.size} {a.fps} к/с -> http://0.0.0.0:{a.port}/stream',
          flush=True)
    print(f'один кадр: http://0.0.0.0:{a.port}/snapshot', flush=True)
    with Server(('0.0.0.0', a.port), Handler) as s:
        s.serve_forever()


if __name__ == '__main__':
    main()
