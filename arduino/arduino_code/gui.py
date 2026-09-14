"""
GUI для тестирования моторов Arduino.
Зависимости: pyserial, matplotlib.
    pip install pyserial matplotlib

Запуск:
    python arduino_gui.py
"""
import queue
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import ttk, messagebox

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from serial.tools import list_ports

from arduino_api import ArduinoRobot, Odometry


# ==== Настройки ====
TELEM_HZ      = 20       # частота опроса одометрии
PLOT_POINTS   = 400      # сколько точек держать на графике
GUI_UPDATE_MS = 100      # как часто перерисовывать графики
CONTROL_MS    = 50       # как часто обрабатывать клавиатуру
KEY_TIMEOUT   = 0.15     # сек: если клавиша не нажималась — считаем отпущенной
LIN_SPEED     = 0.20     # м/с   (клавиатура: W/S)
ANG_SPEED     = 1.00     # рад/с (клавиатура: A/D)


# ======================================================================
#            Фоновый воркер: единственный поток, трогающий serial
# ======================================================================
class BotController:
    def __init__(self, port: str, baud: int = 115200):
        self.robot = ArduinoRobot(port, baud)
        self.handshake_ok = self.robot.handshake()
        self.cmd_queue: queue.Queue = queue.Queue()
        self.telem_queue: queue.Queue = queue.Queue(maxsize=200)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def post(self, fn):
        """Поставить в очередь функцию вида fn(robot). Выполнится в воркере."""
        self.cmd_queue.put(fn)

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
        try:
            self.robot.close()
        except Exception:
            pass

    # --- воркер ---
    def _run(self):
        period = 1.0 / TELEM_HZ
        while not self._stop.is_set():
            t0 = time.monotonic()

            # 1) отработать все накопившиеся исходящие команды
            while True:
                try:
                    fn = self.cmd_queue.get_nowait()
                except queue.Empty:
                    break
                try:
                    fn(self.robot)
                except Exception as e:
                    print("[cmd error]", e)

            # 2) запросить телеметрию
            try:
                d = self.robot.get_odom()
                try:
                    self.telem_queue.put_nowait(d)
                except queue.Full:
                    pass
            except Exception as e:
                print("[telem error]", e)
                time.sleep(0.3)

            dt = time.monotonic() - t0
            if dt < period:
                time.sleep(period - dt)


# ======================================================================
#                                GUI
# ======================================================================
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Arduino Motor Control")
        self.geometry("1180x720")
        self.minsize(950, 600)

        self.bot: BotController | None = None
        self.odom = Odometry()

        # данные для графиков
        self.n = 0
        self.tx = deque(maxlen=PLOT_POINTS)
        self.lt = deque(maxlen=PLOT_POINTS)
        self.rt = deque(maxlen=PLOT_POINTS)
        self.lv = deque(maxlen=PLOT_POINTS)
        self.rv = deque(maxlen=PLOT_POINTS)
        self.telem_queue: queue.Queue = queue.Queue(maxsize=2000)

        # клавиатура: keysym -> время последнего KeyPress
        self.pressed: dict[str, float] = {}
        self.last_sent = (0.0, 0.0)  # кэш последней отправленной (linear, angular)

        self._build_ui()
        self.refresh_ports()

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(GUI_UPDATE_MS, self._tick_gui)
        self.after(CONTROL_MS,    self._tick_control)

    # ---------------- UI ----------------
    def _build_ui(self):
        # ---- верхняя панель: порт ----
        top = ttk.Frame(self, padding=6)
        top.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(top, text="Порт:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(top, textvariable=self.port_var, width=28,
                                    state="readonly")
        self.port_cb.pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Обновить", command=self.refresh_ports).pack(side=tk.LEFT)

        self.connect_btn = ttk.Button(top, text="Подключить", command=self._toggle_connect)
        self.connect_btn.pack(side=tk.LEFT, padx=8)

        self.status_var = tk.StringVar(value="Не подключено")
        self.status_lbl = ttk.Label(top, textvariable=self.status_var, foreground="gray")
        self.status_lbl.pack(side=tk.LEFT, padx=12)

        # ---- центральная область: слева графики, справа команды ----
        body = ttk.Frame(self)
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        plot_frame = ttk.Frame(body)
        plot_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(6, 5), dpi=100)
        self.ax_t = self.fig.add_subplot(211)
        self.ax_v = self.fig.add_subplot(212, sharex=self.ax_t)
        self.fig.subplots_adjust(hspace=0.35, left=0.12, right=0.97,
                                 top=0.94, bottom=0.09)

        self.ax_t.set_title("Приращение тиков за такт")
        self.ax_t.set_ylabel("ticks")
        self.ax_t.grid(True, alpha=0.3)
        self.line_lt, = self.ax_t.plot([], [], 'b-', lw=1.0, label="L")
        self.line_rt, = self.ax_t.plot([], [], 'r-', lw=1.0, label="R")
        self.ax_t.legend(loc="upper right", fontsize=8)

        self.ax_v.set_title("Скорость колёс")
        self.ax_v.set_xlabel("такт")
        self.ax_v.set_ylabel("м/с")
        self.ax_v.grid(True, alpha=0.3)
        self.line_lv, = self.ax_v.plot([], [], 'b-', lw=1.0, label="L")
        self.line_rv, = self.ax_v.plot([], [], 'r-', lw=1.0, label="R")
        self.ax_v.legend(loc="upper right", fontsize=8)

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # ---- правая колонка: команды ----
        right = ttk.Frame(body, padding=8, width=320)
        right.pack(side=tk.RIGHT, fill=tk.Y)
        right.pack_propagate(False)

        # скорость
        gv = ttk.LabelFrame(right, text="Скорость (м/с, рад/с)", padding=6)
        gv.pack(fill=tk.X, pady=4)
        ttk.Label(gv, text="linear:").grid(row=0, column=0, sticky="w")
        self.e_lin = ttk.Entry(gv, width=10)
        self.e_lin.insert(0, "0.20")
        self.e_lin.grid(row=0, column=1, padx=4, pady=2)
        ttk.Label(gv, text="angular:").grid(row=1, column=0, sticky="w")
        self.e_ang = ttk.Entry(gv, width=10)
        self.e_ang.insert(0, "0.00")
        self.e_ang.grid(row=1, column=1, padx=4, pady=2)
        ttk.Button(gv, text="Отправить", command=self.cmd_set_velocity).grid(
            row=2, column=0, columnspan=2, sticky="ew", pady=(6, 0))

        # проехать N мм
        gm = ttk.LabelFrame(right, text="Проехать (мм)", padding=6)
        gm.pack(fill=tk.X, pady=4)
        ttk.Label(gm, text="dist:").grid(row=0, column=0, sticky="w")
        self.e_dist = ttk.Entry(gm, width=10); self.e_dist.insert(0, "300")
        self.e_dist.grid(row=0, column=1, padx=4, pady=2)
        ttk.Label(gm, text="speed:").grid(row=1, column=0, sticky="w")
        self.e_dspd = ttk.Entry(gm, width=10); self.e_dspd.insert(0, "100")
        self.e_dspd.grid(row=1, column=1, padx=4, pady=2)
        ttk.Button(gm, text="Поехали", command=self.cmd_move).grid(
            row=2, column=0, columnspan=2, sticky="ew", pady=(6, 0))

        # повернуть
        gt = ttk.LabelFrame(right, text="Повернуть (град)", padding=6)
        gt.pack(fill=tk.X, pady=4)
        ttk.Label(gt, text="angle:").grid(row=0, column=0, sticky="w")
        self.e_ang_deg = ttk.Entry(gt, width=10); self.e_ang_deg.insert(0, "90")
        self.e_ang_deg.grid(row=0, column=1, padx=4, pady=2)
        ttk.Label(gt, text="speed:").grid(row=1, column=0, sticky="w")
        self.e_tspd = ttk.Entry(gt, width=10); self.e_tspd.insert(0, "80")
        self.e_tspd.grid(row=1, column=1, padx=4, pady=2)
        ttk.Button(gt, text="Повернуть", command=self.cmd_turn).grid(
            row=2, column=0, columnspan=2, sticky="ew", pady=(6, 0))

        # клавиатура
        gk = ttk.LabelFrame(right, text="Клавиатура", padding=6)
        gk.pack(fill=tk.X, pady=4)
        self.kbd_enabled = tk.BooleanVar(value=True)
        ttk.Checkbutton(gk, text="Включить WASD / стрелки",
                        variable=self.kbd_enabled).pack(anchor="w")
        self.e_klin = ttk.Entry(gk, width=8); self.e_klin.insert(0, str(LIN_SPEED))
        self.e_kang = ttk.Entry(gk, width=8); self.e_kang.insert(0, str(ANG_SPEED))
        row = ttk.Frame(gk); row.pack(anchor="w", pady=2)
        ttk.Label(row, text="lin").pack(side=tk.LEFT)
        self.e_klin.pack(in_=row, side=tk.LEFT, padx=4)
        ttk.Label(row, text="ang").pack(side=tk.LEFT)
        self.e_kang.pack(in_=row, side=tk.LEFT, padx=4)
                # ---- одометрия ----
        go = ttk.LabelFrame(right, text="Одометрия", padding=6)
        go.pack(fill=tk.X, pady=4)

        self.odom_var = tk.StringVar(value="—")
        ttk.Label(go, textvariable=self.odom_var, justify="left",
                  font=("Consolas", 10)).pack(anchor="w")

        ttk.Button(go, text="Сбросить одометрию",
                   command=self.cmd_reset_odom).pack(fill=tk.X, pady=(6, 0))
        # стоп
        self.stop_btn = tk.Button(right, text="STOP (Space)",
                                  bg="#d33", fg="white",
                                  font=("TkDefaultFont", 11, "bold"),
                                  command=self.cmd_stop)
        self.stop_btn.pack(fill=tk.X, pady=10)

        # помощь
        help_txt = (
            "Управление:\n"
            "  W / ↑   — вперёд\n"
            "  S / ↓   — назад\n"
            "  A / ←   — поворот влево\n"
            "  D / →   — поворот вправо\n"
            "  Space   — стоп\n\n"
            "Клавиатура активна только когда\n"
            "включён чекбокс и есть подключение.\n\n"
            "Перед командой Move/Turn убедись, что\n"
            "клавиатура отпущена — эти режимы не\n"
            "согласованы с ручным управлением.\n"
        )
        ttk.Label(right, text=help_txt, justify="left",
                  foreground="#444").pack(anchor="w", pady=(6, 0))

        # ---- глобальные клавиши ----
        self.bind_all("<KeyPress>",   self._on_key_press)
        self.bind_all("<KeyRelease>", self._on_key_release)

    # ---------------- порт / подключение ----------------
    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_cb["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.bot is None:
            self._connect()
        else:
            self._disconnect()

    def _connect(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("Порт", "Выбери COM-порт")
            return
        self.connect_btn.config(state="disabled", text="Connecting...")
        self.status_var.set(f"Подключение к {port}...")
        self.status_lbl.config(foreground="gray")

        def work():
            try:
                bot = BotController(port)
                bot.start()
                ok = bot.handshake_ok
                self.after(0, lambda: self._on_connected(bot, ok, port))
            except Exception as e:
                self.after(0, lambda: self._on_connect_failed(e))

        threading.Thread(target=work, daemon=True).start()

    def _on_connected(self, bot: BotController, ok: bool, port: str):
        self.bot = bot
        self.connect_btn.config(state="normal", text="Отключить")
        if ok:
            self.status_var.set(f"Подключено: {port} (OK)")
            self.status_lbl.config(foreground="#2a7")
        else:
            self.status_var.set(f"Подключено: {port} (нет handshake)")
            self.status_lbl.config(foreground="#c80")

    def _on_connect_failed(self, err: Exception):
        self.connect_btn.config(state="normal", text="Подключить")
        self.status_var.set(f"Ошибка: {err}")
        self.status_lbl.config(foreground="#c00")
        messagebox.showerror("Ошибка подключения", str(err))

    def _disconnect(self):
        if self.bot is not None:
            try:
                self.bot.post(lambda r: r.stop())
                time.sleep(0.05)
            except Exception:
                pass
            self.bot.stop()
            self.bot = None
        self.connect_btn.config(text="Подключить")
        self.status_var.set("Не подключено")
        self.status_lbl.config(foreground="gray")
        self.pressed.clear()
        self.last_sent = (0.0, 0.0)
        self.pressed.clear()
        self.last_sent = (0.0, 0.0)
        self.odom.reset()               # <-- добавить

    # ---------------- команды ----------------
    def _require_bot(self) -> bool:
        if self.bot is None:
            return False
        return True

    def _parse_float(self, entry: ttk.Entry, name: str) -> float | None:
        try:
            return float(entry.get().replace(",", "."))
        except ValueError:
            messagebox.showwarning("Ввод", f"Не число: {name}")
            return None

    def _parse_int(self, entry: ttk.Entry, name: str) -> int | None:
        try:
            return int(float(entry.get().replace(",", ".")))
        except ValueError:
            messagebox.showwarning("Ввод", f"Не число: {name}")
            return None

    def cmd_set_velocity(self):
        if not self._require_bot():
            return
        lin = self._parse_float(self.e_lin, "linear")
        ang = self._parse_float(self.e_ang, "angular")
        if lin is None or ang is None:
            return
        bot = self.bot
        bot.post(lambda r, l=lin, a=ang: r.set_velocity(l, a))
        self.last_sent = (lin, ang)

    def cmd_move(self):
        if not self._require_bot():
            return
        d = self._parse_int(self.e_dist, "dist")
        s = self._parse_int(self.e_dspd, "speed")
        if d is None or s is None:
            return
        # защита от конфликта с клавиатурой
        self.kbd_enabled.set(False)
        self.pressed.clear()
        bot = self.bot
        bot.post(lambda r, dd=d, ss=s: r.move_distance(dd, ss))

    def cmd_turn(self):
        if not self._require_bot():
            return
        a = self._parse_int(self.e_ang_deg, "angle")
        s = self._parse_int(self.e_tspd, "speed")
        if a is None or s is None:
            return
        if not (-128 <= a <= 127) or not (-128 <= s <= 127):
            messagebox.showwarning(
                "Ограничение",
                "В прошивке угол и скорость читаются как int8_t.\n"
                "Диапазон: -128..127")
            return
        self.kbd_enabled.set(False)
        self.pressed.clear()
        bot = self.bot
        bot.post(lambda r, aa=a, ss=s: r.turn_robot(aa, ss))

    def cmd_stop(self):
        if not self._require_bot():
            return
        self.pressed.clear()
        self.last_sent = (0.0, 0.0)
        bot = self.bot
        bot.post(lambda r: r.stop())
    
    def cmd_reset_odom(self):
        self.odom.reset()
        self._refresh_odom_label()

    # ---------------- клавиатура ----------------
    def _key_id(self, event) -> str | None:
        k = event.keysym
        if k in ("w", "W", "Up"):    return "fwd"
        if k in ("s", "S", "Down"):  return "back"
        if k in ("a", "A", "Left"):  return "left"
        if k in ("d", "D", "Right"): return "right"
        if k == "space":             return "stop"
        return None

    def _on_key_press(self, event):
        # не мешать вводу текста в поля
        w = self.focus_get()
        if isinstance(w, (ttk.Entry, tk.Entry)):
            return
        k = self._key_id(event)
        if k is None:
            return
        if k == "stop":
            self.cmd_stop()
            return
        self.pressed[k] = time.monotonic()

    def _on_key_release(self, event):
        # На Windows KeyRelease прилетает между авто-повторами KeyPress,
        # поэтому здесь ничего не удаляем — таймер сам отсеет.
        pass

    def _tick_control(self):
        try:
            self._control_step()
        finally:
            self.after(CONTROL_MS, self._tick_control)

    def _control_step(self):
        if self.bot is None or not self.kbd_enabled.get():
            return

        now = time.monotonic()
        # отсеять "устаревшие" клавиши
        for k in [k for k, t in self.pressed.items() if now - t > KEY_TIMEOUT]:
            self.pressed.pop(k, None)

        try:
            lin_speed = float(self.e_klin.get().replace(",", "."))
            ang_speed = float(self.e_kang.get().replace(",", "."))
        except ValueError:
            lin_speed, ang_speed = LIN_SPEED, ANG_SPEED

        lin = 0.0
        ang = 0.0
        if "fwd"  in self.pressed: lin += lin_speed
        if "back" in self.pressed: lin -= lin_speed
        if "left" in self.pressed: ang += ang_speed
        if "right" in self.pressed: ang -= ang_speed

        # отправляем только при изменении
        if abs(lin - self.last_sent[0]) < 1e-6 and abs(ang - self.last_sent[1]) < 1e-6:
            return
        self.last_sent = (lin, ang)
        self.bot.post(lambda r, l=lin, a=ang: r.set_velocity(l, a))

    # ---------------- перерисовка графиков ----------------
    def _tick_gui(self):
        try:
            self._drain_telemetry()
            self._redraw()
        finally:
            self.after(GUI_UPDATE_MS, self._tick_gui)

    def _drain_telemetry(self):
        if self.bot is None:
            return
        q = self.bot.telem_queue
        while True:
            try:
                d = q.get_nowait()
            except queue.Empty:
                break
            self.odom.update(d)                     # <-- вот это
            self.n += 1
            self.tx.append(self.n)
            self.lt.append(d.left_ticks)
            self.rt.append(d.right_ticks)
            self.lv.append(d.left_speed)
            self.rv.append(d.right_speed)

        if not self.tx:
            return
        x = list(self.tx)
        self.line_lt.set_data(x, list(self.lt))
        self.line_rt.set_data(x, list(self.rt))
        self.line_lv.set_data(x, list(self.lv))
        self.line_rv.set_data(x, list(self.rv))
        self._refresh_odom_label()

    def _refresh_odom_label(self):
        o = self.odom
        txt = (
            f"L: {o.left_meters:+.3f} м  ({o.left_ticks_total:+d} тик)\n"
            f"R: {o.right_meters:+.3f} м  ({o.right_ticks_total:+d} тик)\n"
            f"Путь: {o.path_meters:+.3f} м\n"
            f"x: {o.x:+.3f} м   y: {o.y:+.3f} м\n"
            f"θ: {o.theta * 180.0 / 3.141592653589793:+.1f}°"
        )
        self.odom_var.set(txt)
    def _redraw(self):
        if not self.tx:
            return
        x0, x1 = self.tx[0], self.tx[-1]
        for ax in (self.ax_t, self.ax_v):
            ax.set_xlim(x0, max(x1, x0 + 1))
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)
        self.canvas.draw_idle()

    # ---------------- выход ----------------
    def _on_close(self):
        if self.bot is not None:
            try:
                self.bot.post(lambda r: r.stop())
                time.sleep(0.05)
            except Exception:
                pass
            self.bot.stop()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()