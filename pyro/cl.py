"""
============================================================
                 ROLLING MOTION LAB  v2
       Real-time ball analyzer  (Arduino UNO + 2 IR gates)
============================================================

Run:
    python rolling_motion_lab.py                 # real hardware
    python rolling_motion_lab.py --port COM6     # choose port
    python rolling_motion_lab.py --demo          # fake Arduino, no hardware

Install:
    pip install pyserial matplotlib numpy

Arduino serial protocol expected (unchanged from v1):
    STATE,A,<0|1>,B,<0|1>
    A,<trial>
    RESULT,<trial>,<time_s>,<speed_m_s>
    REJECT,...   WARNING,...
    WAITING_FOR_A   EXPERIMENT_COMPLETE
and the PC sends:  RESET
"""

import argparse
import csv
import math
import os
import queue
import random
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Optional

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

import numpy as np
import serial
import serial.tools.list_ports

from matplotlib.figure import Figure
from matplotlib.ticker import MaxNLocator
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


# ============================================================
# USER SETTINGS
# ============================================================

DEFAULT_PORT = "COM6"
BAUD_RATE = 115200

SENSOR_DISTANCE = 0.15        # m, gap between the two IR beams
MAX_TRIALS = 10

MAX_PLAUSIBLE_SPEED = 5.0     # m/s, anything faster is rejected as noise
LIVE_TIMEOUT = 5.0            # s, live timer gives up if B never fires
SPEED_MISMATCH_TOL = 0.03     # warn if Arduino speed differs >3 % from d/t
AUTOSAVE_DIR = "rolling_lab_data"

G = 9.81


# ------------------------------------------------------------
# Ball presets.  k = I / (m r^2)
#   hollow thin shell : 2/3     solid sphere : 2/5
# ------------------------------------------------------------

@dataclass(frozen=True)
class Ball:
    name: str
    diameter: float   # m
    mass: float       # kg
    k: float          # I / (m r^2)

    @property
    def radius(self):
        return self.diameter / 2

    @property
    def circumference(self):
        return math.pi * self.diameter


BALLS = {
    b.name: b
    for b in (
        Ball("Table tennis (hollow shell)", 0.040, 0.0027, 2 / 3),
        Ball("Golf ball (~solid sphere)", 0.0427, 0.0459, 2 / 5),
        Ball("Steel ball 20 mm (solid)", 0.020, 0.0327, 2 / 5),
    )
}
DEFAULT_BALL = "Table tennis (hollow shell)"


# ============================================================
# PHYSICS + STATISTICS  (pure functions, no GUI)
# ============================================================

def rolling_energy(ball, v):
    """Return (translational, rotational) kinetic energy in J."""
    trans = 0.5 * ball.mass * v ** 2
    omega = v / ball.radius
    rot = 0.5 * (ball.k * ball.mass * ball.radius ** 2) * omega ** 2
    return trans, rot


def solve_deceleration(d, t, s):
    """
    Constant-deceleration model.
        d = v_B*t + a*t^2/2        (between the gates)
        s = v_B^2 / (2a)           (after gate B)
    Substituting v_B = sqrt(2 a s) and x = sqrt(a):
        (t^2/2) x^2 + t*sqrt(2 s) x - d = 0
    """
    if d <= 0 or t <= 0 or s <= 0:
        return None
    A = 0.5 * t ** 2
    B = t * math.sqrt(2 * s)
    disc = B ** 2 + 4 * A * d
    x = (-B + math.sqrt(disc)) / (2 * A)
    return x ** 2


def stopping_analysis(ball, d, t, s):
    a = solve_deceleration(d, t, s)
    if a is None:
        return None
    v_b = math.sqrt(2 * a * s)
    v_a = v_b + a * t
    crr = (1 + ball.k) * a / G                 # (1+k) m a = Crr m g
    force = crr * ball.mass * G
    energy = force * s
    t_stop = v_b / a
    return {
        "a": a, "v_a": v_a, "v_b": v_b, "crr": crr,
        "force": force, "energy": energy, "t_stop": t_stop,
        "power": energy / t_stop if t_stop > 0 else 0.0,
    }


# Two-sided 95 % Student-t critical values (df -> t)
_T95 = {1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571, 6: 2.447,
        7: 2.365, 8: 2.306, 9: 2.262, 10: 2.228, 11: 2.201, 12: 2.179,
        13: 2.160, 14: 2.145, 15: 2.131, 16: 2.120, 17: 2.110,
        18: 2.101, 19: 2.093, 20: 2.086, 25: 2.060, 30: 2.042}


def t_crit(df):
    if df < 1:
        return 0.0
    for key in sorted(_T95):
        if df <= key:
            return _T95[key]
    return 1.96


def outlier_bounds(values):
    """Tukey fences; needs at least 4 points to mean anything."""
    if len(values) < 4:
        return None
    q1, q3 = np.percentile(values, [25, 75])
    iqr = q3 - q1
    return q1 - 1.5 * iqr, q3 + 1.5 * iqr


def describe(values):
    arr = np.asarray(values, dtype=float)
    n = len(arr)
    if n == 0:
        return None
    mean = float(arr.mean())
    std = float(arr.std(ddof=1)) if n > 1 else 0.0
    sem = std / math.sqrt(n) if n > 1 else 0.0
    return {
        "n": n, "mean": mean, "std": std, "sem": sem,
        "ci95": t_crit(n - 1) * sem,
        "median": float(np.median(arr)),
        "min": float(arr.min()), "max": float(arr.max()),
        "cv": 100 * std / mean if mean else 0.0,
    }


def consistency_label(cv, n):
    if n < 3:
        return "need ≥3 trials"
    if cv < 2:
        return "excellent"
    if cv < 5:
        return "good"
    if cv < 10:
        return "fair"
    return "poor"


# ============================================================
# TRIAL RECORD
# ============================================================

@dataclass
class Trial:
    n: int
    time: float
    arduino_speed: float
    speed: float = 0.0
    stop: Optional[float] = None
    excluded: bool = False
    analysis: Optional[dict] = None

    @property
    def crr(self):
        return self.analysis["crr"] if self.analysis else None


# ============================================================
# DEVICES  (real serial port, or a fake one for demo mode)
# ============================================================

class SerialDevice:
    def __init__(self, port, baud, out_q):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.out_q = out_q
        self._stop = threading.Event()

    def start(self):
        self.ser.reset_input_buffer()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while not self._stop.is_set():
            try:
                raw = self.ser.readline()
            except Exception:
                if not self._stop.is_set():
                    self.out_q.put("__DISCONNECTED__")
                return
            if raw:
                text = raw.decode("utf-8", errors="ignore").strip()
                if text:
                    self.out_q.put(text)

    def send(self, cmd):
        self.ser.write((cmd + "\n").encode())

    def close(self):
        self._stop.set()
        try:
            self.ser.close()
        except Exception:
            pass


class DemoDevice:
    """Imitates the Arduino so the GUI can be tested without hardware."""

    def __init__(self, out_q):
        self.q = out_q
        self._stop = threading.Event()
        self._reset = threading.Event()

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def send(self, cmd):
        if cmd == "RESET":
            self._reset.set()

    def close(self):
        self._stop.set()

    def _run(self):
        n = 0
        self.q.put("WAITING_FOR_A")
        while not self._stop.is_set():
            if self._reset.is_set():
                self._reset.clear()
                n = 0
                self.q.put("WAITING_FOR_A")
            if n >= MAX_TRIALS:
                self._stop.wait(0.2)
                continue
            if self._stop.wait(random.uniform(2.5, 4.0)):
                break
            if self._reset.is_set():
                continue
            n += 1
            self.q.put("STATE,A,1,B,0")
            self.q.put(f"A,{n}")
            t = max(0.1, random.gauss(0.45, 0.025))
            if self._stop.wait(t):
                break
            self.q.put("STATE,A,0,B,1")
            self.q.put(f"RESULT,{n},{t:.5f},{SENSOR_DISTANCE / t:.5f}")
            self.q.put("STATE,A,0,B,0")
            self.q.put("EXPERIMENT_COMPLETE" if n >= MAX_TRIALS
                       else "WAITING_FOR_A")


# ============================================================
# COLORS
# ============================================================

BG = "#0d1117"
PANEL = "#161b22"
PANEL2 = "#1c2128"
BORDER = "#30363d"
TEXT = "#f0f6fc"
MUTED = "#8b949e"
CYAN = "#39d0ff"
GREEN = "#3fb950"
ORANGE = "#d29922"
RED = "#f85149"

RED_STATES = ("REJECT", "WARNING", "TIMEOUT", "LOST", "DISCONNECTED")

AXIS_SPECS = [
    ("SPEED PER TRIAL", "Trial", "m/s"),
    ("TRANSIT TIME A → B", "Trial", "seconds"),
    ("ROLLING KINETIC ENERGY", "Trial", "Energy (J)"),
    ("EFFECTIVE Crr", "Trial", "Crr"),
    ("STOPPING DISTANCE", "Trial", "metres"),
    ("SPEED DISTRIBUTION", "m/s", "count"),
]


def format_axis(ax, title, xlabel, ylabel):
    ax.set_facecolor(PANEL)
    ax.set_title(title, color=TEXT, fontsize=10, fontweight="bold")
    ax.set_xlabel(xlabel, color=MUTED, fontsize=9)
    ax.set_ylabel(ylabel, color=MUTED, fontsize=9)
    ax.tick_params(colors=MUTED, labelsize=8)
    for spine in ax.spines.values():
        spine.set_color(BORDER)
    ax.grid(alpha=0.18)


def style_legend(ax):
    ax.legend(facecolor=PANEL2, edgecolor=BORDER, labelcolor=TEXT,
              fontsize=7)


def placeholder(ax, text):
    ax.text(0.5, 0.5, text, transform=ax.transAxes, ha="center",
            va="center", color=MUTED, fontsize=9)


# ============================================================
# APPLICATION
# ============================================================

class App:
    def __init__(self, root, demo=False, port=DEFAULT_PORT):
        self.root = root
        self.ball = BALLS[DEFAULT_BALL]
        self.sensor_distance = SENSOR_DISTANCE

        self.trials = []
        self.rejected = 0
        self.device = None
        self.queue = queue.Queue()

        self.sensor_a = False
        self.sensor_b = False
        self.status = "DISCONNECTED"
        self.live_start = None
        self.live_time = 0.0
        self.log_lines = []

        self.dirty = True
        self.log_dirty = True
        self._new_session()

        root.title("Rolling Motion Laboratory")
        root.geometry("1600x950")
        root.minsize(1250, 780)
        root.configure(bg=BG)

        self._build_styles()
        self._build_header(port)
        self._build_kpis()
        self._build_table()
        self._build_main()

        self.refresh_ports()
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.bind("<Control-s>", lambda e: self.export_csv())

        root.after(50, self._tick)
        if demo:
            root.after(200, self.start_demo)
        else:
            root.after(300, self.connect)

    # --------------------------------------------------------
    # BUILD UI
    # --------------------------------------------------------

    def _build_styles(self):
        s = ttk.Style()
        s.theme_use("clam")
        s.configure("Treeview", background=PANEL, foreground=TEXT,
                    fieldbackground=PANEL, rowheight=27, borderwidth=0)
        s.configure("Treeview.Heading", background=PANEL2,
                    foreground=TEXT, font=("Segoe UI", 10, "bold"))
        s.map("Treeview", background=[("selected", BORDER)])
        s.configure("TNotebook", background=PANEL, borderwidth=0)
        s.configure("TNotebook.Tab", background=PANEL2, foreground=MUTED,
                    padding=(14, 6), font=("Segoe UI", 10, "bold"))
        s.map("TNotebook.Tab",
              background=[("selected", BORDER)],
              foreground=[("selected", TEXT)])
        s.configure("TCombobox", fieldbackground=PANEL2,
                    background=PANEL2, foreground=TEXT)

    def _button(self, parent, text, command, bg=PANEL2, **pack):
        b = tk.Button(parent, text=text, command=command, bg=bg,
                      fg="white" if bg != PANEL2 else TEXT,
                      activebackground=BORDER, activeforeground=TEXT,
                      relief="flat", font=("Segoe UI", 9, "bold"),
                      cursor="hand2")
        pack.setdefault("side", "left")
        pack.setdefault("fill", "x")
        pack.setdefault("expand", True)
        pack.setdefault("padx", 3)
        b.pack(**pack)
        return b

    def _heading(self, parent, text, pady=(14, 5)):
        tk.Label(parent, text=text, bg=PANEL, fg=TEXT,
                 font=("Segoe UI", 11, "bold")).pack(
            anchor="w", padx=16, pady=pady)

    def _build_header(self, port):
        header = tk.Frame(self.root, bg=PANEL, height=70)
        header.pack(fill="x")

        tk.Label(header, text="ROLLING MOTION LABORATORY", bg=PANEL,
                 fg=TEXT, font=("Segoe UI", 22, "bold")).pack(
            side="left", padx=25, pady=12)

        self.status_label = tk.Label(
            header, text="● DISCONNECTED", bg=PANEL, fg=RED,
            font=("Segoe UI", 12, "bold"))
        self.status_label.pack(side="right", padx=25)

        conn = tk.Frame(header, bg=PANEL)
        conn.pack(side="right", padx=10)

        self.port_var = tk.StringVar(value=port)
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var,
                                       width=30)
        self.port_combo.pack(side="left", padx=3)

        self._button(conn, "⟳", self.refresh_ports, expand=False,
                     fill="none")
        self.connect_btn = self._button(conn, "CONNECT",
                                        self.toggle_connection,
                                        bg="#147d91", expand=False,
                                        fill="none")
        self._button(conn, "DEMO", self.start_demo, expand=False,
                     fill="none")

    def _build_kpis(self):
        frame = tk.Frame(self.root, bg=BG)
        frame.pack(fill="x", padx=16, pady=10)

        def kpi(title, initial):
            card = tk.Frame(frame, bg=PANEL2, highlightbackground=BORDER,
                            highlightthickness=1)
            card.pack(side="left", fill="both", expand=True, padx=5)
            tk.Label(card, text=title, bg=PANEL2, fg=MUTED,
                     font=("Segoe UI", 10)).pack(anchor="w", padx=15,
                                                 pady=(10, 2))
            v = tk.Label(card, text=initial, bg=PANEL2, fg=TEXT,
                         font=("Segoe UI", 20, "bold"))
            v.pack(anchor="w", padx=15, pady=(0, 10))
            return v

        self.trial_kpi = kpi("TRIAL", f"0 / {MAX_TRIALS}")
        self.timer_kpi = kpi("LIVE A → B TIMER", "0.000 s")
        self.speed_kpi = kpi("LATEST SPEED", "—")
        self.mean_kpi = kpi("RUNNING MEAN ± 95 % CI", "—")
        self.cv_kpi = kpi("CONSISTENCY (CV)", "—")
        self.crr_kpi = kpi("MEAN EFFECTIVE Crr", "—")

    def _build_table(self):
        # packed BEFORE the main area so it never gets squeezed out
        frame = tk.Frame(self.root, bg=BG)
        frame.pack(side="bottom", fill="x", padx=16, pady=(6, 10))

        cols = ("trial", "time", "speed", "stop", "decel", "crr",
                "quality")
        heads = {"trial": "TRIAL", "time": "TIME (s)",
                 "speed": "SPEED (m/s)", "stop": "STOP DIST. (m)",
                 "decel": "DECEL. (m/s²)", "crr": "EFFECTIVE Crr",
                 "quality": "QUALITY"}

        self.table = ttk.Treeview(frame, columns=cols, show="headings",
                                  height=6, selectmode="browse")
        for c in cols:
            self.table.heading(c, text=heads[c])
            self.table.column(c, width=120, anchor="center")
        self.table.tag_configure("excluded", foreground=MUTED)
        self.table.tag_configure("outlier", foreground=ORANGE)

        sb = ttk.Scrollbar(frame, orient="vertical",
                           command=self.table.yview)
        self.table.configure(yscrollcommand=sb.set)
        self.table.pack(side="left", fill="x", expand=True)
        sb.pack(side="right", fill="y")
        self.table.bind("<<TreeviewSelect>>",
                        lambda e: self._update_physics())

    def _build_main(self):
        main = tk.Frame(self.root, bg=BG)
        main.pack(fill="both", expand=True, padx=16)

        # ---------------- graphs ----------------
        graph_frame = tk.Frame(main, bg=BG)
        graph_frame.pack(side="left", fill="both", expand=True)

        self.fig = Figure(figsize=(10, 6), dpi=100, facecolor=BG,
                          constrained_layout=True)
        self.axes = self.fig.subplots(2, 3).flatten()
        (self.ax_speed, self.ax_time, self.ax_energy,
         self.ax_crr, self.ax_stop, self.ax_hist) = self.axes

        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # ---------------- right panel ----------------
        right = tk.Frame(main, bg=PANEL, width=410)
        right.pack(side="right", fill="y", padx=(14, 0))
        right.pack_propagate(False)

        nb = ttk.Notebook(right)
        nb.pack(fill="both", expand=True)

        control = tk.Frame(nb, bg=PANEL)
        physics = tk.Frame(nb, bg=PANEL)
        stats = tk.Frame(nb, bg=PANEL)
        log = tk.Frame(nb, bg=PANEL)
        for frame, name in ((control, "Control"), (physics, "Physics"),
                            (stats, "Stats"), (log, "Log")):
            nb.add(frame, text=name)

        self._build_control_tab(control)

        self.physics_label = tk.Label(
            physics, text="Waiting for first valid trial...", bg=PANEL,
            fg=MUTED, justify="left", anchor="nw", font=("Consolas", 9))
        self.physics_label.pack(anchor="nw", padx=14, pady=12)

        self.stats_label = tk.Label(
            stats, text="No data yet.", bg=PANEL, fg=MUTED,
            justify="left", anchor="nw", font=("Consolas", 9))
        self.stats_label.pack(anchor="nw", padx=14, pady=12)

        self.log_text = tk.Text(log, bg=BG, fg=MUTED, relief="flat",
                                font=("Consolas", 9), wrap="none")
        self.log_text.pack(fill="both", expand=True, padx=8, pady=8)

    def _build_control_tab(self, tab):
        # ---- ball ----
        self._heading(tab, "BALL", pady=(12, 4))
        self.ball_var = tk.StringVar(value=DEFAULT_BALL)
        combo = ttk.Combobox(tab, textvariable=self.ball_var,
                             values=list(BALLS), state="readonly")
        combo.pack(fill="x", padx=16)
        combo.bind("<<ComboboxSelected>>", self.on_ball_change)

        self.ball_info = tk.Label(tab, bg=PANEL, fg=MUTED, justify="left",
                                  font=("Consolas", 9))
        self.ball_info.pack(anchor="w", padx=16, pady=(6, 0))
        self._update_ball_info()

        # ---- sensor spacing ----
        row = tk.Frame(tab, bg=PANEL)
        row.pack(fill="x", padx=16, pady=(8, 0))
        tk.Label(row, text="Gate spacing (m)", bg=PANEL, fg=MUTED,
                 font=("Segoe UI", 9)).pack(side="left")
        self.dist_var = tk.StringVar(value=f"{self.sensor_distance:g}")
        tk.Entry(row, textvariable=self.dist_var, width=8, bg=PANEL2,
                 fg=TEXT, insertbackground=TEXT, relief="flat").pack(
            side="left", padx=6)
        self._button(row, "APPLY", self.apply_distance, expand=False,
                     fill="none")

        # ---- sensors ----
        self._heading(tab, "SENSOR STATUS")
        row = tk.Frame(tab, bg=PANEL)
        row.pack(fill="x", padx=16)
        self.sensor_a_label = tk.Label(row, text="● SENSOR A", bg=PANEL,
                                       fg=MUTED,
                                       font=("Segoe UI", 11, "bold"))
        self.sensor_a_label.pack(side="left")
        self.sensor_b_label = tk.Label(row, text="● SENSOR B", bg=PANEL,
                                       fg=MUTED,
                                       font=("Segoe UI", 11, "bold"))
        self.sensor_b_label.pack(side="right")

        # ---- gauge ----
        self._heading(tab, "LIVE SPEED")
        self.gauge = tk.Canvas(tab, width=320, height=135, bg=PANEL,
                               highlightthickness=0)
        self.gauge.pack(fill="x")

        # ---- stop distance ----
        self._heading(tab, "STOPPING DISTANCE (after Sensor B)")
        tk.Label(tab, text="Select a row in the table (default: latest),\n"
                           "enter distance in metres, press Enter.",
                 bg=PANEL, fg=MUTED, justify="left",
                 font=("Segoe UI", 9)).pack(anchor="w", padx=16)

        self.stop_entry = tk.Entry(tab, bg=PANEL2, fg=TEXT,
                                   insertbackground=TEXT, relief="flat")
        self.stop_entry.pack(fill="x", padx=16, pady=(7, 4))
        self.stop_entry.bind("<Return>", self.save_stop)

        row = tk.Frame(tab, bg=PANEL)
        row.pack(fill="x", padx=13, pady=4)
        self._button(row, "SAVE STOP", self.save_stop, bg="#147d91")
        self._button(row, "EXCLUDE / INCLUDE", self.toggle_exclude)
        self._button(row, "DELETE", self.delete_trial)

        row = tk.Frame(tab, bg=PANEL)
        row.pack(fill="x", padx=13, pady=4)
        self._button(row, "RESET", self.reset)
        self._button(row, "EXPORT CSV", self.export_csv)
        self._button(row, "SAVE PLOT", self.save_plot)

    # --------------------------------------------------------
    # SESSION / DERIVED DATA
    # --------------------------------------------------------

    def _new_session(self):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.autosave_path = os.path.join(AUTOSAVE_DIR,
                                          f"session_{stamp}.csv")

    def included(self):
        return [t for t in self.trials if not t.excluded]

    def recompute(self):
        for t in self.trials:
            t.speed = self.sensor_distance / t.time
            t.analysis = (stopping_analysis(self.ball,
                                            self.sensor_distance,
                                            t.time, t.stop)
                          if t.stop else None)

    def _log(self, text):
        self.log_lines.append(f"{datetime.now():%H:%M:%S}  {text}")
        if len(self.log_lines) > 300:
            self.log_lines.pop(0)
        self.log_dirty = True

    # --------------------------------------------------------
    # CONNECTION
    # --------------------------------------------------------

    def refresh_ports(self):
        ports = list(serial.tools.list_ports.comports())
        names = [p.device for p in ports]
        self.port_combo["values"] = names
        if self.port_var.get() in names or not ports:
            return
        hint = [p.device for p in ports
                if any(k in p.description.lower()
                       for k in ("arduino", "ch340", "usb serial"))]
        self.port_var.set(hint[0] if hint else names[0])

    def toggle_connection(self):
        if self.device:
            self.disconnect()
        else:
            self.connect()

    def connect(self, demo=False):
        if self.device:
            return
        try:
            dev = (DemoDevice(self.queue) if demo else
                   SerialDevice(self.port_var.get(), BAUD_RATE,
                                self.queue))
        except serial.SerialException as err:
            avail = "\n".join(f"{p.device} - {p.description}" for p in
                              serial.tools.list_ports.comports())
            self.status = "DISCONNECTED"
            messagebox.showerror(
                "Arduino connection failed",
                f"Could not open {self.port_var.get()}.\n\n{err}\n\n"
                f"Available ports:\n{avail or '(none found)'}\n\n"
                "Tip: close the Arduino IDE Serial Monitor first.")
            return
        self.device = dev
        self.status = "DEMO MODE" if demo else "CONNECTING..."
        self.connect_btn.config(text="DISCONNECT")
        # Uno resets when the port opens; give it ~2 s to boot
        self.root.after(100 if demo else 2000, self._finish_connect)

    def _finish_connect(self):
        if not self.device:
            return
        self.device.start()
        self.device.send("RESET")
        if self.status == "CONNECTING...":
            self.status = "CONNECTED"

    def disconnect(self, lost=False):
        if self.device:
            self.device.close()
        self.device = None
        self.live_start = None
        self.sensor_a = self.sensor_b = False
        self.status = "CONNECTION LOST" if lost else "DISCONNECTED"
        self.connect_btn.config(text="CONNECT")

    def start_demo(self):
        if self.device:
            self.disconnect()
        self.connect(demo=True)

    def _send(self, cmd):
        if not self.device:
            return
        try:
            self.device.send(cmd)
        except Exception as err:
            self._log(f"SEND FAILED: {err}")
            self.disconnect(lost=True)

    # --------------------------------------------------------
    # SERIAL EVENT HANDLING
    # --------------------------------------------------------

    def _handle(self, line):
        self._log(line)

        if line == "__DISCONNECTED__":
            self.disconnect(lost=True)

        elif line.startswith("STATE,"):
            p = line.split(",")          # STATE,A,0,B,1
            self.sensor_a = p[2] == "1"
            self.sensor_b = p[4] == "1"

        elif line.startswith("A,"):
            self.live_start = time.perf_counter()
            self.live_time = 0.0
            self.status = f"TRIAL {int(line.split(',')[1])}: BALL IN MOTION"

        elif line.startswith("RESULT,"):
            self._on_result(line)

        elif line.startswith("REJECT,"):
            self.rejected += 1
            self.live_start = None
            self.status = "READING REJECTED"
            self.dirty = True

        elif line.startswith("WARNING,"):
            self.status = "SENSOR WARNING"

        elif line == "WAITING_FOR_A":
            self.live_start = None
            self.status = "READY — ROLL BALL"

        elif line == "EXPERIMENT_COMPLETE":
            self.live_start = None
            self.status = "EXPERIMENT COMPLETE"

    def _on_result(self, line):
        p = line.split(",")
        trial, t, v_ard = int(p[1]), float(p[2]), float(p[3])
        self.live_start = None

        speed = self.sensor_distance / t if t > 0 else 0
        if t <= 0 or speed <= 0 or speed > MAX_PLAUSIBLE_SPEED:
            self.rejected += 1
            self._log("SOFTWARE REJECTED IMPOSSIBLE READING")
            self.status = "READING REJECTED"
            self.dirty = True
            return

        if abs(speed - v_ard) / speed > SPEED_MISMATCH_TOL:
            self._log(f"NOTE: Arduino speed {v_ard:.4f} differs from "
                      f"d/t = {speed:.4f}. Is the gate spacing in the "
                      "sketch the same as here?")

        self.trials.append(Trial(n=trial, time=t, arduino_speed=v_ard,
                                 speed=speed))
        self.status = f"TRIAL {trial}: COMPLETE"
        self.root.bell()
        self.dirty = True
        self._autosave()

    # --------------------------------------------------------
    # USER ACTIONS
    # --------------------------------------------------------

    def _selected_index(self):
        sel = self.table.selection()
        if sel:
            return int(sel[0])
        return len(self.trials) - 1 if self.trials else None

    def save_stop(self, event=None):
        idx = self._selected_index()
        if idx is None:
            messagebox.showwarning("No trial",
                                   "Complete a ball roll first.")
            return
        try:
            d = float(self.stop_entry.get().replace(",", "."))
            if d <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Invalid distance",
                                 "Enter a positive distance in metres.\n\n"
                                 "Example: 1.42")
            return
        self.trials[idx].stop = d
        self.stop_entry.delete(0, tk.END)
        self.recompute()
        self.dirty = True
        self._autosave()

    def toggle_exclude(self):
        idx = self._selected_index()
        if idx is None:
            return
        t = self.trials[idx]
        t.excluded = not t.excluded
        self._log(f"Trial {t.n} {'excluded' if t.excluded else 'included'}")
        self.dirty = True
        self._autosave()

    def delete_trial(self):
        idx = self._selected_index()
        if idx is None:
            return
        t = self.trials[idx]
        if messagebox.askyesno("Delete trial", f"Delete trial {t.n}?"):
            del self.trials[idx]
            self.dirty = True
            self._autosave()

    def reset(self):
        if self.trials and not messagebox.askyesno(
                "Reset experiment",
                "Clear all trials?\n\nThe autosaved CSV of this session "
                "is kept on disk."):
            return
        self.trials.clear()
        self.rejected = 0
        self.live_start = None
        self.live_time = 0.0
        self._new_session()
        if self.device:
            self.status = "RESETTING"
            self._send("RESET")
        self.dirty = True

    def on_ball_change(self, event=None):
        self.ball = BALLS[self.ball_var.get()]
        self._update_ball_info()
        self.recompute()
        self.dirty = True

    def apply_distance(self):
        try:
            d = float(self.dist_var.get().replace(",", "."))
            if not 0.01 <= d <= 5:
                raise ValueError
        except ValueError:
            messagebox.showerror("Invalid spacing",
                                 "Enter a gate spacing between 0.01 and "
                                 "5 metres.")
            return
        self.sensor_distance = d
        self.recompute()
        self.dirty = True
        self._log(f"Gate spacing set to {d:g} m")

    def _update_ball_info(self):
        b = self.ball
        self.ball_info.config(text=(
            f"Diameter       : {b.diameter * 1000:.1f} mm\n"
            f"Mass           : {b.mass * 1000:.2f} g\n"
            f"Circumference  : {b.circumference * 1000:.1f} mm\n"
            f"I / mr²        : {b.k:.4f}"))

    # --------------------------------------------------------
    # EXPORT
    # --------------------------------------------------------

    def _write_csv(self, path):
        folder = os.path.dirname(path)
        if folder:
            os.makedirs(folder, exist_ok=True)
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                "Trial", "Time_s", "Speed_m_s", "Arduino_Speed_m_s",
                "Stop_Distance_m", "Deceleration_m_s2", "Effective_Crr",
                "Entry_Speed_A_m_s", "Exit_Speed_B_m_s",
                "Angular_Speed_rad_s", "Translational_Energy_J",
                "Rotational_Energy_J", "Total_Energy_J", "Excluded",
                "Ball", "Gate_Spacing_m"])
            for t in self.trials:
                trans, rot = rolling_energy(self.ball, t.speed)
                a = t.analysis or {}
                w.writerow([
                    t.n, t.time, t.speed, t.arduino_speed,
                    t.stop if t.stop else "", a.get("a", ""),
                    a.get("crr", ""), a.get("v_a", ""), a.get("v_b", ""),
                    t.speed / self.ball.radius, trans, rot, trans + rot,
                    int(t.excluded), self.ball.name, self.sensor_distance])

    def _autosave(self):
        try:
            if self.trials:
                self._write_csv(self.autosave_path)
        except OSError as err:
            self._log(f"AUTOSAVE FAILED: {err}")

    def export_csv(self):
        if not self.trials:
            messagebox.showinfo("No data", "No experiment data available.")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv", filetypes=[("CSV files", "*.csv")],
            initialfile=f"rolling_{datetime.now():%Y%m%d_%H%M%S}.csv",
            title="Save experiment")
        if path:
            self._write_csv(path)
            messagebox.showinfo("Saved", "Experiment exported.")

    def save_plot(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".png", filetypes=[("PNG image", "*.png")],
            initialfile=f"rolling_plots_{datetime.now():%Y%m%d_%H%M%S}.png",
            title="Save plots")
        if path:
            self.fig.savefig(path, dpi=150,
                             facecolor=self.fig.get_facecolor())
            messagebox.showinfo("Saved", "Plots saved.")

    # --------------------------------------------------------
    # PERIODIC LOOP
    # --------------------------------------------------------

    def _tick(self):
        while True:
            try:
                line = self.queue.get_nowait()
            except queue.Empty:
                break
            try:
                self._handle(line)
            except Exception as err:
                self._log(f"BAD LINE '{line}': {err}")

        if self.live_start is not None:
            self.live_time = time.perf_counter() - self.live_start
            if self.live_time > LIVE_TIMEOUT:
                self.live_start = None
                self.status = "TIMEOUT — NO SENSOR B SIGNAL"
        self.timer_kpi.config(text=f"{self.live_time:.3f} s")

        self._update_status()

        if self.dirty:
            self.dirty = False
            self._update_kpis()
            self._update_table()
            self._update_physics()
            self._update_stats()
            self._update_gauge()
            self._update_graphs()

        if self.log_dirty:
            self.log_dirty = False
            self.log_text.delete("1.0", tk.END)
            self.log_text.insert(tk.END, "\n".join(self.log_lines[-100:]))
            self.log_text.see(tk.END)

        self.root.after(50, self._tick)

    # --------------------------------------------------------
    # VIEW UPDATES
    # --------------------------------------------------------

    def _update_status(self):
        color = CYAN
        if any(k in self.status for k in RED_STATES):
            color = RED
        elif "COMPLETE" in self.status:
            color = GREEN
        self.status_label.config(text=f"● {self.status}", fg=color)
        self.sensor_a_label.config(fg=CYAN if self.sensor_a else MUTED)
        self.sensor_b_label.config(fg=ORANGE if self.sensor_b else MUTED)

    def _update_kpis(self):
        self.trial_kpi.config(text=f"{len(self.trials)} / {MAX_TRIALS}")
        inc = self.included()
        if not self.trials:
            self.speed_kpi.config(text="—")
        else:
            self.speed_kpi.config(text=f"{self.trials[-1].speed:.3f} m/s")

        s = describe([t.speed for t in inc])
        if s:
            self.mean_kpi.config(
                text=f"{s['mean']:.3f} ± {s['ci95']:.3f}"
                if s["n"] > 1 else f"{s['mean']:.3f} m/s")
            self.cv_kpi.config(
                text=f"{s['cv']:.2f}%" if s["n"] > 1 else "—")
        else:
            self.mean_kpi.config(text="—")
            self.cv_kpi.config(text="—")

        crrs = [t.crr for t in inc if t.crr is not None]
        self.crr_kpi.config(
            text=f"{np.mean(crrs):.5f}" if crrs else "—")

    def _update_table(self):
        selected = self.table.selection()
        self.table.delete(*self.table.get_children())

        inc = self.included()
        bounds = outlier_bounds([t.speed for t in inc])

        for i, t in enumerate(self.trials):
            quality, tag = "VALID", ()
            if t.excluded:
                quality, tag = "EXCLUDED", ("excluded",)
            elif bounds and not bounds[0] <= t.speed <= bounds[1]:
                quality, tag = "OUTLIER", ("outlier",)
            a = t.analysis
            self.table.insert("", "end", iid=str(i), tags=tag, values=(
                t.n, f"{t.time:.5f}", f"{t.speed:.4f}",
                f"{t.stop:.3f}" if t.stop else "",
                f"{a['a']:.5f}" if a else "",
                f"{a['crr']:.6f}" if a else "", quality))

        if selected and self.table.exists(selected[0]):
            self.table.selection_set(selected[0])
        elif self.trials:
            self.table.see(str(len(self.trials) - 1))

    def _update_physics(self):
        idx = self._selected_index()
        if idx is None or idx >= len(self.trials):
            self.physics_label.config(
                text="Waiting for first valid trial...")
            return

        t = self.trials[idx]
        b = self.ball
        trans, rot = rolling_energy(b, t.speed)
        text = (
            f"TRIAL {t.n}"
            f"{'  (excluded)' if t.excluded else ''}\n"
            f"Average speed       : {t.speed:.5f} m/s\n"
            f"Transit time        : {t.time:.5f} s\n"
            f"Angular speed       : {t.speed / b.radius:.3f} rad/s\n"
            f"Spin / roll turns   : "
            f"{self.sensor_distance / b.circumference:.3f} rev\n\n"
            f"Translational KE    : {trans:.7f} J\n"
            f"Rotational KE       : {rot:.7f} J\n"
            f"Total rolling KE    : {trans + rot:.7f} J\n"
            f"Rotational fraction : {100 * rot / (trans + rot):.1f} %\n")

        a = t.analysis
        if a:
            text += (
                f"\nSTOPPING ANALYSIS\n"
                f"Distance after B    : {t.stop:.3f} m\n"
                f"Speed at A (est.)   : {a['v_a']:.5f} m/s\n"
                f"Speed at B (est.)   : {a['v_b']:.5f} m/s\n"
                f"Angular speed at B  : {a['v_b'] / b.radius:.3f} rad/s\n"
                f"Deceleration        : {a['a']:.5f} m/s²\n"
                f"Effective Crr       : {a['crr']:.6f}\n"
                f"Rolling resist. F   : {a['force']:.8f} N\n"
                f"Energy dissipated   : {a['energy']:.8f} J\n"
                f"Time to stop after B: {a['t_stop']:.3f} s\n"
                f"Avg. power loss     : {a['power']:.8f} W\n")
        else:
            text += "\nEnter a stopping distance to get Crr.\n"

        self.physics_label.config(text=text, fg=TEXT)

    def _update_stats(self):
        inc = self.included()
        lines = [f"Trials recorded   : {len(self.trials)}",
                 f"Used in statistics: {len(inc)}",
                 f"Rejected readings : {self.rejected}", ""]

        s = describe([t.speed for t in inc])
        if not s:
            self.stats_label.config(text="\n".join(lines), fg=MUTED)
            return

        lines += [
            "SPEED (m/s)",
            f"  Mean     : {s['mean']:.5f}",
            f"  Std dev  : {s['std']:.5f}",
            f"  SEM      : {s['sem']:.5f}",
            f"  95 % CI  : ± {s['ci95']:.5f}  (Student-t)",
            f"  Median   : {s['median']:.5f}",
            f"  Min/Max  : {s['min']:.4f} / {s['max']:.4f}",
            f"  CV       : {s['cv']:.2f} %  "
            f"({consistency_label(s['cv'], s['n'])})", ""]

        c = describe([t.crr for t in inc if t.crr is not None])
        if c:
            lines += [
                f"EFFECTIVE Crr  (n = {c['n']})",
                f"  Mean     : {c['mean']:.6f}",
                f"  Std dev  : {c['std']:.6f}",
                f"  95 % CI  : ± {c['ci95']:.6f}", ""]

        accel = describe([t.analysis["a"] for t in inc if t.analysis])
        if accel:
            lines += [
                "DECELERATION (m/s²)",
                f"  Mean     : {accel['mean']:.5f} "
                f"± {accel['std']:.5f}", ""]

        bounds = outlier_bounds([t.speed for t in inc])
        if bounds:
            lines.append(f"Outlier fences    : {bounds[0]:.4f} – "
                         f"{bounds[1]:.4f} m/s")
        else:
            lines.append("Outlier check needs ≥ 4 trials.")

        self.stats_label.config(text="\n".join(lines), fg=TEXT)

    def _update_gauge(self):
        g = self.gauge
        g.delete("all")
        speed = self.trials[-1].speed if self.trials else 0.0
        full = max(3.0, math.ceil(speed))
        w = g.winfo_width()
        cx = (w if w > 100 else 320) / 2
        cy, r = 105, 78
        box = (cx - r, cy - r, cx + r, cy + r)

        g.create_arc(*box, start=150, extent=-120, style="arc",
                     outline=BORDER, width=14)
        ratio = min(max(speed / full, 0), 1)
        if ratio > 0:
            g.create_arc(*box, start=150, extent=-120 * ratio,
                         style="arc", outline=CYAN, width=14)

        g.create_text(cx, 68, text=f"{speed:.3f}", fill=TEXT,
                      font=("Segoe UI", 24, "bold"))
        g.create_text(cx, 96, text="m/s", fill=MUTED,
                      font=("Segoe UI", 10))
        edge = r * math.cos(math.radians(30))
        g.create_text(cx - edge, cy - r / 2 + 18, text="0", fill=MUTED,
                      font=("Segoe UI", 8))
        g.create_text(cx + edge, cy - r / 2 + 18, text=f"{full:g}",
                      fill=MUTED, font=("Segoe UI", 8))

    def _update_graphs(self):
        for ax, spec in zip(self.axes, AXIS_SPECS):
            ax.clear()
            format_axis(ax, *spec)
            if spec[1] == "Trial":
                ax.xaxis.set_major_locator(MaxNLocator(integer=True))

        if not self.trials:
            self.canvas.draw_idle()
            return

        b = self.ball
        x = np.array([t.n for t in self.trials])
        v = np.array([t.speed for t in self.trials])
        tm = np.array([t.time for t in self.trials])
        ex = np.array([t.excluded for t in self.trials])
        ok = ~ex

        # ---- speed ----
        ax = self.ax_speed
        ax.plot(x, v, color=CYAN, marker="o", lw=2, label="Measured")
        if ex.any():
            ax.plot(x[ex], v[ex], "x", color=RED, ms=10, mew=2,
                    label="Excluded")
        if ok.any():
            m = v[ok].mean()
            ax.axhline(m, ls="--", color=ORANGE, label="Mean")
            if ok.sum() > 1:
                sd = v[ok].std(ddof=1)
                ax.axhspan(m - sd, m + sd, color=ORANGE, alpha=0.12,
                           label="±1σ")
        style_legend(ax)

        # ---- time ----
        self.ax_time.plot(x, tm, color=CYAN, marker="o", lw=2)

        # ---- energy (stacked) ----
        trans = 0.5 * b.mass * v ** 2
        rot = 0.5 * b.k * b.mass * v ** 2
        self.ax_energy.bar(x, trans, color=CYAN, label="Translational")
        self.ax_energy.bar(x, rot, bottom=trans, color=GREEN,
                           label="Rotational")
        style_legend(self.ax_energy)

        # ---- Crr ----
        cx_ = [t.n for t in self.trials if t.crr is not None]
        cy_ = [t.crr for t in self.trials if t.crr is not None]
        if cy_:
            self.ax_crr.plot(cx_, cy_, color=GREEN, marker="o", lw=2)
            inc_crr = [t.crr for t in self.included()
                       if t.crr is not None]
            if inc_crr:
                self.ax_crr.axhline(np.mean(inc_crr), ls="--",
                                    color=ORANGE, label="Mean")
                style_legend(self.ax_crr)
        else:
            placeholder(self.ax_crr, "Enter stopping distances\n"
                                     "to see Crr")

        # ---- stopping distance ----
        sx = [t.n for t in self.trials if t.stop]
        sy = [t.stop for t in self.trials if t.stop]
        if sy:
            self.ax_stop.plot(sx, sy, color=ORANGE, marker="s", lw=2)
        else:
            placeholder(self.ax_stop, "No stopping distances yet")

        # ---- histogram ----
        if ok.sum() >= 2:
            bins = min(8, max(3, int(ok.sum())))
            self.ax_hist.hist(v[ok], bins=bins, color=CYAN, alpha=0.85,
                              edgecolor=BG)
            self.ax_hist.axvline(v[ok].mean(), ls="--", color=ORANGE)
        else:
            placeholder(self.ax_hist, "Needs ≥ 2 trials")

        self.canvas.draw_idle()

    # --------------------------------------------------------
    # SHUTDOWN
    # --------------------------------------------------------

    def close(self):
        if self.device:
            self.device.close()
        self.root.destroy()


# ============================================================
# MAIN
# ============================================================

def main():
    parser = argparse.ArgumentParser(description="Rolling Motion Lab")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--demo", action="store_true",
                        help="simulate the Arduino (no hardware needed)")
    args = parser.parse_args()

    root = tk.Tk()
    App(root, demo=args.demo, port=args.port)
    root.mainloop()


if __name__ == "__main__":
    main()