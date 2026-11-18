# ============================================================
#           ROLLING MOTION LABORATORY  v2
#
#   Real-Time Table Tennis Ball Analyzer
#   Hardware: Arduino UNO + 2 IR break-beam sensors
#   Physics model: hollow spherical shell  I = (2/3) m r^2
#
#   Serial protocol (unchanged from v1 firmware):
#     STATE,A,0,B,1
#     A,<trial>
#     RESULT,<trial>,<time_ms_or_s>,<speed>
#     REJECT / WARNING / WAITING_FOR_A / EXPERIMENT_COMPLETE
#   Commands sent to Arduino:
#     RESET\n
# ============================================================

import json
import os
import csv
import math
import time
import queue
import threading
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

try:
    import numpy as np
    HAVE_NP = True
except ImportError:
    HAVE_NP = False

import serial
from serial.tools import list_ports          # FIX: explicit import

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

plt.rcParams.update({
    "axes.edgecolor": "#30363d",
    "axes.labelcolor": "#8b949e",
    "xtick.color": "#8b949e",
    "ytick.color": "#8b949e",
    "text.color": "#f0f6fc",
    "axes.titlecolor": "#f0f6fc",
})

# ============================================================
# CONFIG (persisted to rolling_lab_config.json)
# ============================================================

CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__))
                           if "__file__" in dir() else os.getcwd(),
                           "rolling_lab_config.json")

DEFAULT_CONFIG = {
    "port": "COM6",
    "baud": 115200,
    "sensor_distance_m": 0.15,
    "ball_diameter_m": 0.040,
    "ball_mass_kg": 0.0027,
    "inertia_factor": 2.0 / 3.0,   # hollow spherical shell
    "max_trials": 10,
    "max_speed_ms": 5.0,           # reject above this (impossible)
    "gauge_max_ms": 3.0,
}


def load_config():
    cfg = dict(DEFAULT_CONFIG)
    try:
        with open(CONFIG_FILE, "r") as f:
            cfg.update(json.load(f))
    except Exception:
        pass
    return cfg


def save_config(cfg):
    try:
        with open(CONFIG_FILE, "w") as f:
            json.dump(cfg, f, indent=2)
    except Exception:
        pass


CFG = load_config()

# Derived ball quantities
BALL_RADIUS = CFG["ball_diameter_m"] / 2.0
BALL_CIRCUMFERENCE = math.pi * CFG["ball_diameter_m"]
BETA = CFG["inertia_factor"]          # I / (m r^2)
K_ROLL = 1.0 + BETA                   # 5/3 for hollow shell
G = 9.81

# ============================================================
# DATA MODEL  (one dict per trial - no parallel arrays)
# ============================================================

trials = []          # {trial, time, speed, stop, crr, excluded, note}
event_log = []

# Real-time state
sensor_a_state = False
sensor_b_state = False
live_timer_start = None
live_transit_time = 0.0
experiment_status = "DISCONNECTED"
last_arduino_trial = 0

# Serial
arduino = None
serial_queue = queue.Queue()
program_running = True
serial_lock = threading.Lock()
session_log_file = None

# ============================================================
# PHYSICS HELPERS
# ============================================================

def solve_deceleration(d, t, s):
    """Constant-deceleration model.

    Between sensors:  v_avg = d/t = (v_A + v_B)/2,  v_B = v_avg - a*t/2
    After sensor B:   v_B^2 = 2*a*s
    =>  (t/2)*a + sqrt(2s)*sqrt(a) - v_avg = 0   (quadratic in sqrt(a))
    Returns (a, v_A, v_B) or None."""
    if d <= 0 or t <= 0 or s <= 0:
        return None
    A, B, C = t / 2.0, math.sqrt(2.0 * s), -d / t
    disc = B * B - 4.0 * A * C
    if disc <= 0:
        return None
    sqrt_a = (-B + math.sqrt(disc)) / (2.0 * A)
    if sqrt_a <= 0:
        return None
    a = sqrt_a * sqrt_a
    v_avg = d / t
    v_B = v_avg - a * t / 2.0
    v_A = v_avg + a * t / 2.0
    if v_B <= 0 or v_A <= 0:
        return None
    return a, v_A, v_B


def crr_from_deceleration(a):
    """Pure rolling on horizontal surface: a = g*Crr / (1 + I/mr^2)."""
    return K_ROLL * a / G


def kinetic_energy(v):
    return 0.5 * CFG["ball_mass_kg"] * v * v * K_ROLL


def recompute_crr(t):
    """Fill in t['crr'] from its stop distance."""
    t["crr"] = None
    if t.get("stop") is None:
        return
    sol = solve_deceleration(CFG["sensor_distance_m"], t["time"], t["stop"])
    if sol:
        t["crr"] = crr_from_deceleration(sol[0])


def included_trials():
    return [t for t in trials if not t["excluded"]]


def mean_valid_crr():
    vals = [t["crr"] for t in included_trials()
            if t.get("crr") is not None]
    return (sum(vals) / len(vals)) if vals else None


# ============================================================
# STATISTICS
# ============================================================

def calculate_statistics():
    data = [t["speed"] for t in included_trials()]
    n = len(data)
    if n == 0:
        return None
    if HAVE_NP:
        arr = np.array(data, dtype=float)
        mean = float(np.mean(arr)); std = float(np.std(arr, ddof=1)) if n > 1 else 0.0
        median = float(np.median(arr)); mn = float(np.min(arr)); mx = float(np.max(arr))
        q1 = float(np.percentile(arr, 25)); q3 = float(np.percentile(arr, 75))
    else:
        mean = sum(data) / n
        std = math.sqrt(sum((x - mean) ** 2 for x in data) / (n - 1)) if n > 1 else 0.0
        srt = sorted(data); median = srt[n // 2] if n % 2 else (srt[n//2-1]+srt[n//2])/2
        mn, mx = srt[0], srt[-1]
        q1 = srt[int(0.25 * (n - 1))]; q3 = srt[int(0.75 * (n - 1))]
    sem = std / math.sqrt(n) if n > 1 else 0.0
    ci95 = 1.96 * sem
    cv = 100.0 * std / mean if mean else 0.0
    iqr = q3 - q1
    low, high = q1 - 1.5 * iqr, q3 + 1.5 * iqr
    outliers = [t["trial"] for t in included_trials()
                if t["speed"] < low or t["speed"] > high]
    return {"n": n, "mean": mean, "std": std, "sem": sem, "ci95": ci95,
            "median": median, "min": mn, "max": mx, "cv": cv,
            "outliers": outliers}


# ============================================================
# TKINTER SETUP
# ============================================================

root = tk.Tk()
root.title("Rolling Motion Laboratory  v2")
root.geometry("1650x980")
root.minsize(1250, 780)

BG = "#0d1117"; PANEL = "#161b22"; PANEL2 = "#1c2128"
TEXT = "#f0f6fc"; MUTED = "#8b949e"
CYAN = "#39d0ff"; GREEN = "#3fb950"; ORANGE = "#d29922"; RED = "#f85149"

root.configure(bg=BG)

style = ttk.Style()
style.theme_use("clam")
style.configure("Treeview", background=PANEL, foreground=TEXT,
                fieldbackground=PANEL, rowheight=27, borderwidth=0)
style.configure("Treeview.Heading", background=PANEL2, foreground=TEXT,
                font=("Segoe UI", 10, "bold"))
style.map("Treeview", background=[("selected", "#30363d")])

# ============================================================
# HEADER
# ============================================================

header = tk.Frame(root, bg=PANEL, height=70)
header.pack(fill="x")

tk.Label(header, text="ROLLING MOTION LABORATORY",
         bg=PANEL, fg=TEXT, font=("Segoe UI", 24, "bold")
         ).pack(side="left", padx=25)

# connection controls
conn_frame = tk.Frame(header, bg=PANEL)
conn_frame.pack(side="right", padx=15)

port_combo = ttk.Combobox(conn_frame, width=12, values=[])
port_combo.set(CFG["port"])
port_combo.pack(side="left", padx=3)

def scan_ports():
    ports = [p.device for p in list_ports.comports()]
    port_combo["values"] = ports
    if ports and port_combo.get() not in ports:
        port_combo.set(ports[0])
    log_event("PORT SCAN: " + (", ".join(ports) if ports else "none found"))

tk.Button(conn_frame, text="SCAN", command=scan_ports,
          bg=PANEL2, fg=TEXT, relief="flat", width=6).pack(side="left", padx=3)

connect_button = tk.Button(conn_frame, text="CONNECT", command=lambda: connect_arduino(),
                           bg="#147d91", fg="white", relief="flat", width=9)
connect_button.pack(side="left", padx=3)

def disconnect_arduino():
    global arduino, experiment_status
    program_running_local = True
    try:
        if arduino:
            with serial_lock:
                arduino.close()
            arduino = None
    except Exception:
        pass
    experiment_status = "DISCONNECTED"
    connect_button.config(state="normal")
    refresh_ui()

tk.Button(conn_frame, text="DISCONNECT", command=disconnect_arduino,
          bg=PANEL2, fg=TEXT, relief="flat", width=10).pack(side="left", padx=3)

status_label = tk.Label(header, text="● DISCONNECTED", bg=PANEL, fg=RED,
                        font=("Segoe UI", 12, "bold"))
status_label.pack(side="right", padx=15)

# ============================================================
# KPI AREA
# ============================================================

kpi_frame = tk.Frame(root, bg=BG)
kpi_frame.pack(fill="x", padx=16, pady=10)

def create_kpi(title, initial):
    card = tk.Frame(kpi_frame, bg=PANEL2,
                    highlightbackground="#30363d", highlightthickness=1)
    card.pack(side="left", fill="both", expand=True, padx=5)
    tk.Label(card, text=title, bg=PANEL2, fg=MUTED,
             font=("Segoe UI", 10)).pack(anchor="w", padx=15, pady=(10, 2))
    value = tk.Label(card, text=initial, bg=PANEL2, fg=TEXT,
                     font=("Segoe UI", 18, "bold"))
    value.pack(anchor="w", padx=15, pady=(0, 10))
    return value

trial_kpi   = create_kpi("TRIAL", f"0 / {CFG['max_trials']}")
timer_kpi   = create_kpi("LIVE A→B TIMER", "0.000 s")
proj_kpi    = create_kpi("PROJECTED SPEED", "—")
speed_kpi   = create_kpi("LAST SPEED", "0.000 m/s")
mean_kpi    = create_kpi("MEAN ± 95% CI", "—")
cv_kpi      = create_kpi("CONSISTENCY (CV)", "—")
crr_kpi     = create_kpi("EFFECTIVE Crr (mean)", "—")

# ============================================================
# MAIN AREA
# ============================================================

main_frame = tk.Frame(root, bg=BG)
main_frame.pack(fill="both", expand=True, padx=16)

graph_frame = tk.Frame(main_frame, bg=BG)
graph_frame.pack(side="left", fill="both", expand=True)

fig = Figure(figsize=(10, 6.4), dpi=100, facecolor=BG)
ax_speed  = fig.add_subplot(221)
ax_time   = fig.add_subplot(222)
ax_energy = fig.add_subplot(223)
ax_crr    = fig.add_subplot(224)

canvas = FigureCanvasTkAgg(fig, master=graph_frame)
canvas.get_tk_widget().pack(fill="both", expand=True)

def format_axis(ax, title, xlabel, ylabel):
    ax.set_facecolor(PANEL)
    ax.set_title(title, fontsize=11, fontweight="bold")
    ax.set_xlabel(xlabel); ax.set_ylabel(ylabel)
    ax.grid(alpha=0.18)
    for spine in ax.spines.values():
        spine.set_color("#30363d")

# ============================================================
# RIGHT PANEL
# ============================================================

right_panel = tk.Frame(main_frame, bg=PANEL, width=390)
right_panel.pack(side="right", fill="y", padx=(14, 0))
right_panel.pack_propagate(False)

# ---- Track animation (NEW) ----
tk.Label(right_panel, text="TRACK", bg=PANEL, fg=TEXT,
         font=("Segoe UI", 13, "bold")).pack(anchor="w", padx=18, pady=(12, 4))

track = tk.Canvas(right_panel, width=340, height=95, bg=PANEL,
                  highlightthickness=0)
track.pack(fill="x")

def draw_track(progress=None):
    track.delete("all")
    w = track.winfo_width() or 340
    y, pad = 60, 30
    x0, x1 = pad, w - pad
    track.create_line(x0, y, x1, y, fill="#30363d", width=4)
    for x, col, lbl in ((x0, CYAN, "A"), (x1, ORANGE, "B")):
        track.create_line(x, y - 22, x, y + 8, fill=col, width=3)
        track.create_text(x, y + 22, text=lbl, fill=col,
                          font=("Segoe UI", 10, "bold"))
    track.create_text(w / 2, 12,
                      text=f"d = {CFG['sensor_distance_m']*100:.1f} cm",
                      fill=MUTED, font=("Segoe UI", 9))
    if progress is not None:
        p = min(max(progress, 0.0), 1.0)
        bx = x0 + p * (x1 - x0)
        track.create_oval(bx - 8, y - 8, bx + 8, y + 8, fill=TEXT, outline="")
    # sensor state lamps
    track.create_oval(x0 - 5, y - 34, x0 + 5, y - 24,
                      fill=CYAN if sensor_a_state else "#30363d", outline="")
    track.create_oval(x1 - 5, y - 34, x1 + 5, y - 24,
                      fill=ORANGE if sensor_b_state else "#30363d", outline="")

# ---- Ball info ----
tk.Label(right_panel, text="TABLE TENNIS BALL", bg=PANEL, fg=TEXT,
         font=("Segoe UI", 13, "bold")).pack(anchor="w", padx=18, pady=(12, 6))

ball_info = (
    f"Diameter       : {CFG['ball_diameter_m']*1000:.1f} mm\n"
    f"Radius         : {BALL_RADIUS*1000:.1f} mm\n"
    f"Mass           : {CFG['ball_mass_kg']*1000:.2f} g\n"
    f"Circumference  : {BALL_CIRCUMFERENCE*1000:.1f} mm\n"
    f"Model          : Hollow spherical shell\n"
    f"I / mr²        : {BETA:.4f}   (1+β = {K_ROLL:.4f})"
)
tk.Label(right_panel, text=ball_info, bg=PANEL, fg=MUTED, justify="left",
         font=("Consolas", 10)).pack(anchor="w", padx=18)

# ---- Sensor status ----
tk.Label(right_panel, text="SENSOR STATUS", bg=PANEL, fg=TEXT,
         font=("Segoe UI", 13, "bold")).pack(anchor="w", padx=18, pady=(14, 6))

sensor_row = tk.Frame(right_panel, bg=PANEL)
sensor_row.pack(fill="x", padx=18)
sensor_a_label = tk.Label(sensor_row, text="● SENSOR A", bg=PANEL, fg=MUTED,
                          font=("Segoe UI", 11, "bold"))
sensor_a_label.pack(side="left")
sensor_b_label = tk.Label(sensor_row, text="● SENSOR B", bg=PANEL, fg=MUTED,
                          font=("Segoe UI", 11, "bold"))
sensor_b_label.pack(side="right")

# ---- Live speed gauge ----
tk.Label(right_panel, text="LIVE SPEED", bg=PANEL, fg=TEXT,
         font=("Segoe UI", 13, "bold")).pack(anchor="w", padx=18, pady=(14, 5))

gauge = tk.Canvas(right_panel, width=320, height=145, bg=PANEL,
                  highlightthickness=0)
gauge.pack(fill="x")

def draw_gauge(speed):
    gauge.delete("all")
    width = gauge.winfo_width() or 320
    cx, cy, radius = width / 2, 110, 78
    gauge.create_arc(cx - radius, cy - radius, cx + radius, cy + radius,
                     start=30, extent=120, style="arc", outline="#30363d", width=14)
    ratio = min(max(speed / CFG["gauge_max_ms"], 0), 1)
    color = GREEN if ratio < 0.5 else (ORANGE if ratio < 0.8 else RED)
    gauge.create_arc(cx - radius, cy - radius, cx + radius, cy + radius,
                     start=30, extent=120 * ratio, style="arc", outline=color, width=14)
    gauge.create_text(cx, 70, text=f"{speed:.3f}", fill=TEXT,
                      font=("Segoe UI", 24, "bold"))
    gauge.create_text(cx, 100, text="m/s", fill=MUTED, font=("Segoe UI", 10))

# ---- Stopping-distance input ----
tk.Label(right_panel, text="STOPPING-DISTANCE ANALYSIS", bg=PANEL, fg=TEXT,
         font=("Segoe UI", 12, "bold")).pack(anchor="w", padx=18, pady=(10, 5))
tk.Label(right_panel,
         text="After each roll, measure from Sensor B\nto where the ball stops (metres).",
         bg=PANEL, fg=MUTED, justify="left").pack(anchor="w", padx=18)

stop_entry = tk.Entry(right_panel, bg=PANEL2, fg=TEXT,
                      insertbackground=TEXT, relief="flat")
stop_entry.pack(fill="x", padx=18, pady=(7, 4))

# ---- Buttons ----
button_frame = tk.Frame(right_panel, bg=PANEL)
button_frame.pack(fill="x", padx=18, pady=6)


def log_event(msg):
    event_log.append(msg)
    if len(event_log) > 40:
        event_log.pop(0)
    if session_log_file:
        try:
            with open(session_log_file, "a", newline="") as f:
                csv.writer(f).writerow(
                    [time.strftime("%H:%M:%S"), "EVENT", msg])
        except Exception:
            pass


def reset_experiment():
    """FIX: v1 called update_everything() which did not exist."""
    global live_timer_start
    trials.clear()
    live_timer_start = None
    log_event("EXPERIMENT RESET BY USER")
    if arduino:
        try:
            with serial_lock:
                arduino.write(b"RESET\n")
        except Exception:
            pass
    refresh_ui()


def undo_last_trial():
    """NEW: drop the most recent trial locally (Arduino counter keeps going)."""
    if not trials:
        return
    t = trials.pop()
    log_event(f"UNDO: removed trial {t['trial']} "
              f"({t['speed']:.3f} m/s) from analysis")


def save_stop_distance():
    if not trials:
        messagebox.showwarning("No trial", "Complete a ball roll first.")
        return
    try:
        distance = float(stop_entry.get())
        if distance <= 0:
            raise ValueError
    except ValueError:
        messagebox.showerror("Invalid distance",
                             "Enter a positive distance in metres.\n\nExample: 1.42")
        return
    t = trials[-1]
    t["stop"] = distance
    recompute_crr(t)
    log_event(f"STOP SAVED: trial {t['trial']} s = {distance:.3f} m")
    stop_entry.delete(0, tk.END)
    refresh_ui()


def write_csv_rows(writer):
    writer.writerow(["Trial", "Excluded", "Time_s", "Average_Speed_m_s",
                     "Angular_Speed_rad_s", "Stop_Distance_m", "Effective_Crr",
                     "Deceleration_m_s2", "v_A_m_s", "v_B_m_s",
                     "Translational_Energy_J", "Rotational_Energy_J",
                     "Total_Energy_J", "Energy_Balance_pct", "Note"])
    for t in trials:
        v = t["speed"]; om = v / BALL_RADIUS
        kt = 0.5 * CFG["ball_mass_kg"] * v * v
        kr = 0.5 * BETA * CFG["ball_mass_kg"] * v * v
        dec = va = vb = balance = ""
        if t.get("stop") is not None:
            sol = solve_deceleration(CFG["sensor_distance_m"], t["time"], t["stop"])
            if sol:
                a, va, vb = sol
                dec = a
                # NEW: energy balance check
                ke_a = kinetic_energy(va)
                work = (t["crr"] * CFG["ball_mass_kg"] * G
                        * (CFG["sensor_distance_m"] + t["stop"]))
                balance = 100.0 * work / ke_a if ke_a > 0 else ""
        writer.writerow([t["trial"], "yes" if t["excluded"] else "no",
                         t["time"], v, om,
                         t["stop"] if t["stop"] is not None else "",
                         t["crr"] if t.get("crr") is not None else "",
                         dec, va, vb, kt, kr, kt + kr, balance,
                         t.get("note", "")])


def export_csv():
    if not trials:
        messagebox.showinfo("No data", "No experiment data available.")
        return
    filename = filedialog.asksaveasfilename(
        defaultextension=".csv", filetypes=[("CSV files", "*.csv")],
        title="Save experiment")
    if not filename:
        return
    with open(filename, "w", newline="") as f:
        write_csv_rows(csv.writer(f))
    messagebox.showinfo("Saved", "Experiment exported successfully.")


def export_png():
    """NEW: save the full figure for lab reports."""
    if not trials:
        messagebox.showinfo("No data", "No experiment data available.")
        return
    filename = filedialog.asksaveasfilename(
        defaultextension=".png", filetypes=[("PNG image", "*.png")],
        title="Save graphs as image")
    if not filename:
        return
    fig.savefig(filename, facecolor=BG, dpi=150, bbox_inches="tight")
    messagebox.showinfo("Saved", f"Graph image saved:\n{filename}")


def save_note():
    """NEW: attach a note to the latest trial."""
    if not trials:
        return
    note = note_entry.get().strip()
    trials[-1]["note"] = note
    note_entry.delete(0, tk.END)
    refresh_ui()


for text, cmd, bgc, fgc in (
    ("RESET",      reset_experiment,   PANEL2,  TEXT),
    ("UNDO",       undo_last_trial,    PANEL2,  TEXT),
    ("SAVE STOP",  save_stop_distance, "#147d91", "white"),
    ("EXPORT CSV", export_csv,         PANEL2,  TEXT),
    ("EXPORT PNG", export_png,         PANEL2,  TEXT),
):
    tk.Button(button_frame, text=text, command=cmd, bg=bgc, fg=fgc,
              relief="flat").pack(side="left", fill="x", expand=True, padx=2)

# ---- Note field (NEW) ----
note_row = tk.Frame(right_panel, bg=PANEL)
note_row.pack(fill="x", padx=18, pady=(2, 4))
tk.Label(note_row, text="Note:", bg=PANEL, fg=MUTED,
         font=("Segoe UI", 9)).pack(side="left")
note_entry = tk.Entry(note_row, bg=PANEL2, fg=TEXT,
                      insertbackground=TEXT, relief="flat")
note_entry.pack(side="left", fill="x", expand=True, padx=(4, 4))
tk.Button(note_row, text="ADD", command=save_note, bg=PANEL2, fg=TEXT,
          relief="flat").pack(side="left")

# ---- Physics panel ----
tk.Label(right_panel, text="PHYSICS", bg=PANEL, fg=TEXT,
         font=("Segoe UI", 12, "bold")).pack(anchor="w", padx=18, pady=(12, 5))
physics_label = tk.Label(right_panel, text="Waiting for data...", bg=PANEL,
                         fg=MUTED, justify="left", anchor="nw",
                         font=("Consolas", 9))
physics_label.pack(anchor="w", padx=18)

# ---- Event log ----
tk.Label(right_panel, text="LIVE EVENT LOG", bg=PANEL, fg=TEXT,
         font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=18, pady=(10, 3))
event_text = tk.Text(right_panel, height=5, bg=BG, fg=MUTED, relief="flat",
                     font=("Consolas", 8))
event_text.pack(fill="both", expand=True, padx=18, pady=(0, 10))

# ============================================================
# TABLE  (with right-click exclude/delete - NEW)
# ============================================================

columns = ("trial", "time", "speed", "stop", "crr", "balance", "quality")
table = ttk.Treeview(root, columns=columns, show="headings", height=6)
headings = {"trial": "TRIAL", "time": "TIME (s)", "speed": "SPEED (m/s)",
            "stop": "STOP DIST. (m)", "crr": "EFFECTIVE Crr",
            "balance": "ENERGY BAL. (%)", "quality": "STATUS"}

for column in columns:
    table.heading(column, text=headings[column])
    table.column(column, width=120, anchor="center")
table.pack(fill="x", padx=16, pady=(6, 4))

table_menu = tk.Menu(root, tearoff=0)

def _selected_index():
    sel = table.selection()
    if not sel:
        return None
    row = table.index(sel[0])
    return row if 0 <= row < len(trials) else None

def toggle_exclude_selected():
    i = _selected_index()
    if i is not None:
        trials[i]["excluded"] = not trials[i]["excluded"]
        log_event(f"TRIAL {trials[i]['trial']} "
                  f"{'EXCLUDED' if trials[i]['excluded'] else 'REINSTATED'}")
        refresh_ui()

def delete_selected():
    i = _selected_index()
    if i is not None:
        t = trials.pop(i)
        log_event(f"DELETED trial {t['trial']} from analysis")
        refresh_ui()

def add_stop_to_selected():
    """NEW: assign the typed stop distance to the selected (older) trial."""
    i = _selected_index()
    if i is None:
        return
    try:
        d = float(stop_entry.get())
        if d <= 0:
            raise ValueError
    except ValueError:
        messagebox.showerror("Invalid distance",
                             "Type a positive distance in the box first.")
        return
    trials[i]["stop"] = d
    recompute_crr(trials[i])
    log_event(f"STOP SAVED: trial {trials[i]['trial']} s = {d:.3f} m")
    stop_entry.delete(0, tk.END)
    refresh_ui()

table_menu.add_command(label="Include / Exclude from stats",
                       command=toggle_exclude_selected)
table_menu.add_command(label="Assign stop distance from box",
                       command=add_stop_to_selected)
table_menu.add_command(label="Delete row", command=delete_selected)

def show_table_menu(event):
    iid = table.identify_row(event.y)
    if iid:
        table.selection_set(iid)
        table_menu.tk_popup(event.x_root, event.y_root)

table.bind("<Button-3>", show_table_menu)          # right-click
table.bind("<Button-2>", show_table_menu)          # macOS right-click

# ============================================================
# UI REFRESH  (single funnel - replaces v1's update_everything bug)
# ============================================================

def update_physics_panel():
    if not trials:
        physics_label.config(text="Waiting for first valid trial...")
        return
    t = trials[-1]
    v, tr = t["speed"], t["time"]
    om = v / BALL_RADIUS
    kt = 0.5 * CFG["ball_mass_kg"] * v * v
    kr = BETA * kt
    txt = (f"Average speed       : {v:.5f} m/s\n"
           f"Transit time        : {tr:.5f} s\n"
           f"Angular speed       : {om:.3f} rad/s\n\n"
           f"Translational KE    : {kt:.7f} J\n"
           f"Rotational KE       : {kr:.7f} J\n"
           f"Total rolling KE    : {kt + kr:.7f} J\n")
    if t.get("stop") is not None:
        sol = solve_deceleration(CFG["sensor_distance_m"], tr, t["stop"])
        if sol:
            a, v_a, v_b = sol
            crr = t["crr"]
            force = crr * CFG["ball_mass_kg"] * G
            ke_a = kinetic_energy(v_a)
            work = force * (CFG["sensor_distance_m"] + t["stop"])
            balance = 100.0 * work / ke_a if ke_a > 0 else 0.0
            t_stop = v_b / a
            txt += (f"\nSTOPPING ANALYSIS (trial {t['trial']})\n"
                    f"Distance after B   : {t['stop']:.3f} m\n"
                    f"v at A / B         : {v_a:.5f} / {v_b:.5f} m/s\n"
                    f"Angular speed at B : {v_b / BALL_RADIUS:.3f} rad/s\n"
                    f"Deceleration       : {a:.5f} m/s²\n"
                    f"Effective Crr      : {crr:.6f}\n"
                    f"Rolling force      : {force:.8f} N\n"
                    f"Energy dissipated  : {work:.8f} J\n"
                    f"Energy balance     : {balance:.1f} % (work vs KE at A)\n")
            # NEW: predict stopping distance for the NEXT roll
            mc = mean_valid_crr()
            if mc:
                s_pred = v_a * v_a * K_ROLL / (2 * G * mc) - CFG["sensor_distance_m"]
                if s_pred > 0:
                    txt += f"\nPREDICTION (mean Crr)\nNext stop distance ≈ {s_pred:.3f} m\n"
    if t.get("note"):
        txt += f"\nNote: {t['note']}"
    physics_label.config(text=txt)


def update_table():
    table.delete(*table.get_children())
    stats = calculate_statistics()
    outliers = stats["outliers"] if stats else []
    for t in trials:
        stop = f"{t['stop']:.3f}" if t.get("stop") is not None else ""
        crr = f"{t['crr']:.6f}" if t.get("crr") is not None else ""
        balance = ""
        if t.get("stop") is not None:
            sol = solve_deceleration(CFG["sensor_distance_m"], t["time"], t["stop"])
            if sol:
                a, v_a, _ = sol
                ke_a = kinetic_energy(v_a)
                work = (t["crr"] * CFG["ball_mass_kg"] * G
                        * (CFG["sensor_distance_m"] + t["stop"]))
                balance = (f"{100.0 * work / ke_a:.1f}" if ke_a > 0 else "")
        if t["excluded"]:
            quality = "EXCLUDED"
        elif t["trial"] in outliers:
            quality = "OUTLIER"
        else:
            quality = "VALID"
        tag = "strike" if t["excluded"] else ""
        table.insert("", "end", values=(t["trial"], f"{t['time']:.5f}",
                                        f"{t['speed']:.4f}", stop, crr,
                                        balance, quality))
    table.tag_configure("strike", foreground=MUTED)


def update_graphs():
    for ax in (ax_speed, ax_time, ax_energy, ax_crr):
        ax.clear()
    format_axis(ax_speed,  "REAL-TIME SPEED", "Trial", "m/s")
    format_axis(ax_time,   "TRANSIT TIME",    "Trial", "seconds")
    format_axis(ax_energy, "ROLLING KINETIC ENERGY", "Trial", "Energy (J)")
    format_axis(ax_crr,    "EFFECTIVE Crr",   "Trial", "Crr")
    if not trials:
        canvas.draw_idle()
        return
    allx = [t["trial"] for t in trials]
    inc = included_trials()
    if inc:
        x  = [t["trial"] for t in inc]
        sp = [t["speed"] for t in inc]
        tm = [t["time"]  for t in inc]
        en = [kinetic_energy(t["speed"]) for t in inc]
        ax_speed.plot(x, sp, marker="o", linewidth=2, label="Measured")
        mean = sum(sp) / len(sp)
        ax_speed.axhline(mean, linestyle="--", label="Mean")
        ax_speed.legend(facecolor=PANEL2, labelcolor=TEXT)
        ax_time.plot(x, tm, marker="o", linewidth=2)
        ax_energy.plot(x, en, marker="o", linewidth=2)
    excl = [t for t in trials if t["excluded"]]
    if excl:
        ax_speed.plot([t["trial"] for t in excl], [t["speed"] for t in excl],
                      marker="x", linestyle="none", color=MUTED, label="Excluded")
    cx = [t["trial"] for t in inc if t.get("crr") is not None]
    cy = [t["crr"] for t in inc if t.get("crr") is not None]
    if cy:
        ax_crr.plot(cx, cy, marker="o", linewidth=2)
        m = sum(cy) / len(cy)
        ax_crr.axhline(m, linestyle="--")
        ax_crr.set_ylim(0, max(cy) * 1.4)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    canvas.draw_idle()


def refresh_ui():
    """Single safe refresh funnel (called by buttons AND the serial loop)."""
    global live_transit_time
    if live_timer_start is not None:
        live_transit_time = time.perf_counter() - live_timer_start
    timer_kpi.config(text=f"{live_transit_time:.3f} s")

    # projected speed during transit (NEW)
    if live_timer_start is not None and live_transit_time > 0:
        proj = CFG["sensor_distance_m"] / live_transit_time
        proj_kpi.config(text=f"{proj:.3f} m/s")
        gauge_speed = proj
        # animate ball across the track
        stats = calculate_statistics()
        expected = (CFG["sensor_distance_m"] / stats["mean"]
                    if stats else 1.0)
        draw_track(progress=live_transit_time / max(expected, 0.05))
    else:
        proj_kpi.config(text="—")
        gauge_speed = trials[-1]["speed"] if trials else 0.0
        draw_track(progress=None)

    sensor_a_label.config(fg=CYAN if sensor_a_state else MUTED)
    sensor_b_label.config(fg=ORANGE if sensor_b_state else MUTED)

    color = CYAN
    if experiment_status in ("SENSOR WARNING", "READING REJECTED"):
        color = RED
    elif "COMPLETE" in experiment_status:
        color = GREEN
    status_label.config(text=f"● {experiment_status}", fg=color)

    trial_kpi.config(text=f"{len(trials)} / {CFG['max_trials']}")

    if trials:
        speed_kpi.config(text=f"{trials[-1]['speed']:.3f} m/s")
        stats = calculate_statistics()
        if stats:
            mean_kpi.config(text=f"{stats['mean']:.3f} ± {stats['ci95']:.3f} m/s")
            cv_kpi.config(text=f"{stats['cv']:.2f}%")
    else:
        speed_kpi.config(text="0.000 m/s")
        mean_kpi.config(text="—")
        cv_kpi.config(text="—")

    mc = mean_valid_crr()
    crr_kpi.config(text=f"{mc:.6f}" if mc else "—")

    event_text.delete("1.0", tk.END)
    for e in event_log[-12:]:
        event_text.insert(tk.END, e + "\n")
    event_text.see(tk.END)

    draw_gauge(gauge_speed)
    update_physics_panel()
    update_table()
    update_graphs()


# ============================================================
# SERIAL
# ============================================================

def serial_reader():
    while program_running:
        try:
            with serial_lock:
                dev = arduino
            if dev is None:
                time.sleep(0.2)
                continue
            line = dev.readline()
            if line:
                decoded = line.decode("utf-8", errors="ignore").strip()
                if decoded:
                    serial_queue.put(decoded)
        except Exception:
            time.sleep(0.2)


def connect_arduino():
    global arduino, experiment_status, session_log_file
    port = port_combo.get().strip() or CFG["port"]
    CFG["port"] = port
    save_config(CFG)
    try:
        arduino = serial.Serial(port, CFG["baud"], timeout=0.1)
        time.sleep(2)
        arduino.reset_input_buffer()
        with serial_lock:
            arduino.write(b"RESET\n")
        experiment_status = "CONNECTED"
        connect_button.config(state="disabled")
        if session_log_file is None:
            session_log_file = time.strftime("session_%Y%m%d_%H%M%S.csv")
            with open(session_log_file, "a", newline="") as f:
                csv.writer(f).writerow(
                    ["Time", "Type", "Trial", "Transit_s", "Speed_m_s",
                     "Stop_m", "Crr"])
        log_event(f"CONNECTED to {port} @ {CFG['baud']}")
        threading.Thread(target=serial_reader, daemon=True).start()
    except serial.SerialException as error:
        ports = "\n".join(p.device + " - " + p.description
                          for p in list_ports.comports()) or "none found"
        # FIX: list_ports is now properly imported
        messagebox.showerror(
            "Arduino connection failed",
            f"Could not open {port}.\n\n{error}\n\nAvailable ports:\n{ports}")
    refresh_ui()


def append_session_row(t):
    if not session_log_file:
        return
    try:
        with open(session_log_file, "a", newline="") as f:
            csv.writer(f).writerow([
                time.strftime("%H:%M:%S"), "RESULT", t["trial"],
                f"{t['time']:.5f}", f"{t['speed']:.4f}",
                t["stop"] if t.get("stop") is not None else "",
                f"{t['crr']:.6f}" if t.get("crr") is not None else ""])
    except Exception:
        pass


def process_serial_events():
    global sensor_a_state, sensor_b_state
    global current_trial, experiment_status, live_timer_start
    global last_arduino_trial

    while True:
        try:
            line = serial_queue.get_nowait()
        except queue.Empty:
            break
        if not line:
            continue
        log_event(line)

        if line.startswith("STATE,"):
            try:
                parts = line.split(",")
                sensor_a_state = parts[2] == "1"
                sensor_b_state = parts[4] == "1"
            except Exception:
                pass

        elif line.startswith("A,"):
            try:
                last_arduino_trial = int(line.split(",")[1])
                live_timer_start = time.perf_counter()
                experiment_status = f"TRIAL {last_arduino_trial}: BALL IN MOTION"
            except Exception:
                pass

        elif line.startswith("RESULT,"):
            try:
                _, trial, measured_time, speed = line.split(",")[:4]
                trial = int(trial)
                measured_time = float(measured_time)
                speed = float(speed)
                # Arduino may send ms instead of s
                if measured_time > 100:            # clearly milliseconds
                    measured_time /= 1000.0
                if (measured_time <= 0 or speed <= 0
                        or speed > CFG["max_speed_ms"]):
                    raise ValueError("impossible reading")
                # NEW: cross-check Arduino speed against our geometry
                local_speed = CFG["sensor_distance_m"] / measured_time
                if abs(local_speed - speed) / local_speed > 0.02:
                    log_event(f"WARNING: Arduino speed {speed:.4f} disagrees "
                              f"with d/t = {local_speed:.4f} m/s - "
                              "check SENSOR_DISTANCE in both programs")
                trials.append({"trial": trial, "time": measured_time,
                               "speed": speed, "stop": None, "crr": None,
                               "excluded": False, "note": ""})
                append_session_row(trials[-1])
                live_timer_start = None
                experiment_status = f"TRIAL {trial}: COMPLETE"
                log_event(f"RESULT trial {trial}: {measured_time:.4f} s, "
                          f"{speed:.4f} m/s")
            except (ValueError, IndexError):
                log_event("SOFTWARE REJECTED IMPOSSIBLE READING")

        elif line.startswith("REJECT,"):
            experiment_status = "READING REJECTED"
        elif line.startswith("WARNING,"):
            experiment_status = "SENSOR WARNING"
        elif line == "WAITING_FOR_A":
            experiment_status = "READY — ROLL BALL"
        elif line == "EXPERIMENT_COMPLETE":
            experiment_status = "EXPERIMENT COMPLETE"

    refresh_ui()
    root.after(50, process_serial_events)


# ============================================================
# SHORTCUTS (NEW)
# ============================================================

root.bind("<r>", lambda e: reset_experiment())
root.bind("<u>", lambda e: undo_last_trial())
root.bind("<e>", lambda e: export_csv())
root.bind("<g>", lambda e: export_png())
root.bind("<Escape>", lambda e: close_program())

# ============================================================
# CLEAN EXIT
# ============================================================

def close_program():
    global program_running
    program_running = False
    try:
        if arduino:
            arduino.close()
    except Exception:
        pass
    save_config(CFG)
    root.destroy()


root.protocol("WM_DELETE_WINDOW", close_program)

# ============================================================
# START
# ============================================================

root.after(200, scan_ports)
root.after(400, connect_arduino)
root.after(500, process_serial_events)

root.mainloop()