# ============================================================
#                  ROLLING MOTION LAB
#
#       Real-Time Table Tennis Ball Analyzer
#
# Hardware:
#   Arduino UNO
#   2 IR sensors
#
# Physics model:
#   Hollow spherical shell
#
# ============================================================


import tkinter as tk
from tkinter import ttk, messagebox, filedialog

import serial
import threading
import queue
import csv
import math
import time

import numpy as np

from matplotlib.figure import Figure

from matplotlib.backends.backend_tkagg import (
    FigureCanvasTkAgg
)


# ============================================================
# USER SETTINGS
# ============================================================

PORT = "COM6"

BAUD_RATE = 115200


# ------------------------------------------------------------
# Sensor geometry
# ------------------------------------------------------------

SENSOR_DISTANCE = 0.15      # m


# ------------------------------------------------------------
# TABLE TENNIS BALL
# ------------------------------------------------------------

BALL_DIAMETER = 0.040         # m
BALL_RADIUS = 0.020           # m
BALL_MASS = 0.0027             # kg
BALL_CIRCUMFERENCE = 0.1256   # m


# ------------------------------------------------------------
# Hollow spherical shell
#
# I = (2/3) m r²
# ------------------------------------------------------------

INERTIA_FACTOR = 2.0 / 3.0


# ------------------------------------------------------------
# Physics constant
# ------------------------------------------------------------

G = 9.81


# ------------------------------------------------------------
# Experiment
# ------------------------------------------------------------

MAX_TRIALS = 10


# ============================================================
# DATA
# ============================================================

trial_numbers = []

times = []

average_speeds = []

stop_distances = []

effective_crr = []

event_log = []


# ============================================================
# REAL-TIME STATE
# ============================================================

sensor_a_state = False

sensor_b_state = False

current_trial = 0

experiment_status = "DISCONNECTED"

live_timer_start = None

live_transit_time = 0.0


# ============================================================
# SERIAL
# ============================================================

arduino = None

serial_queue = queue.Queue()

program_running = True


# ============================================================
# TKINTER
# ============================================================

root = tk.Tk()

root.title(
    "Rolling Motion Laboratory"
)

root.geometry(
    "1600x950"
)

root.configure(
    bg="#0d1117"
)

root.minsize(
    1200,
    750
)


# ============================================================
# COLORS
# ============================================================

BG = "#0d1117"

PANEL = "#161b22"

PANEL2 = "#1c2128"

TEXT = "#f0f6fc"

MUTED = "#8b949e"

CYAN = "#39d0ff"

GREEN = "#3fb950"

ORANGE = "#d29922"

RED = "#f85149"


# ============================================================
# TK STYLE
# ============================================================

style = ttk.Style()

style.theme_use("clam")

style.configure(
    "Treeview",
    background=PANEL,
    foreground=TEXT,
    fieldbackground=PANEL,
    rowheight=27,
    borderwidth=0
)

style.configure(
    "Treeview.Heading",
    background=PANEL2,
    foreground=TEXT,
    font=("Segoe UI", 10, "bold")
)

style.map(
    "Treeview",
    background=[
        ("selected", "#30363d")
    ]
)


# ============================================================
# HEADER
# ============================================================

header = tk.Frame(
    root,
    bg=PANEL,
    height=70
)

header.pack(
    fill="x"
)


tk.Label(
    header,
    text="ROLLING MOTION LABORATORY",
    bg=PANEL,
    fg=TEXT,
    font=("Segoe UI", 24, "bold")
).pack(
    side="left",
    padx=25
)


status_label = tk.Label(
    header,
    text="● DISCONNECTED",
    bg=PANEL,
    fg=RED,
    font=("Segoe UI", 12, "bold")
)

status_label.pack(
    side="right",
    padx=25
)


# ============================================================
# KPI AREA
# ============================================================

kpi_frame = tk.Frame(
    root,
    bg=BG
)

kpi_frame.pack(
    fill="x",
    padx=16,
    pady=10
)


def create_kpi(title, initial):

    card = tk.Frame(
        kpi_frame,
        bg=PANEL2,
        highlightbackground="#30363d",
        highlightthickness=1
    )

    card.pack(
        side="left",
        fill="both",
        expand=True,
        padx=5
    )


    tk.Label(
        card,
        text=title,
        bg=PANEL2,
        fg=MUTED,
        font=("Segoe UI", 10)
    ).pack(
        anchor="w",
        padx=15,
        pady=(10, 2)
    )


    value = tk.Label(
        card,
        text=initial,
        bg=PANEL2,
        fg=TEXT,
        font=("Segoe UI", 21, "bold")
    )

    value.pack(
        anchor="w",
        padx=15,
        pady=(0, 10)
    )


    return value


trial_kpi = create_kpi(
    "TRIAL",
    "0 / 10"
)

timer_kpi = create_kpi(
    "LIVE A → B TIMER",
    "0.000 s"
)

speed_kpi = create_kpi(
    "AVERAGE SPEED",
    "0.000 m/s"
)

mean_kpi = create_kpi(
    "RUNNING MEAN",
    "0.000 m/s"
)

cv_kpi = create_kpi(
    "CONSISTENCY",
    "—"
)

crr_kpi = create_kpi(
    "EFFECTIVE Crr",
    "—"
)


# ============================================================
# MAIN AREA
# ============================================================

main_frame = tk.Frame(
    root,
    bg=BG
)

main_frame.pack(
    fill="both",
    expand=True,
    padx=16
)


# ============================================================
# GRAPH AREA
# ============================================================

graph_frame = tk.Frame(
    main_frame,
    bg=BG
)

graph_frame.pack(
    side="left",
    fill="both",
    expand=True
)


fig = Figure(
    figsize=(10, 6),
    dpi=100,
    facecolor=BG
)


ax_speed = fig.add_subplot(221)

ax_time = fig.add_subplot(222)

ax_energy = fig.add_subplot(223)

ax_crr = fig.add_subplot(224)


# ============================================================
# GRAPH FORMAT
# ============================================================

def format_axis(
    ax,
    title,
    xlabel,
    ylabel
):

    ax.set_facecolor(PANEL)

    ax.set_title(
        title,
        color=TEXT,
        fontsize=11,
        fontweight="bold"
    )

    ax.set_xlabel(
        xlabel,
        color=MUTED
    )

    ax.set_ylabel(
        ylabel,
        color=MUTED
    )

    ax.tick_params(
        colors=MUTED
    )

    for spine in ax.spines.values():

        spine.set_color(
            "#30363d"
        )

    ax.grid(
        alpha=0.18
    )


canvas = FigureCanvasTkAgg(
    fig,
    master=graph_frame
)

canvas.get_tk_widget().pack(
    fill="both",
    expand=True
)


# ============================================================
# RIGHT PANEL
# ============================================================

right_panel = tk.Frame(
    main_frame,
    bg=PANEL,
    width=380
)

right_panel.pack(
    side="right",
    fill="y",
    padx=(14, 0)
)

right_panel.pack_propagate(
    False
)


# ============================================================
# BALL INFORMATION
# ============================================================

tk.Label(
    right_panel,
    text="TABLE TENNIS BALL",
    bg=PANEL,
    fg=TEXT,
    font=("Segoe UI", 13, "bold")
).pack(
    anchor="w",
    padx=18,
    pady=(15, 7)
)


ball_info = (
    "Diameter       : 40.0 mm\n"
    "Radius         : 20.0 mm\n"
    "Mass           : 2.70 g\n"
    "Circumference  : 125.6 mm\n"
    "Model          : Hollow spherical shell\n"
    "I / mr²        : 0.6667\n"
)

tk.Label(
    right_panel,
    text=ball_info,
    bg=PANEL,
    fg=MUTED,
    justify="left",
    font=("Consolas", 10)
).pack(
    anchor="w",
    padx=18
)


# ============================================================
# SENSOR STATUS
# ============================================================

tk.Label(
    right_panel,
    text="SENSOR STATUS",
    bg=PANEL,
    fg=TEXT,
    font=("Segoe UI", 13, "bold")
).pack(
    anchor="w",
    padx=18,
    pady=(18, 6)
)


sensor_row = tk.Frame(
    right_panel,
    bg=PANEL
)

sensor_row.pack(
    fill="x",
    padx=18
)


sensor_a_label = tk.Label(
    sensor_row,
    text="● SENSOR A",
    bg=PANEL,
    fg=MUTED,
    font=("Segoe UI", 11, "bold")
)

sensor_a_label.pack(
    side="left"
)


sensor_b_label = tk.Label(
    sensor_row,
    text="● SENSOR B",
    bg=PANEL,
    fg=MUTED,
    font=("Segoe UI", 11, "bold")
)

sensor_b_label.pack(
    side="right"
)


# ============================================================
# LIVE GAUGE
# ============================================================

tk.Label(
    right_panel,
    text="LIVE SPEED",
    bg=PANEL,
    fg=TEXT,
    font=("Segoe UI", 13, "bold")
).pack(
    anchor="w",
    padx=18,
    pady=(18, 5)
)


gauge = tk.Canvas(
    right_panel,
    width=320,
    height=145,
    bg=PANEL,
    highlightthickness=0
)

gauge.pack(
    fill="x"
)


def draw_gauge(speed):

    gauge.delete(
        "all"
    )

    width = gauge.winfo_width()

    if width < 100:
        width = 320

    cx = width / 2

    cy = 110

    radius = 78


    gauge.create_arc(
        cx - radius,
        cy - radius,
        cx + radius,
        cy + radius,
        start=30,
        extent=120,
        style="arc",
        outline="#30363d",
        width=14
    )


    # 0–3 m/s display range

    ratio = min(
        max(speed / 3.0, 0),
        1
    )


    gauge.create_arc(
        cx - radius,
        cy - radius,
        cx + radius,
        cy + radius,
        start=30,
        extent=120 * ratio,
        style="arc",
        outline=CYAN,
        width=14
    )


    gauge.create_text(
        cx,
        70,
        text=f"{speed:.3f}",
        fill=TEXT,
        font=("Segoe UI", 24, "bold")
    )


    gauge.create_text(
        cx,
        100,
        text="m/s",
        fill=MUTED,
        font=("Segoe UI", 10)
    )


# ============================================================
# INPUT FIELDS
# ============================================================

tk.Label(
    right_panel,
    text="STOPPING-DISTANCE ANALYSIS",
    bg=PANEL,
    fg=TEXT,
    font=("Segoe UI", 12, "bold")
).pack(
    anchor="w",
    padx=18,
    pady=(10, 5)
)


tk.Label(
    right_panel,
    text=(
        "After each roll, measure the distance from\n"
        "Sensor B to where the ball stops."
    ),
    bg=PANEL,
    fg=MUTED,
    justify="left"
).pack(
    anchor="w",
    padx=18
)


stop_entry = tk.Entry(
    right_panel,
    bg=PANEL2,
    fg=TEXT,
    insertbackground=TEXT,
    relief="flat"
)

stop_entry.pack(
    fill="x",
    padx=18,
    pady=(7, 4)
)

stop_entry.insert(
    0,
    ""
)


# ============================================================
# BUTTONS
# ============================================================

button_frame = tk.Frame(
    right_panel,
    bg=PANEL
)

button_frame.pack(
    fill="x",
    padx=18,
    pady=6
)


def reset_experiment():

    global current_trial
    global experiment_status
    global live_timer_start

    trial_numbers.clear()

    times.clear()

    average_speeds.clear()

    stop_distances.clear()

    effective_crr.clear()

    event_log.clear()

    current_trial = 0

    live_timer_start = None

    experiment_status = "RESETTING"


    if arduino:

        try:

            arduino.write(
                b"RESET\n"
            )

        except Exception:
            pass

    update_everything()


def save_stop_distance():

    if not average_speeds:

        messagebox.showwarning(
            "No trial",
            "Complete a ball roll first."
        )

        return


    try:

        distance = float(
            stop_entry.get()
        )

        if distance <= 0:
            raise ValueError

    except ValueError:

        messagebox.showerror(
            "Invalid distance",
            "Enter a positive distance in metres.\n\n"
            "Example: 1.42"
        )

        return


    index = len(
        average_speeds
    ) - 1


    while len(
        stop_distances
    ) < len(average_speeds):

        stop_distances.append(
            None
        )


    stop_distances[index] = distance


    calculate_crr()


    update_everything()


    stop_entry.delete(
        0,
        tk.END
    )


reset_button = tk.Button(
    button_frame,
    text="RESET",
    command=reset_experiment,
    bg=PANEL2,
    fg=TEXT,
    relief="flat"
)

reset_button.pack(
    side="left",
    fill="x",
    expand=True,
    padx=3
)


save_button = tk.Button(
    button_frame,
    text="SAVE STOP",
    command=save_stop_distance,
    bg="#147d91",
    fg="white",
    relief="flat"
)

save_button.pack(
    side="left",
    fill="x",
    expand=True,
    padx=3
)


def export_csv():

    if not trial_numbers:

        messagebox.showinfo(
            "No data",
            "No experiment data available."
        )

        return


    filename = filedialog.asksaveasfilename(
        defaultextension=".csv",
        filetypes=[
            (
                "CSV files",
                "*.csv"
            )
        ],
        title="Save experiment"
    )


    if not filename:

        return


    with open(
        filename,
        "w",
        newline=""
    ) as file:

        writer = csv.writer(
            file
        )


        writer.writerow([
            "Trial",
            "Time_s",
            "Average_Speed_m_s",
            "Stop_Distance_m",
            "Effective_Crr",
            "Angular_Speed_rad_s",
            "Translational_Energy_J",
            "Rotational_Energy_J",
            "Total_Energy_J"
        ])


        for i in range(
            len(trial_numbers)
        ):

            speed = average_speeds[i]

            omega = (
                speed /
                BALL_RADIUS
            )


            translational_energy = (
                0.5 *
                BALL_MASS *
                speed ** 2
            )


            rotational_energy = (
                0.5 *
                (
                    INERTIA_FACTOR *
                    BALL_MASS *
                    BALL_RADIUS ** 2
                )
                *
                omega ** 2
            )


            total_energy = (
                translational_energy +
                rotational_energy
            )


            stop = (
                stop_distances[i]
                if i < len(stop_distances)
                else ""
            )


            crr = (
                effective_crr[i]
                if i < len(effective_crr)
                else ""
            )


            writer.writerow([
                trial_numbers[i],
                times[i],
                speed,
                stop,
                crr,
                omega,
                translational_energy,
                rotational_energy,
                total_energy
            ])


    messagebox.showinfo(
        "Saved",
        "Experiment exported successfully."
    )


export_button = tk.Button(
    button_frame,
    text="EXPORT",
    command=export_csv,
    bg=PANEL2,
    fg=TEXT,
    relief="flat"
)

export_button.pack(
    side="left",
    fill="x",
    expand=True,
    padx=3
)


# ============================================================
# PHYSICS ANALYSIS
# ============================================================

tk.Label(
    right_panel,
    text="PHYSICS",
    bg=PANEL,
    fg=TEXT,
    font=("Segoe UI", 12, "bold")
).pack(
    anchor="w",
    padx=18,
    pady=(13, 5)
)


physics_label = tk.Label(
    right_panel,
    text="Waiting for data...",
    bg=PANEL,
    fg=MUTED,
    justify="left",
    anchor="nw",
    font=("Consolas", 9)
)

physics_label.pack(
    anchor="w",
    padx=18
)


# ============================================================
# EVENT LOG
# ============================================================

tk.Label(
    right_panel,
    text="LIVE EVENT LOG",
    bg=PANEL,
    fg=TEXT,
    font=("Segoe UI", 11, "bold")
).pack(
    anchor="w",
    padx=18,
    pady=(10, 3)
)


event_text = tk.Text(
    right_panel,
    height=5,
    bg="#0d1117",
    fg=MUTED,
    relief="flat",
    font=("Consolas", 8)
)

event_text.pack(
    fill="both",
    padx=18
)


# ============================================================
# STATISTICS
# ============================================================

def calculate_statistics():

    if not average_speeds:

        return None


    data = np.array(
        average_speeds,
        dtype=float
    )


    mean = np.mean(
        data
    )


    std = (
        np.std(
            data,
            ddof=1
        )
        if len(data) > 1
        else 0
    )


    sem = (
        std /
        math.sqrt(
            len(data)
        )
        if len(data) > 1
        else 0
    )


    ci95 = 1.96 * sem


    median = np.median(
        data
    )


    minimum = np.min(
        data
    )


    maximum = np.max(
        data
    )


    cv = (
        100 *
        std /
        mean
        if mean != 0
        else 0
    )


    q1 = np.percentile(
        data,
        25
    )


    q3 = np.percentile(
        data,
        75
    )


    iqr = q3 - q1


    low = q1 - 1.5 * iqr

    high = q3 + 1.5 * iqr


    outliers = []

    for i, value in enumerate(
        data
    ):

        if (
            value < low
            or
            value > high
        ):

            outliers.append(
                i + 1
            )


    return {
        "mean": mean,
        "std": std,
        "sem": sem,
        "ci95": ci95,
        "median": median,
        "min": minimum,
        "max": maximum,
        "cv": cv,
        "outliers": outliers
    }


# ============================================================
# Crr CALCULATION
# ============================================================

def calculate_crr():

    while len(
        effective_crr
    ) < len(average_speeds):

        effective_crr.append(
            None
        )


    for i in range(
        len(average_speeds)
    ):

        if (
            i >= len(stop_distances)
            or
            stop_distances[i] is None
        ):

            effective_crr[i] = None

            continue


        d = SENSOR_DISTANCE

        s = stop_distances[i]

        t = times[i]


        if (
            s <= 0
            or t <= 0
        ):

            effective_crr[i] = None

            continue


        # ----------------------------------------------------
        # Constant-deceleration model:
        #
        # d = v_B*t + 1/2*a*t²
        #
        # stopping distance after B:
        #
        # s = v_B²/(2a)
        #
        # Solve for sqrt(a)
        # ----------------------------------------------------


        A = 0.5 * t ** 2

        B = t * math.sqrt(
            2 * s
        )

        C = -d


        discriminant = (
            B ** 2
            - 4 * A * C
        )


        if discriminant <= 0:

            effective_crr[i] = None

            continue


        sqrt_a = (
            -B +
            math.sqrt(
                discriminant
            )
        ) / (
            2 * A
        )


        acceleration = (
            sqrt_a ** 2
        )


        # For hollow spherical shell:
        #
        # Crr = (1 + I/mr²) * a/g
        #
        # 1 + 2/3 = 5/3
        #

        k = (
            1 +
            INERTIA_FACTOR
        )


        crr = (
            k *
            acceleration /
            G
        )


        effective_crr[i] = crr


# ============================================================
# CURRENT PHYSICS
# ============================================================

def update_physics():

    if not average_speeds:

        physics_label.config(
            text="Waiting for first valid trial..."
        )

        return


    i = len(
        average_speeds
    ) - 1


    v_avg = average_speeds[i]

    t = times[i]


    # --------------------------------------------------------
    # Average angular speed
    # --------------------------------------------------------

    omega_avg = (
        v_avg /
        BALL_RADIUS
    )


    # --------------------------------------------------------
    # Energies based on measured average speed
    # --------------------------------------------------------

    translational = (
        0.5 *
        BALL_MASS *
        v_avg ** 2
    )


    rotational = (
        0.5 *
        (
            INERTIA_FACTOR *
            BALL_MASS *
            BALL_RADIUS ** 2
        )
        *
        omega_avg ** 2
    )


    total_energy = (
        translational +
        rotational
    )


    text = (
        f"Average speed       : {v_avg:.5f} m/s\n"
        f"Transit time        : {t:.5f} s\n"
        f"Angular speed       : {omega_avg:.3f} rad/s\n"
        f"\n"
        f"Translational KE    : {translational:.7f} J\n"
        f"Rotational KE       : {rotational:.7f} J\n"
        f"Total rolling KE    : {total_energy:.7f} J\n"
    )


    # --------------------------------------------------------
    # Stopping analysis
    # --------------------------------------------------------

    if (
        i < len(stop_distances)
        and
        stop_distances[i] is not None
    ):

        s = stop_distances[i]


        # Solve acceleration as described above.

        A = 0.5 * t ** 2

        B = t * math.sqrt(
            2 * s
        )

        C = -SENSOR_DISTANCE


        discriminant = (
            B ** 2
            - 4 * A * C
        )


        if discriminant > 0:

            sqrt_a = (
                -B +
                math.sqrt(
                    discriminant
                )
            ) / (
                2 * A
            )


            acceleration = (
                sqrt_a ** 2
            )


            v_b = math.sqrt(
                2 *
                acceleration *
                s
            )


            omega_b = (
                v_b /
                BALL_RADIUS
            )


            crr = (
                (
                    1 +
                    INERTIA_FACTOR
                )
                *
                acceleration /
                G
            )


            friction_force = (
                crr *
                BALL_MASS *
                G
            )


            stopping_energy = (
                friction_force *
                s
            )


            stopping_time = (
                v_b /
                acceleration
            )


            power_loss = (
                stopping_energy /
                stopping_time
                if stopping_time > 0
                else 0
            )


            text += (
                f"\n"
                f"STOPPING ANALYSIS\n"
                f"Distance after B   : {s:.3f} m\n"
                f"Estimated v at B   : {v_b:.5f} m/s\n"
                f"Angular speed at B : {omega_b:.3f} rad/s\n"
                f"Deceleration       : {acceleration:.5f} m/s²\n"
                f"Effective Crr      : {crr:.6f}\n"
                f"Rolling force      : {friction_force:.8f} N\n"
                f"Energy dissipated  : {stopping_energy:.8f} J\n"
                f"Approx. power loss : {power_loss:.8f} W\n"
            )


    physics_label.config(
        text=text
    )


# ============================================================
# UPDATE TABLE
# ============================================================

columns = (
    "trial",
    "time",
    "speed",
    "stop",
    "crr",
    "quality"
)


table = ttk.Treeview(
    root,
    columns=columns,
    show="headings",
    height=6
)


headings = {
    "trial": "TRIAL",
    "time": "TIME (s)",
    "speed": "SPEED (m/s)",
    "stop": "STOP DIST. (m)",
    "crr": "EFFECTIVE Crr",
    "quality": "QUALITY"
}


for column in columns:

    table.heading(
        column,
        text=headings[column]
    )

    table.column(
        column,
        width=130,
        anchor="center"
    )


table.pack(
    fill="x",
    padx=16,
    pady=(6, 10)
)


def update_table():

    for item in table.get_children():

        table.delete(
            item
        )


    stats = calculate_statistics()

    outliers = (
        stats["outliers"]
        if stats
        else []
    )


    for i in range(
        len(trial_numbers)
    ):

        stop = ""


        if (
            i < len(stop_distances)
            and
            stop_distances[i] is not None
        ):

            stop = (
                f"{stop_distances[i]:.3f}"
            )


        crr = ""


        if (
            i < len(effective_crr)
            and
            effective_crr[i] is not None
        ):

            crr = (
                f"{effective_crr[i]:.6f}"
            )


        quality = "VALID"


        if (
            i + 1
            in outliers
        ):

            quality = "OUTLIER"


        table.insert(
            "",
            "end",
            values=(
                trial_numbers[i],
                f"{times[i]:.5f}",
                f"{average_speeds[i]:.4f}",
                stop,
                crr,
                quality
            )
        )


# ============================================================
# UPDATE GRAPHS
# ============================================================

def update_graphs():

    for ax in (
        ax_speed,
        ax_time,
        ax_energy,
        ax_crr
    ):

        ax.clear()


    format_axis(
        ax_speed,
        "REAL-TIME SPEED",
        "Trial",
        "m/s"
    )


    format_axis(
        ax_time,
        "TRANSIT TIME",
        "Trial",
        "seconds"
    )


    format_axis(
        ax_energy,
        "ROLLING KINETIC ENERGY",
        "Trial",
        "Energy (J)"
    )


    format_axis(
        ax_crr,
        "EFFECTIVE Crr",
        "Trial",
        "Crr"
    )


    if not trial_numbers:

        canvas.draw_idle()

        return


    x = trial_numbers


    # --------------------------------------------------------
    # SPEED
    # --------------------------------------------------------

    ax_speed.plot(
        x,
        average_speeds,
        marker="o",
        linewidth=2,
        label="Measured"
    )


    mean_speed = np.mean(
        average_speeds
    )


    ax_speed.axhline(
        mean_speed,
        linestyle="--",
        label="Mean"
    )


    ax_speed.legend(
        facecolor=PANEL2,
        labelcolor=TEXT
    )


    # --------------------------------------------------------
    # TIME
    # --------------------------------------------------------

    ax_time.plot(
        x,
        times,
        marker="o",
        linewidth=2
    )


    # --------------------------------------------------------
    # ENERGY
    # --------------------------------------------------------

    energies = []


    for speed in average_speeds:

        omega = (
            speed /
            BALL_RADIUS
        )


        translational = (
            0.5 *
            BALL_MASS *
            speed ** 2
        )


        rotational = (
            0.5 *
            (
                INERTIA_FACTOR *
                BALL_MASS *
                BALL_RADIUS ** 2
            )
            *
            omega ** 2
        )


        energies.append(
            translational +
            rotational
        )


    ax_energy.plot(
        x,
        energies,
        marker="o",
        linewidth=2
    )


    # --------------------------------------------------------
    # Crr
    # --------------------------------------------------------

    crr_x = []

    crr_y = []


    for i, value in enumerate(
        effective_crr
    ):

        if value is not None:

            crr_x.append(
                trial_numbers[i]
            )

            crr_y.append(
                value
            )


    if crr_y:

        ax_crr.plot(
            crr_x,
            crr_y,
            marker="o",
            linewidth=2
        )


    fig.tight_layout(
        rect=[
            0,
            0,
            1,
            0.96
        ]
    )


    canvas.draw_idle()


# ============================================================
# EVENT PROCESSING
# ============================================================

def process_serial_events():

    global sensor_a_state
    global sensor_b_state
    global current_trial
    global experiment_status
    global live_timer_start
    global live_transit_time


    while True:

        try:

            line = serial_queue.get_nowait()

        except queue.Empty:

            break


        if not line:

            continue


        event_log.append(
            line
        )


        if len(event_log) > 30:

            event_log.pop(0)


        # ----------------------------------------------------
        # SENSOR STATE
        # ----------------------------------------------------

        if line.startswith(
            "STATE,"
        ):

            parts = line.split(",")


            try:

                # STATE,A,0,B,1

                sensor_a_state = (
                    parts[2] == "1"
                )


                sensor_b_state = (
                    parts[4] == "1"
                )


            except Exception:

                pass


        # ----------------------------------------------------
        # A EVENT
        # ----------------------------------------------------

        elif line.startswith(
            "A,"
        ):

            parts = line.split(",")


            try:

                current_trial = int(
                    parts[1]
                )

                live_timer_start = (
                    time.perf_counter()
                )

                live_transit_time = 0

                experiment_status = (
                    f"TRIAL {current_trial}: BALL IN MOTION"
                )


            except Exception:

                pass


        # ----------------------------------------------------
        # RESULT
        # ----------------------------------------------------

        elif line.startswith(
            "RESULT,"
        ):

            parts = line.split(",")


            try:

                trial = int(
                    parts[1]
                )


                measured_time = float(
                    parts[2]
                )


                speed = float(
                    parts[3]
                )


                # ------------------------------------------------
                # Safety filter
                #
                # A table-tennis ball should not be producing
                # absurd numbers such as 170 m/s in this setup.
                # ------------------------------------------------

                if (
                    measured_time <= 0
                    or
                    speed <= 0
                    or
                    speed > 5
                ):

                    event_log.append(
                        "SOFTWARE REJECTED IMPOSSIBLE READING"
                    )

                else:

                    trial_numbers.append(
                        trial
                    )


                    times.append(
                        measured_time
                    )


                    average_speeds.append(
                        speed
                    )


                    stop_distances.append(
                        None
                    )


                    effective_crr.append(
                        None
                    )


                    live_timer_start = None


                    experiment_status = (
                        f"TRIAL {trial}: COMPLETE"
                    )


                    calculate_crr()


            except ValueError:

                pass


        # ----------------------------------------------------
        # REJECTION
        # ----------------------------------------------------

        elif line.startswith(
            "REJECT,"
        ):

            experiment_status = (
                "READING REJECTED"
            )


        # ----------------------------------------------------
        # WARNING
        # ----------------------------------------------------

        elif line.startswith(
            "WARNING,"
        ):

            experiment_status = (
                "SENSOR WARNING"
            )


        elif line == "WAITING_FOR_A":

            experiment_status = (
                "READY — ROLL BALL"
            )


        elif line == "EXPERIMENT_COMPLETE":

            experiment_status = (
                "EXPERIMENT COMPLETE"
            )


    # ========================================================
    # LIVE TIMER
    # ========================================================

    if live_timer_start is not None:

        live_transit_time = (
            time.perf_counter()
            -
            live_timer_start
        )


    timer_kpi.config(
        text=f"{live_transit_time:.3f} s"
    )


    # ========================================================
    # SENSOR DISPLAY
    # ========================================================

    sensor_a_label.config(
        fg=CYAN
        if sensor_a_state
        else MUTED
    )


    sensor_b_label.config(
        fg=ORANGE
        if sensor_b_state
        else MUTED
    )


    # ========================================================
    # STATUS DISPLAY
    # ========================================================

    status_color = CYAN


    if (
        experiment_status
        in (
            "SENSOR WARNING",
            "READING REJECTED"
        )
    ):

        status_color = RED


    elif (
        "COMPLETE"
        in experiment_status
    ):

        status_color = GREEN


    status_label.config(
        text=f"● {experiment_status}",
        fg=status_color
    )


    # ========================================================
    # KPIs
    # ========================================================

    trial_kpi.config(
        text=f"{len(trial_numbers)} / {MAX_TRIALS}"
    )


    if average_speeds:

        latest_speed = (
            average_speeds[-1]
        )


        mean_speed = np.mean(
            average_speeds
        )


        speed_kpi.config(
            text=f"{latest_speed:.3f} m/s"
        )


        mean_kpi.config(
            text=f"{mean_speed:.3f} m/s"
        )


        stats = calculate_statistics()


        if stats:

            cv_kpi.config(
                text=f"{stats['cv']:.2f}%"
            )


    valid_crr = [
        x
        for x in effective_crr
        if x is not None
    ]


    if valid_crr:

        crr_kpi.config(
            text=f"{np.mean(valid_crr):.6f}"
        )

    else:

        crr_kpi.config(
            text="—"
        )


    # ========================================================
    # EVENT LOG
    # ========================================================

    event_text.delete(
        "1.0",
        tk.END
    )


    for event in event_log[-12:]:

        event_text.insert(
            tk.END,
            event + "\n"
        )


    event_text.see(
        tk.END
    )


    draw_gauge(
        average_speeds[-1]
        if average_speeds
        else 0
    )


    update_physics()

    update_table()

    update_graphs()


    root.after(
        50,
        process_serial_events
    )


# ============================================================
# SERIAL THREAD
# ============================================================

def serial_reader():

    global program_running


    while program_running:

        try:

            line = arduino.readline()


            if line:

                decoded = line.decode(
                    "utf-8",
                    errors="ignore"
                ).strip()


                if decoded:

                    serial_queue.put(
                        decoded
                    )


        except Exception:

            break


# ============================================================
# CONNECT TO ARDUINO
# ============================================================

def connect_arduino():

    global arduino
    global experiment_status


    try:

        arduino = serial.Serial(
            PORT,
            BAUD_RATE,
            timeout=0.1
        )


        time.sleep(2)


        arduino.reset_input_buffer()


        arduino.write(
            b"RESET\n"
        )


        experiment_status = (
            "CONNECTED"
        )


        thread = threading.Thread(
            target=serial_reader,
            daemon=True
        )


        thread.start()


    except serial.SerialException as error:

        ports = serial.tools.list_ports.comports()


        available = "\n".join(
            p.device
            + " - "
            + p.description
            for p in ports
        )


        messagebox.showerror(
            "Arduino connection failed",
            f"Could not open {PORT}.\n\n"
            f"{error}\n\n"
            f"Available ports:\n"
            f"{available}"
        )


# ============================================================
# LIVE TIMER UPDATE
# ============================================================

def timer_loop():

    global live_transit_time


    if live_timer_start is not None:

        live_transit_time = (
            time.perf_counter()
            -
            live_timer_start
        )


        timer_kpi.config(
            text=f"{live_transit_time:.3f} s"
        )


    root.after(
        20,
        timer_loop
    )


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


    root.destroy()


root.protocol(
    "WM_DELETE_WINDOW",
    close_program
)


# ============================================================
# START
# ============================================================

root.after(
    200,
    connect_arduino
)

root.after(
    300,
    process_serial_events
)

root.after(
    300,
    timer_loop
)


root.mainloop()