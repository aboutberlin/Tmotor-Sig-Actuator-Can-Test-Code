#!/usr/bin/env python3
"""
Plot 6 IMU angles and velocities from walking_log CSV.

Usage:
    python plot_imu.py                          # default file & order
    python plot_imu.py walking_log_002.csv      # specify file
    python plot_imu.py -o R L 1 2 3 4           # custom plot order (top to bottom)
    python plot_imu.py -s 5 -d 10               # start=5s, duration=10s

Options:
    -f / --file       CSV filename (in same folder, or full path)
    -s / --start      Start time in seconds (default: interactive picker)
    -d / --duration   Window duration in seconds (default: 10)
    -o / --order      IMU plot order, e.g. R L 1 2 3 4
"""

import argparse
import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# ---------- column name mapping ----------
IMU_INFO = {
    "R": {"angle": "imu_RTx",             "vel": "imu_Rvel",  "label": "R (right thigh)"},
    "L": {"angle": "imu_LTx",             "vel": "imu_Lvel",  "label": "L (left thigh)"},
    "1": {"angle": "imu_1_left_shaking",   "vel": "imu_vel_1", "label": "IMU1 (left shank)"},
    "2": {"angle": "imu_2_right_shaking",  "vel": "imu_vel_2", "label": "IMU2 (right shank)"},
    "3": {"angle": "imu_3_left_foot",      "vel": "imu_vel_3", "label": "IMU3 (left foot)"},
    "4": {"angle": "imu_4_right_foot",     "vel": "imu_vel_4", "label": "IMU4 (right foot)"},
}
DEFAULT_ORDER = ["R", "L", "1", "2", "3", "4"]
COLORS = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3", "#ff7f00", "#a65628"]


def load_csv(path):
    df = pd.read_csv(path)
    df["time_s"] = (df["Time_ms"] - df["Time_ms"].iloc[0]) / 1000.0
    return df


def plot_window(df, t_start, duration, order):
    mask = (df["time_s"] >= t_start) & (df["time_s"] < t_start + duration)
    seg = df.loc[mask]
    if seg.empty:
        print(f"No data in [{t_start:.1f}, {t_start + duration:.1f}] s")
        return

    t = seg["time_s"].values

    fig, (ax_ang, ax_vel) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)

    for i, key in enumerate(order):
        info = IMU_INFO[key]
        c = COLORS[i % len(COLORS)]
        ax_ang.plot(t, seg[info["angle"]].values, label=info["label"], color=c, linewidth=1.2)
        ax_vel.plot(t, seg[info["vel"]].values,   label=info["label"], color=c, linewidth=1.2)

    ax_ang.set_ylabel("Angle (deg)")
    ax_ang.set_title(f"6-IMU Angles  [{t_start:.1f} ~ {t_start + duration:.1f} s]")
    ax_ang.legend(loc="upper right", fontsize=8, ncol=3)
    ax_ang.grid(True, alpha=0.3)

    ax_vel.set_ylabel("Angular velocity (deg/s)")
    ax_vel.set_xlabel("Time (s)")
    ax_vel.set_title(f"6-IMU Velocities  [{t_start:.1f} ~ {t_start + duration:.1f} s]")
    ax_vel.legend(loc="upper right", fontsize=8, ncol=3)
    ax_vel.grid(True, alpha=0.3)

    fig.tight_layout()
    plt.show()


def interactive_plot(df, duration, order):
    """Show a slider to pick the 10-second window."""
    t_max = df["time_s"].iloc[-1]
    if t_max <= duration:
        print(f"Data is only {t_max:.1f}s long, plotting all.")
        plot_window(df, 0, t_max, order)
        return

    fig, (ax_ang, ax_vel) = plt.subplots(2, 1, figsize=(13, 8))
    plt.subplots_adjust(bottom=0.15)

    ax_slider = plt.axes([0.15, 0.03, 0.70, 0.03])
    slider = Slider(ax_slider, "Start (s)", 0, t_max - duration,
                    valinit=0, valstep=0.1)

    def update(val):
        t0 = slider.val
        mask = (df["time_s"] >= t0) & (df["time_s"] < t0 + duration)
        seg = df.loc[mask]
        t = seg["time_s"].values

        ax_ang.cla()
        ax_vel.cla()
        for i, key in enumerate(order):
            info = IMU_INFO[key]
            c = COLORS[i % len(COLORS)]
            ax_ang.plot(t, seg[info["angle"]].values, label=info["label"], color=c, linewidth=1.2)
            ax_vel.plot(t, seg[info["vel"]].values,   label=info["label"], color=c, linewidth=1.2)

        ax_ang.set_ylabel("Angle (deg)")
        ax_ang.set_title(f"6-IMU Angles  [{t0:.1f} ~ {t0 + duration:.1f} s]")
        ax_ang.legend(loc="upper right", fontsize=8, ncol=3)
        ax_ang.grid(True, alpha=0.3)

        ax_vel.set_ylabel("Angular velocity (deg/s)")
        ax_vel.set_xlabel("Time (s)")
        ax_vel.set_title(f"6-IMU Velocities  [{t0:.1f} ~ {t0 + duration:.1f} s]")
        ax_vel.legend(loc="upper right", fontsize=8, ncol=3)
        ax_vel.grid(True, alpha=0.3)

        fig.canvas.draw_idle()

    slider.on_changed(update)
    update(0)
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Plot 6-IMU angles & velocities")
    parser.add_argument("-f", "--file", default="walking_log_002.csv",
                        help="CSV file name or path")
    parser.add_argument("-s", "--start", type=float, default=None,
                        help="Start time (s). If omitted, shows interactive slider.")
    parser.add_argument("-d", "--duration", type=float, default=10.0,
                        help="Window duration in seconds (default 10)")
    parser.add_argument("-o", "--order", nargs="+", default=DEFAULT_ORDER,
                        choices=["R", "L", "1", "2", "3", "4"],
                        help="IMU plot order top-to-bottom, e.g. -o R L 1 2 3 4")
    args = parser.parse_args()

    # resolve file path
    csv_path = args.file
    if not os.path.isabs(csv_path):
        csv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), csv_path)
    if not os.path.isfile(csv_path):
        sys.exit(f"File not found: {csv_path}")

    df = load_csv(csv_path)
    print(f"Loaded {len(df)} rows, {df['time_s'].iloc[-1]:.1f}s total")
    print(f"Plot order: {args.order}")

    if args.start is not None:
        plot_window(df, args.start, args.duration, args.order)
    else:
        interactive_plot(df, args.duration, args.order)


if __name__ == "__main__":
    main()
