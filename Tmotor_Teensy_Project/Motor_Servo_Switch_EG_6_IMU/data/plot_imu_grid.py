#!/usr/bin/env python3
"""
Plot 6 IMU signals: 2 subplots per figure (top=first 3, bottom=last 3).
One figure for angles, one for velocities. Both saved as PNG.

Usage:
    python plot_imu_grid.py -s 5 -e 15
    python plot_imu_grid.py -s 5 -e 15 -o R L 1 2 3 4
    python plot_imu_grid.py -f walking_log_002.csv -s 0 -e 10 -o 1 2 3 4 R L

Options:
    -f / --file    CSV file (default: walking_log_002.csv)
    -s / --start   Start time in seconds
    -e / --end     End time in seconds
    -o / --order   6 IMU keys: first 3 -> top subplot, last 3 -> bottom subplot
    
        python plot_imu_grid.py -s 35 -e 45 -o R 2 1 L 3 4
        
        
"""

import argparse, os, sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from itertools import combinations
from scipy.signal import butter, filtfilt

IMU_INFO = {
    "R": {"angle": "imu_RTx",            "vel": "imu_Rvel",  "label": "Right thigh"},
    "L": {"angle": "imu_LTx",            "vel": "imu_Lvel",  "label": "Left thigh"},
    "1": {"angle": "imu_1_left_shaking",  "vel": "imu_vel_1", "label": "Right bar 1"},
    "2": {"angle": "imu_2_right_shaking", "vel": "imu_vel_2", "label": "Right bar 2"},
    "3": {"angle": "imu_3_left_foot",     "vel": "imu_vel_3", "label": "Left bar 1"},
    "4": {"angle": "imu_4_right_foot",    "vel": "imu_vel_4", "label": "Left bar 2"},
}
DEFAULT_ORDER = ["R", "L", "1", "2", "3", "4"]
COLORS = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3", "#ff7f00", "#a65628"]


def load_csv(path):
    df = pd.read_csv(path)
    df["time_s"] = (df["Time_ms"] - df["Time_ms"].iloc[0]) / 1000.0
    return df


def apply_butter_lp(df, cutoff=6.0, order=4):
    """Apply zero-phase Butterworth low-pass to all IMU angle & velocity columns."""
    dt = np.median(np.diff(df["time_s"].values))
    fs = 1.0 / dt
    b, a = butter(order, cutoff / (fs / 2), btype='low')
    cols = set()
    for info in IMU_INFO.values():
        cols.add(info["angle"])
        cols.add(info["vel"])
    for c in cols:
        if c in df.columns:
            df[c] = filtfilt(b, a, df[c].values)
    return df


def make_figure(seg, order, mode, ylabel, suptitle, save_path):
    t = seg["time_s"].values
    top3 = order[:3]
    bot3 = order[3:]

    fig, (ax_top, ax_bot) = plt.subplots(2, 1, figsize=(13, 7), sharex=True)

    for i, key in enumerate(top3):
        info = IMU_INFO[key]
        col = info["angle"] if mode == "angle" else info["vel"]
        ax_top.plot(t, seg[col].values, label=info["label"], color=COLORS[i], linewidth=1.2)

    for i, key in enumerate(bot3):
        info = IMU_INFO[key]
        col = info["angle"] if mode == "angle" else info["vel"]
        ax_bot.plot(t, seg[col].values, label=info["label"], color=COLORS[3 + i], linewidth=1.2)

    for ax in (ax_top, ax_bot):
        ax.set_ylabel(ylabel)
        ax.legend(loc="upper right", fontsize=9, ncol=3)
        ax.grid(True, alpha=0.3)

    ax_bot.set_xlabel("Time (s)")
    t0 = seg["time_s"].iloc[0]
    t1 = seg["time_s"].iloc[-1]
    fig.suptitle(f"{suptitle}  [{t0:.1f} ~ {t1:.1f} s]", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.95])

    fig.savefig(save_path, dpi=200, bbox_inches="tight")
    print(f"Saved: {save_path}")


def r_squared(a, b):
    """R² between two arrays (treating b as 'predicted' for a)."""
    ss_res = np.sum((a - b) ** 2)
    ss_tot = np.sum((a - np.mean(a)) ** 2)
    if ss_tot == 0:
        return float('nan')
    return 1.0 - ss_res / ss_tot


def report_r2(seg, order):
    top3 = order[:3]
    bot3 = order[3:]

    def group_report(keys, group_name):
        print(f"\n{'='*60}")
        print(f"  {group_name}:  {', '.join(IMU_INFO[k]['label'] for k in keys)}")
        print(f"{'='*60}")

        # --- range ---
        print(f"\n  {'Signal':<18s} {'Angle range (deg)':>22s}   {'Velocity range (deg/s)':>26s}")
        print(f"  {'-'*18} {'-'*22}   {'-'*26}")
        for k in keys:
            info = IMU_INFO[k]
            ang = seg[info["angle"]].values
            vel = seg[info["vel"]].values
            print(f"  {info['label']:<18s} [{ang.min():+8.2f}, {ang.max():+8.2f}]"
                  f"   [{vel.min():+9.2f}, {vel.max():+9.2f}]")

        # --- R² pairwise ---
        print(f"\n  Pairwise R²:")
        print(f"  {'Pair':<36s} {'Angle R²':>10s}  {'Velocity R²':>12s}")
        print(f"  {'-'*36} {'-'*10}  {'-'*12}")
        for (k1, k2) in combinations(keys, 2):
            i1, i2 = IMU_INFO[k1], IMU_INFO[k2]
            a1 = seg[i1["angle"]].values;  a2 = seg[i2["angle"]].values
            v1 = seg[i1["vel"]].values;    v2 = seg[i2["vel"]].values
            r2_ang = r_squared(a1, a2)
            r2_vel = r_squared(v1, v2)
            pair = f"{i1['label']} vs {i2['label']}"
            print(f"  {pair:<36s} {r2_ang:>10.4f}  {r2_vel:>12.4f}")

    group_report(top3, "Top row (group 1)")
    group_report(bot3, "Bottom row (group 2)")
    print()


def main():
    parser = argparse.ArgumentParser(description="Plot 6-IMU (2 rows x 1 col, 3 lines each)")
    parser.add_argument("-f", "--file", default="walking_log_002.csv")
    parser.add_argument("-s", "--start", type=float, required=True, help="Start time (s)")
    parser.add_argument("-e", "--end",   type=float, required=True, help="End time (s)")
    parser.add_argument("-o", "--order", nargs=6, default=DEFAULT_ORDER,
                        choices=["R", "L", "1", "2", "3", "4"],
                        help="6 IMU keys: first 3 -> top row, last 3 -> bottom row")
    parser.add_argument("-c", "--cutoff", type=float, default=6.0,
                        help="Butterworth LP cutoff freq in Hz (default: 6)")
    args = parser.parse_args()

    csv_path = args.file
    if not os.path.isabs(csv_path):
        csv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), csv_path)
    if not os.path.isfile(csv_path):
        sys.exit(f"File not found: {csv_path}")

    df = load_csv(csv_path)
    df = apply_butter_lp(df, cutoff=args.cutoff)
    print(f"Applied 4th-order Butterworth LP @ {args.cutoff} Hz")
    mask = (df["time_s"] >= args.start) & (df["time_s"] <= args.end)
    seg = df.loc[mask]
    if seg.empty:
        sys.exit(f"No data in [{args.start:.1f}, {args.end:.1f}] s")

    print(f"Loaded {len(df)} rows, {df['time_s'].iloc[-1]:.1f}s total")
    print(f"Top row: {args.order[:3]},  Bottom row: {args.order[3:]}")

    out_dir = os.path.dirname(os.path.abspath(__file__))
    make_figure(seg, args.order, "angle", "Angle (deg)", "6-IMU Angles",
                os.path.join(out_dir, "imu_angles.png"))
    make_figure(seg, args.order, "vel", "Ang. velocity (deg/s)", "6-IMU Velocities",
                os.path.join(out_dir, "imu_velocities.png"))

    # ---- R² report ----
    report_r2(seg, args.order)
    sys.stdout.flush()

    plt.show()


if __name__ == "__main__":
    main()
