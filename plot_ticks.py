# AI-GENERATED (Claude Opus 5.5, 2026-11-27) -- appendix entry [N]
"""
plot_ticks.py -- plot one or more pid_path_new tick logs on top of each other.

Usage (run from the simulator folder, the one with simulation.py):
    python3 plot_ticks.py <ticks file> [<ticks file> ...] [out=<name>.png]

Example, Part II on the zigzag (fixed speed 0.25 and 0.3 vs adaptive, low noise, seed 1):
    python3 plot_ticks.py logfiles/pidpath_part2_zigzag_pid_0.25_low_*_s1_ticks.csv \\
                          logfiles/pidpath_part2_zigzag_pid_0.3_low_*_s1_ticks.csv \\
                          logfiles/pidpath_part2_zigzag_pid_adaptive_low_*_s1_ticks.csv \\
                          out=part2_zigzag.png

The figure has
    left         the robot paths seen from above, with the reference path (dashed)
    right, 1-4   cross-track error, heading error, turn command (pect_l - pect_r)
                 and forward-speed command (caudal) over time
Only ticks up to the goal are used (the robot keeps coasting afterwards).
The RMSE printed for each run is time-weighted, the same way pid_path_new.py
computes it, so it should match the value in the summary file.
"""
import sys
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")          # save to a file, no window needed
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec


def reference_path(name):
    """Same three paths as make_path() in pid_path_new.py, in mm."""
    if name == 'straight':
        return np.array([[-2000, 0], [2000, 0]], dtype=float)
    if name == 'curve':
        a = np.radians(np.linspace(-135, -45, 31))
        return np.column_stack([2500 * np.cos(a), 2500 * np.sin(a)])
    if name == 'zigzag':
        return np.array([[-1400, -700], [0, -700], [0, 700], [1400, 700]], dtype=float)
    return None


def parse_name(path):
    """pidpath_<tag>_<path>_<ctrl>_<speed>_<noise>_..._s<seed>_ticks.csv"""
    parts = os.path.basename(path).replace("_ticks.csv", "").split("_")
    if len(parts) >= 6:
        speed = parts[4] if parts[4] == 'adaptive' else 'fixed ' + parts[4]
        return parts[2], "{} | {} | noise {}".format(parts[3].upper(), speed, parts[5])
    return None, os.path.basename(path)


def main(files, out):
    fig = plt.figure(figsize=(13, 7.5))
    gs = GridSpec(4, 2, width_ratios=[1.15, 1], figure=fig, hspace=0.55, wspace=0.22)
    ax_xy = fig.add_subplot(gs[:, 0])
    ax_ct = fig.add_subplot(gs[0, 1])
    ax_he = fig.add_subplot(gs[1, 1], sharex=ax_ct)
    ax_turn = fig.add_subplot(gs[2, 1], sharex=ax_ct)
    ax_speed = fig.add_subplot(gs[3, 1], sharex=ax_ct)

    drawn_paths = set()
    for f in files:
        d = np.genfromtxt(f, delimiter=",", names=True)
        hit = np.where(d["reached"] > 0.5)[0]
        if len(hit) > 0:
            d = d[: hit[0] + 1]
        path_name, lab = parse_name(f)

        # time-weighted RMSE, like pid_path_new.py (weight = length of each tick)
        dt = np.diff(d["t_s"], append=d["t_s"][-1] + np.median(np.diff(d["t_s"])))
        rmse = np.sqrt(np.sum(d["cross_track_mm"] ** 2 * dt) / np.sum(dt))
        reached = "reached at {:.1f} s".format(d["t_s"][-1]) if len(hit) else "NOT reached"
        print("{}:  RMSE {:.1f} mm, {}".format(lab, rmse, reached))

        if path_name not in drawn_paths:
            ref = reference_path(path_name)
            if ref is not None:
                ax_xy.plot(ref[:, 0], ref[:, 1], "--", color="0.35", lw=1.5,
                           label="reference path" if not drawn_paths else None, zorder=1)
            drawn_paths.add(path_name)

        line, = ax_xy.plot(d["x_mm"], d["y_mm"], lw=2, label="{} ({:.0f} mm)".format(lab, rmse), zorder=2)
        c = line.get_color()
        ax_xy.plot(d["x_mm"][0], d["y_mm"][0], "o", color=c, ms=6)
        ax_xy.plot(d["x_mm"][-1], d["y_mm"][-1], "x", color=c, ms=8, mew=2)
        ax_ct.plot(d["t_s"], d["cross_track_mm"], color=c, lw=1.3)
        ax_he.plot(d["t_s"], np.degrees(d["heading_error_rad"]), color=c, lw=1.3)
        ax_turn.plot(d["t_s"], d["pect_l"] - d["pect_r"], color=c, lw=1)
        ax_speed.plot(d["t_s"], d["caudal"], color=c, lw=1.5)

    ax_xy.set(title="Robot path seen from above (o = start, x = goal reached)", xlabel="x [mm]", ylabel="y [mm]")
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.legend(fontsize=8, loc="best")
    ax_ct.set(title="Cross-track error [mm]")
    ax_he.set(title="Heading error to lookahead point [deg]")
    ax_turn.set(title="Turn command (left fin - right fin)")
    ax_speed.set(title="Forward-speed command (caudal fin)", xlabel="time [s]")
    ax_speed.set_ylim(0, 1.05)
    for ax in (ax_xy, ax_ct, ax_he, ax_turn, ax_speed):
        ax.grid(alpha=0.3)
    for ax in (ax_ct, ax_he, ax_turn):
        ax.axhline(0, color="gray", lw=0.8)
        plt.setp(ax.get_xticklabels(), visible=False)

    fig.savefig(out, dpi=150, bbox_inches="tight")
    print("saved", out)


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("out=")]
    outs = [a[4:] for a in sys.argv[1:] if a.startswith("out=")]
    if not args:
        print(__doc__)
        sys.exit(1)
    main(args, os.path.join("logfiles", outs[0]) if outs else "logfiles/ticks_plot.png")
# END AI-GENERATED (rewritten 2026-11-27: reference path, speed panel, time-weighted RMSE, out= option)
