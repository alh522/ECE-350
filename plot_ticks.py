# AI-GENERATED (Claude Opus 5.5, 2026-11-27) -- appendix entry [N]
"""
plot_ticks.py -- plot one or more pid_path tick logs.

Usage (run from the simulator folder):
    python3 plot_ticks.py logfiles/pidpath_test_zigzag_pid_0.25_none_s1_ticks.csv
    python3 plot_ticks.py logfiles/*zigzag_pid*ticks.csv logfiles/*zigzag_p_*ticks.csv

Makes a 2x2 figure:
    top-left     top-down path of the robot (x vs y)
    top-right    cross-track error vs time
    bottom-left  heading error vs time
    bottom-right turn command (pect_l - pect_r) vs time
and prints the RMSE of cross_track_mm for each file, so you can check it
against the RMSE pid_path.py printed at the end of the run.
"""
import sys
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")          # save to file, no window needed
import matplotlib.pyplot as plt


def label_from_name(path):
    # pidpath_<tag>_<path>_<ctrl>_<speed>_<noise>_s<seed>_ticks.csv
    parts = os.path.basename(path).replace("_ticks.csv", "").split("_")
    if len(parts) >= 7:
        return f"{parts[2]} | ctrl={parts[3]} | v={parts[4]} | noise={parts[5]}"
    return os.path.basename(path)


def main(files):
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    ax_xy, ax_ct, ax_he, ax_turn = axes.ravel()

    for f in files:
        d = np.genfromtxt(f, delimiter=",", names=True)
        # Keep only ticks up to the first one where the goal was reached.
        # The simulator keeps running after the stop condition, and the
        # robot drifts; those ticks should not count toward RMSE.
        hit = np.where(d["reached"] > 0.5)[0]
        if len(hit) > 0:
            d = d[: hit[0] + 1]
        lab = label_from_name(f)
        rmse = np.sqrt(np.mean(d["cross_track_mm"] ** 2))
        print(f"{lab}:  cross-track RMSE = {rmse:.1f} mm, "
              f"final t = {d['t_s'][-1]:.1f} s, reached = {int(d['reached'][-1])}")

        line, = ax_xy.plot(d["x_mm"], d["y_mm"], label=f"{lab} ({rmse:.0f} mm)")
        c = line.get_color()
        ax_xy.plot(d["x_mm"][0], d["y_mm"][0], "o", color=c)    # start
        ax_xy.plot(d["x_mm"][-1], d["y_mm"][-1], "x", color=c)  # end
        ax_ct.plot(d["t_s"], d["cross_track_mm"], color=c)
        ax_he.plot(d["t_s"], np.degrees(d["heading_error_rad"]), color=c)
        ax_turn.plot(d["t_s"], d["pect_l"] - d["pect_r"], color=c)

    ax_xy.set(title="Robot path (o = start, x = end)", xlabel="x [mm]", ylabel="y [mm]")
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.legend(fontsize=8)
    ax_ct.set(title="Cross-track error", xlabel="time [s]", ylabel="error [mm]")
    ax_he.set(title="Heading error to lookahead point", xlabel="time [s]", ylabel="error [deg]")
    ax_turn.set(title="Turn command", xlabel="time [s]", ylabel="pect_l - pect_r")
    for ax in axes.ravel():
        ax.grid(alpha=0.3)
        if ax is not ax_xy:
            ax.axhline(0, color="gray", lw=0.8)

    fig.tight_layout()
    out = "logfiles/ticks_plot.png"
    fig.savefig(out, dpi=150)
    print(f"saved {out}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    main(sys.argv[1:])
# END AI-GENERATED
