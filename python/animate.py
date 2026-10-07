"""Animate one run: the vehicle, its GPS fixes, the EKF estimate and the 3 s forecast.

Needs a data directory in which the EKF was run with forecasts:
    build/main ekf <data> --forecast 10
"""
import argparse
from collections import namedtuple
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # render off-screen, at the same pixel size on every machine

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np
import pandas as pd
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

from forecast import ellipse
from plot import GRID, INK, MUTED, ORANGE, SECONDARY, SURFACE

HALF_VIEW = 55.0  # m, the camera follows the vehicle
STEP = 0.4        # s of the run per frame
FPS = 10
Point = namedtuple("Point", "x y Pxx Pxy Pyy")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--data", type=Path, default=Path("data/sim_demo"))
    parser.add_argument("--out", type=Path, default=Path("results/demo.gif"))
    args = parser.parse_args()

    truth = pd.read_csv(args.data / "truth.csv")
    gps = pd.read_csv(args.data / "gps.csv")
    est = pd.read_csv(args.data / "estimate_ekf.csv")
    ahead = pd.read_csv(args.data / "forecast_ekf.csv")
    ahead = ahead[ahead.held == 1]  # the current yaw rate is held
    origins = ahead.t0.unique()
    fix_interval = np.median(np.diff(gps.t))

    fig, ax = plt.subplots(figsize=(5.6, 6.0), dpi=90, facecolor=SURFACE)
    fig.subplots_adjust(left=0.03, right=0.97, top=0.93, bottom=0.17)
    legend = [
        Line2D([], [], color=INK, linewidth=2, alpha=0.35, label="True path"),
        Line2D([], [], color=INK, marker="o", linestyle="", markersize=7, label="Vehicle"),
        Line2D([], [], color=MUTED, marker="o", linestyle="", markersize=5, label="GPS fix (1 Hz, 3 m noise)"),
        Line2D([], [], color=ORANGE, linewidth=2, label="EKF estimate"),
        Line2D([], [], color=ORANGE, linewidth=1.2, linestyle="--", label="Forecast, 3 s"),
        Patch(facecolor=ORANGE, alpha=0.3, edgecolor=ORANGE, label="95% ellipse"),
    ]

    def draw(t):
        ax.clear()
        ax.set_facecolor(SURFACE)
        now = est.iloc[np.searchsorted(est.t, t)]
        cx, cy = np.interp(now.t, truth.t, truth.x), np.interp(now.t, truth.t, truth.y)

        ax.plot(truth.x, truth.y, color=INK, linewidth=2, alpha=0.35)
        seen = gps[gps.t <= now.t]
        ax.plot(seen.x, seen.y, "o", color=MUTED, markersize=5)
        trail = est[est.t <= now.t]
        ax.plot(trail.x, trail.y, color=ORANGE, linewidth=2)

        # Where the filter thinks the vehicle is, and where it expects it to go.
        ax.add_patch(ellipse(Point(now.x, now.y, now.P00, now.P01, now.P11), ORANGE))
        f = ahead[ahead.t0 == origins[np.abs(origins - now.t).argmin()]]
        ax.plot([now.x, *f.x], [now.y, *f.y], color=ORANGE, linewidth=1.2, linestyle="--")
        for row in f[np.isclose(f.horizon % 1, 0)].itertuples():
            ax.add_patch(ellipse(row, ORANGE))
        ax.plot(cx, cy, "o", color=INK, markersize=7)

        # Grid lines every 20 m, fixed in the world while the view moves with the vehicle.
        ax.set_xticks(np.arange(np.ceil((cx - HALF_VIEW) / 20) * 20, cx + HALF_VIEW, 20))
        ax.set_yticks(np.arange(np.ceil((cy - HALF_VIEW) / 20) * 20, cy + HALF_VIEW, 20))
        ax.set_xlim(cx - HALF_VIEW, cx + HALF_VIEW)
        ax.set_ylim(cy - HALF_VIEW, cy + HALF_VIEW)
        ax.set_aspect("equal")
        ax.tick_params(length=0, labelbottom=False, labelleft=False)
        ax.grid(color=GRID, linewidth=0.8)
        ax.set_axisbelow(True)
        for spine in ax.spines.values():
            spine.set_color(GRID)

        no_gps = now.t - seen.t.iloc[-1] > 1.5 * fix_interval
        ax.set_title(f"t = {now.t:4.1f} s" + ("    no GPS, IMU only" if no_gps else ""),
                     color=INK, loc="left", fontsize=11, fontweight="bold" if no_gps else "normal")
        ax.text(0.98, 0.02, "grid 20 m", transform=ax.transAxes, ha="right", color=SECONDARY, fontsize=8)

    fig.legend(handles=legend, loc="lower center", ncol=2, frameon=False, labelcolor=SECONDARY,
               fontsize=8)
    times = np.arange(est.t.iloc[0], est.t.iloc[-1], STEP)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    FuncAnimation(fig, draw, frames=times).save(args.out, writer=PillowWriter(fps=FPS))


if __name__ == "__main__":
    main()
