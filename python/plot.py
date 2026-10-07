"""Plot the trajectory, the position error, the position uncertainty and the gyro bias estimate."""
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from evaluate import TURNING, position_error

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
SECONDARY = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
BLUE = "#2a78d6"
ORANGE = "#eb6834"
AQUA = "#1baf7a"

# (file, label, color, line style) of each filter estimate, in fixed color order.
# The UKF is dashed because it runs almost on top of the EKF.
FILTERS = [
    ("estimate_kf.csv", "KF (CV)", BLUE, "-"),
    ("estimate_ekf.csv", "EKF (CTRV)", ORANGE, "-"),
    ("estimate_ukf.csv", "UKF (CTRV)", AQUA, "--"),
]


def style_axes(ax, title, xlabel, ylabel):
    ax.set_facecolor(SURFACE)
    ax.set_title(title, color=INK, loc="left")
    ax.set_xlabel(xlabel, color=SECONDARY)
    ax.set_ylabel(ylabel, color=SECONDARY)
    ax.grid(color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    ax.tick_params(colors=MUTED, which="both")
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("bottom", "left"):
        ax.spines[side].set_color(GRID)


def add_legend(ax):
    # Outside the axes, so that it never covers data.
    ax.legend(frameon=False, labelcolor=SECONDARY, loc="center left", bbox_to_anchor=(1.02, 0.5))


def new_axes(figsize):
    fig, ax = plt.subplots(figsize=figsize, facecolor=SURFACE)
    return fig, ax


def finish(fig, ax, title, xlabel, ylabel, path):
    style_axes(ax, title, xlabel, ylabel)
    add_legend(ax)
    fig.savefig(path, dpi=150, bbox_inches="tight")


def shade(ax, t, mask, label):
    """Mark the time spans where mask is true."""
    ax.fill_between(t, 0, 1, where=mask, transform=ax.get_xaxis_transform(),
                    color=GRID, alpha=0.6, linewidth=0, label=label)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=Path("data/sim"))
    parser.add_argument("--out", type=Path, default=Path("results"))
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    truth = pd.read_csv(args.data / "truth.csv")
    gps = pd.read_csv(args.data / "gps.csv")
    filters = [(pd.read_csv(args.data / file), label, color, style)
               for file, label, color, style in FILTERS if (args.data / file).exists()]

    # Times without GPS: more than 1.5 fix intervals after the latest fix.
    last_fix = gps.t.values[np.searchsorted(gps.t.values, truth.t.values, side="right") - 1]
    no_gps = truth.t.values - last_fix > 1.5 * np.median(np.diff(gps.t))

    fig, ax = new_axes((7, 9))
    ax.plot(gps.x, gps.y, "o", color=MUTED, markersize=6, label="GPS")
    ax.plot(truth.x, truth.y, color=INK, linewidth=2, label="Ground truth")
    for est, label, color, style in filters:
        ax.plot(est.x, est.y, color=color, linewidth=2, linestyle=style, label=label)
    # Time stamps along the path, since an x-y plot has no time axis.
    every = 10 * np.ceil(truth.t.iloc[-1] / 100)
    marks = truth.iloc[np.searchsorted(truth.t, np.arange(0, truth.t.iloc[-1] + 1e-9, every))]
    ax.plot(marks.x, marks.y, "o", color=INK, markersize=5)
    for row in marks.itertuples():
        ax.annotate(f"{row.t:.0f} s", (row.x, row.y), xytext=(6, 6),
                    textcoords="offset points", color=SECONDARY, fontsize=9)
    ax.set_aspect("equal")
    finish(fig, ax, "Trajectory", "x (m)", "y (m)", args.out / "trajectory.png")

    fig, ax = new_axes((9, 4))
    shade(ax, truth.t, truth.psi_dot.abs() > TURNING, "Turning")
    ax.plot(gps.t, position_error(truth, gps), "o", color=MUTED, markersize=6, label="GPS")
    for est, label, color, style in filters:
        ax.plot(est.t, position_error(truth, est), color=color, linewidth=2, linestyle=style,
                label=label)
    ax.set_ylim(bottom=0)
    finish(fig, ax, "Position error", "time (s)", "error (m)", args.out / "error.png")

    fig, ax = new_axes((9, 4))
    if no_gps.any():
        shade(ax, truth.t, no_gps, "No GPS")
    for est, label, color, style in filters:
        ax.plot(est.t, np.sqrt(est.P00 + est.P11), color=color, linewidth=2, linestyle=style,
                label=label)
    ax.set_ylim(bottom=0)
    finish(fig, ax, "Position uncertainty the filter reports (1σ)", "time (s)",
           "√(Pxx + Pyy) (m)", args.out / "uncertainty.png")

    ekf_path = args.data / "estimate_ekf.csv"
    if ekf_path.exists():
        ekf = pd.read_csv(ekf_path)
        sigma = np.sqrt(ekf.P44)
        fig, ax = new_axes((9, 4))
        ax.fill_between(ekf.t, ekf.b_yaw - 2 * sigma, ekf.b_yaw + 2 * sigma,
                        color=ORANGE, alpha=0.2, linewidth=0, label="Estimate ± 2σ")
        ax.plot(ekf.t, ekf.b_yaw, color=ORANGE, linewidth=2, label="EKF estimate")
        if truth.b_yaw.notna().all():  # known for simulated data only
            ax.axhline(truth.b_yaw.iloc[0], color=INK, linewidth=1, linestyle="--", label="True bias")
        finish(fig, ax, "Gyro bias estimate", "time (s)", "bias (rad/s)", args.out / "bias.png")


if __name__ == "__main__":
    main()
