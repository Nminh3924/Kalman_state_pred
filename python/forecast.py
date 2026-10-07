"""Evaluate forecasts of the vehicle position up to 3 s ahead.

ADE is the displacement error averaged over the horizons of a forecast, FDE the
error at the last horizon. Coverage is the share of true positions that fall
inside the 95% ellipse of the forecast covariance: 95% if that covariance is right.
"""
import argparse
import shlex
import tempfile
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.patches import Ellipse

from evaluate import TURNING, print_table
from monte_carlo import SIMULATE, run_filter, simulate
from plot import INK, ORANGE, SURFACE, add_legend, style_axes

CHI2_95 = 5.991  # 95% quantile of chi-square with 2 degrees of freedom
EVERY = 10       # forecast from every 10th estimate

# (label, filter, held). held: 0 = straight ahead at constant speed, 1 = the
# current yaw rate is held, 2 = the current acceleration is held as well.
VARIANTS = [
    ("KF (CV)", "kf", 0),
    ("EKF state, straight ahead", "ekf", 0),
    ("EKF, yaw rate held (CTRV)", "ekf", 1),
    ("UKF, yaw rate held (CTRV)", "ukf", 1),
    ("EKF, yaw rate and accel held", "ekf", 2),
]


def score(data, name, held):
    """One row per forecast point: its error against the truth and whether the ellipse covers it."""
    truth = pd.read_csv(data / "truth.csv")
    f = pd.read_csv(data / f"forecast_{name}.csv")
    f = f[(f.held == held) & (f.t0 + f.horizon.max() <= truth.t.iloc[-1])].copy()
    ex = f.x - np.interp(f.t0 + f.horizon, truth.t, truth.x)
    ey = f.y - np.interp(f.t0 + f.horizon, truth.t, truth.y)
    det = f.Pxx * f.Pyy - f.Pxy ** 2
    mahalanobis = (f.Pyy * ex ** 2 - 2 * f.Pxy * ex * ey + f.Pxx * ey ** 2) / det
    f["error"] = np.hypot(ex, ey)
    f["covered"] = mahalanobis <= CHI2_95
    f["turning"] = np.abs(np.interp(f.t0, truth.t, truth.psi_dot)) > TURNING
    return f


def summarize(f):
    last = f[f.horizon == f.horizon.max()]

    def at(h):
        return f[np.isclose(f.horizon, h)]

    return [f"{f.error.mean():.2f}", f"{last.error.mean():.2f}",
            f"{last.error[~last.turning].mean():.2f}", f"{last.error[last.turning].mean():.2f}",
            f"{at(1).error.mean():.2f}", f"{at(1).covered.mean():.0%}", f"{at(3).covered.mean():.0%}",
            f"{f.covered.mean():.0%}"]


def ellipse(row, color):
    """The 95% ellipse of one forecast point."""
    values, vectors = np.linalg.eigh([[row.Pxx, row.Pxy], [row.Pxy, row.Pyy]])
    width, height = 2 * np.sqrt(CHI2_95 * values)
    angle = np.degrees(np.arctan2(vectors[1, 0], vectors[0, 0]))
    return Ellipse((row.x, row.y), width, height, angle=angle, facecolor=color, alpha=0.18,
                   edgecolor=color, linewidth=1)


def plot(data, window, path):
    """Forecasts every 2 s inside a time window: straight ahead next to yaw rate held."""
    truth = pd.read_csv(data / "truth.csv")
    forecasts = pd.read_csv(data / "forecast_ekf.csv")
    estimate = pd.read_csv(data / "estimate_ekf.csv").set_index("t")
    shown = truth[(truth.t >= window[0]) & (truth.t <= window[1] + 3)]
    # The forecast made closest to every second second of the window.
    made = forecasts.t0.unique()
    origins = sorted({made[np.abs(made - t).argmin()] for t in np.arange(window[0], window[1] + 1e-9, 2)})

    # Two equal-aspect panels: size the figure to the shape of the path.
    panel = np.clip(6 * (np.ptp(shown.x) + 30) / (np.ptp(shown.y) + 30), 2.5, 7)
    fig, axes = plt.subplots(1, 2, figsize=(2 * panel + 2.5, 6.5), sharex=True, sharey=True,
                             facecolor=SURFACE)
    for ax, held, title in zip(axes, (0, 1), ("Straight ahead", "Yaw rate held (CTRV)")):
        ax.plot(shown.x, shown.y, color=INK, linewidth=2, label="Ground truth")
        for i, t0 in enumerate(origins):
            f = forecasts[(forecasts.t0 == t0) & (forecasts.held == held)]
            start = estimate.loc[t0]
            ax.plot([start.x, *f.x], [start.y, *f.y], color=ORANGE, linewidth=1.5,
                    label="Forecast, 3 s" if i == 0 else None)
            for row in f[np.isclose(f.horizon % 1, 0)].itertuples():
                ax.add_patch(ellipse(row, ORANGE))
            ax.plot(np.interp(t0 + 3, truth.t, truth.x), np.interp(t0 + 3, truth.t, truth.y), "o",
                    color=INK, markersize=5, label="Truth 3 s later" if i == 0 else None)
        ax.plot([], [], "s", color=ORANGE, alpha=0.4, markersize=10, label="95% ellipse at 1, 2, 3 s")
        ax.set_aspect("equal")
        style_axes(ax, title, "x (m)", "y (m)" if held == 0 else "")
    add_legend(axes[1])
    fig.savefig(path, dpi=150, bbox_inches="tight")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        epilog="Any other argument is passed on to the generator.")
    parser.add_argument("--runs", type=int, default=50)
    parser.add_argument("--generator", type=Path, default=SIMULATE,
                        help="script that writes one run; kitti_convert.py for recorded data")
    parser.add_argument("--main_args", default="",
                        help='arguments for every filter, as one string: --main_args="--forecast_yaw 0.1"')
    parser.add_argument("--main", type=Path, default=Path("build/main"), help="filter executable")
    parser.add_argument("--plot", type=Path, help="write a figure of the first run to this file")
    parser.add_argument("--window", type=float, nargs=2, default=(0, 30), metavar=("START", "END"),
                        help="time span of the figure in seconds")
    args, generator_args = parser.parse_known_args()

    extra = ["--forecast", str(EVERY), *shlex.split(args.main_args)]
    scores = {label: [] for label, _, _ in VARIANTS}
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        for seed in range(args.runs):
            simulate(tmp, seed, generator_args, args.generator)
            for name in ("kf", "ekf", "ukf"):
                run_filter(args.main, name, tmp, extra)
            for label, name, held in VARIANTS:
                scores[label].append(score(tmp, name, held))
            if seed == 0 and args.plot:
                args.plot.parent.mkdir(parents=True, exist_ok=True)
                plot(tmp, args.window, args.plot)

    print(f"Forecasts up to 3 s ahead, {args.runs} runs pooled"
          + (f", {args.generator.name} {' '.join(generator_args)}" if generator_args else "")
          + (f", filters {args.main_args}" if args.main_args else ""))
    print()
    print_table(["", "ADE (m)", "FDE (m)", "FDE straight", "FDE turning", "error at 1 s",
                 "coverage at 1 s", "coverage at 3 s", "coverage, all"],
                [[label] + summarize(pd.concat(scores[label])) for label, _, _ in VARIANTS])


if __name__ == "__main__":
    main()
