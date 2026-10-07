"""NEES and NIS of every filter over many simulated runs, against 95% chi-square bounds.

A consistent filter reports a covariance that matches its real errors. Then the
NEES e' P^-1 e averages to the number of states, and the NIS of each GPS fix to
the size of the measurement (2). Values above the band mean the filter is
overconfident, values below it that it is too cautious.
"""
import argparse
import tempfile
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.stats import chi2

from evaluate import nees, print_table
from monte_carlo import run_filter, simulate
from plot import FILTERS, GRID, INK, SECONDARY, SURFACE, add_legend, style_axes

GPS_DOF = 2
BIAS_SIGMA0 = 0.05  # rad/s, the prior the filters put on the gyro bias (BIAS_VAR0 in main.cpp)


def band(dof, runs):
    """95% interval of the mean of `runs` chi-square variables, divided by dof (so it sits around 1)."""
    low, high = chi2.ppf([0.025, 0.975], dof * runs) / (dof * runs)
    return low, high


def plot_panels(series, runs, marker, title, ylabel, path):
    """One panel per filter: the run-averaged statistic over time inside its 95% band."""
    fig, axes = plt.subplots(len(series), 1, figsize=(9, 2.6 * len(series)), sharex=True,
                             sharey=True, facecolor=SURFACE)
    for ax, (label, color, mean, dof) in zip(axes, series):
        low, high = band(dof, runs)
        ax.axhspan(low, high, color=GRID, alpha=0.8, linewidth=0, label="95% band")
        ax.axhline(1, color=INK, linewidth=1, linestyle="--", label="Consistent")
        ax.plot(mean.index, mean / dof, marker, color=color, linewidth=1.5, markersize=4, label=label)
        style_axes(ax, "", "", ylabel)
        add_legend(ax)
    axes[0].set_title(title, color=INK, loc="left")
    axes[-1].set_xlabel("time (s)", color=SECONDARY)
    fig.savefig(path, dpi=150, bbox_inches="tight")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        epilog="Any other argument is passed on to simulate.py.")
    parser.add_argument("--runs", type=int, default=50)
    parser.add_argument("--main", type=Path, default=Path("build/main"), help="filter executable")
    parser.add_argument("--out", type=Path, default=Path("results"))
    args, simulate_args = parser.parse_known_args()
    args.out.mkdir(parents=True, exist_ok=True)

    names = [file.removeprefix("estimate_").removesuffix(".csv") for file, *_ in FILTERS]
    nees_runs = {name: [] for name in names}  # one series over time per run
    nis_runs = {name: [] for name in names}
    dof = {}
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        for seed in range(args.runs):
            # The test is only meaningful if the truth comes from the distribution the
            # filter assumes, so every run draws its gyro bias from the filter's prior.
            bias = np.random.default_rng(seed).normal(0, BIAS_SIGMA0)
            simulate(tmp, seed, ["--gyro_bias", str(bias), *simulate_args])
            truth = pd.read_csv(tmp / "truth.csv")
            for name in names:
                run_filter(args.main, name, tmp)
                est = pd.read_csv(tmp / f"estimate_{name}.csv")
                values, dof[name] = nees(truth, est)
                nees_runs[name].append(pd.Series(values, index=est.t))
                nis_runs[name].append(est.set_index("t").nis.dropna())

    nees_series, nis_series, rows = [], [], []
    for name, (_, label, color, _) in zip(names, FILTERS):
        # Average over the runs, at the times every run covers.
        nees_mean = pd.concat(nees_runs[name], axis=1).dropna().mean(axis=1)
        nis_mean = pd.concat(nis_runs[name], axis=1).dropna().mean(axis=1)
        nees_series.append((label, color, nees_mean, dof[name]))
        nis_series.append((label, color, nis_mean, GPS_DOF))

        def inside(mean, d):
            low, high = band(d, args.runs)
            return f"{((mean / d >= low) & (mean / d <= high)).mean():.0%}"

        rows.append([label, str(dof[name]),
                     f"{nees_mean.mean() / dof[name]:.2f}", inside(nees_mean, dof[name]),
                     f"{nis_mean.mean() / GPS_DOF:.2f}", inside(nis_mean, GPS_DOF)])

    print(f"{args.runs} runs" + (f", simulate.py {' '.join(simulate_args)}" if simulate_args else ""))
    print()
    print_table(["", "states", "NEES / states", "time inside band", "NIS / 2", "fixes inside band"],
                rows)

    plot_panels(nees_series, args.runs, "-", f"NEES per state, mean of {args.runs} runs",
                "NEES / states", args.out / "nees.png")
    plot_panels(nis_series, args.runs, "o", f"NIS per measurement dimension, mean of {args.runs} runs",
                "NIS / 2", args.out / "nis.png")


if __name__ == "__main__":
    main()
