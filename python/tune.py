"""Sweep the process noise sigma_a of the KF: position RMSE and NEES for each value."""
import argparse
import tempfile
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from evaluate import nees, position_error, print_table, rmse
from monte_carlo import run_filter, simulate
from plot import BLUE, INK, MUTED, SURFACE, add_legend, style_axes


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, epilog="Any other argument is passed on to simulate.py.")
    parser.add_argument("--runs", type=int, default=30)
    parser.add_argument("--main", type=Path, default=Path("build/main"), help="filter executable")
    parser.add_argument("--out", type=Path, default=Path("results"))
    args, simulate_args = parser.parse_known_args()
    args.out.mkdir(parents=True, exist_ok=True)

    sigmas = np.logspace(-2, 1, 13)
    errors = {s: [] for s in sigmas}     # position errors of every sample
    at_fixes = {s: [] for s in sigmas}   # position errors at the GPS timestamps
    nees_all = {s: [] for s in sigmas}
    gps_errors = []
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        for seed in range(args.runs):
            simulate(tmp, seed, simulate_args)
            truth = pd.read_csv(tmp / "truth.csv")
            gps = pd.read_csv(tmp / "gps.csv")
            gps_errors.append(position_error(truth, gps))
            for s in sigmas:
                run_filter(args.main, "kf", tmp, ["--sigma_a", str(s)])
                est = pd.read_csv(tmp / "estimate_kf.csv")
                error = position_error(truth, est)
                errors[s].append(error)
                at_fixes[s].append(error[np.isin(est.t, gps.t)])
                values, dof = nees(truth, est)
                nees_all[s].append(values / dof)

    rmse_all = [rmse(pd.concat(errors[s])) for s in sigmas]
    rmse_fix = [rmse(pd.concat(at_fixes[s])) for s in sigmas]
    nees_mean = [np.mean(np.concatenate(nees_all[s])) for s in sigmas]
    rmse_gps = rmse(pd.concat(gps_errors))

    print(f"KF (CV), {args.runs} runs pooled; raw GPS RMSE {rmse_gps:.2f} m")
    print()
    print_table(["sigma_a", "RMSE all (m)", "RMSE at fixes (m)", "NEES / states"],
                [[f"{s:.3g}", f"{a:.2f}", f"{f:.2f}", f"{n:.3g}"]
                 for s, a, f, n in zip(sigmas, rmse_all, rmse_fix, nees_mean)])

    fig, (top, bottom) = plt.subplots(2, 1, figsize=(8, 7), sharex=True, facecolor=SURFACE)
    top.plot(sigmas, rmse_all, "o-", color=BLUE, linewidth=2, label="KF, every sample")
    top.plot(sigmas, rmse_fix, "o--", color=BLUE, linewidth=2, label="KF, at GPS fixes")
    top.axhline(rmse_gps, color=MUTED, linewidth=1.5, label="Raw GPS")
    top.set_xscale("log")
    top.set_yscale("log")
    style_axes(top, "Effect of the process noise on the KF", "", "position RMSE (m)")
    add_legend(top)

    bottom.plot(sigmas, nees_mean, "o-", color=BLUE, linewidth=2, label="KF")
    bottom.axhline(1, color=INK, linewidth=1, linestyle="--", label="Consistent")
    bottom.set_yscale("log")
    style_axes(bottom, "", "sigma_a", "NEES / states")
    add_legend(bottom)
    fig.savefig(args.out / "tuning.png", dpi=150, bbox_inches="tight")


if __name__ == "__main__":
    main()
