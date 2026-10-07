"""Position RMSE of the raw GPS and of each filter estimate against ground truth."""
import argparse
from pathlib import Path

import numpy as np
import pandas as pd

# (label, file) of everything that can be compared with the truth.
SOURCES = [
    ("GPS raw", "gps.csv"),
    ("KF (CV)", "estimate_kf.csv"),
    ("KF + gating", "estimate_kf_gated.csv"),
    ("EKF, no bias", "estimate_ekf_nobias.csv"),
    ("EKF (CTRV)", "estimate_ekf.csv"),
    ("EKF + gating", "estimate_ekf_gated.csv"),
    ("UKF (CTRV)", "estimate_ukf.csv"),
    ("UKF + gating", "estimate_ukf_gated.csv"),
]
COLUMNS = ["at fixes", "all", "straight", "turning", "no GPS"]
TURNING = 0.05  # rad/s, yaw rates above this count as turning


def position_error(truth, est):
    """Distance between each estimate and the truth interpolated at its timestamp."""
    ex = est.x - np.interp(est.t, truth.t, truth.x)
    ey = est.y - np.interp(est.t, truth.t, truth.y)
    return np.hypot(ex, ey)


def rmse(errors):
    return np.sqrt(np.mean(errors ** 2))


def covariances(est):
    """Covariance matrix of every row, rebuilt from the upper-triangle columns P00, P01, ..."""
    columns = [c for c in est.columns if c[0] == "P"]
    n = round((np.sqrt(8 * len(columns) + 1) - 1) / 2)
    P = np.empty((len(est), n, n))
    for c in columns:
        i, j = int(c[1]), int(c[2])
        P[:, i, j] = P[:, j, i] = est[c]
    return P


def state_error(truth, est):
    """Estimate minus truth for every state of the filter, one row per estimate."""
    def at(column):
        return np.interp(est.t, truth.t, truth[column])

    v, psi = at("v"), at("psi")
    true = {"x": at("x"), "y": at("y"), "v": v, "psi": psi,
            "vx": v * np.cos(psi), "vy": v * np.sin(psi), "b_yaw": at("b_yaw")}
    names = list(est.columns[1:est.columns.get_loc("P00")])
    error = np.column_stack([est[name] - true[name] for name in names])
    if "psi" in names:  # a heading error is an angle: wrap it into [-pi, pi)
        k = names.index("psi")
        error[:, k] = np.remainder(error[:, k] + np.pi, 2 * np.pi) - np.pi
    return error


def nees(truth, est):
    """Normalized estimation error squared e' P^-1 e of every row, and its degrees of freedom."""
    error = state_error(truth, est)
    weighted = np.linalg.solve(covariances(est), error[:, :, None])[:, :, 0]
    return np.sum(error * weighted, axis=1), error.shape[1]


def load_errors(data):
    """Position errors of every source found in data: {label: table of t, error and masks}.

    Filters start at different times, so all of them are cut to the interval
    that every one of them covers.
    """
    truth = pd.read_csv(data / "truth.csv")
    gps = pd.read_csv(data / "gps.csv")
    sources = {label: pd.read_csv(data / file) for label, file in SOURCES if (data / file).exists()}
    start = max(est.t.iloc[0] for est in sources.values())
    fix_interval = np.median(np.diff(gps.t))

    errors = {}
    for label, est in sources.items():
        est = est[est.t >= start]
        last_fix = gps.t.values[np.searchsorted(gps.t.values, est.t.values, side="right") - 1]
        errors[label] = pd.DataFrame({
            "t": est.t.values,
            "error": position_error(truth, est).values,
            "at_fix": np.isin(est.t, gps.t),
            "turning": np.abs(np.interp(est.t, truth.t, truth.psi_dot)) > TURNING,
            "no_gps": est.t.values - last_fix > 1.5 * fix_interval,
        })
    return errors


def summarize(table):
    """RMSE at the GPS timestamps, where raw GPS exists, and over every sample."""
    def over(mask):
        return rmse(table.error[mask]) if mask.any() else np.nan

    return {
        "at fixes": over(table.at_fix),
        "all": rmse(table.error),
        "straight": over(~table.turning),
        "turning": over(table.turning),
        "no GPS": over(table.no_gps),
    }


def table_row(label, summary):
    """Cells of one table row. Raw GPS exists at the fixes only."""
    shown = ["at fixes"] if label == "GPS raw" else COLUMNS
    return [label] + [f"{summary[c]:.2f}" if c in shown and not np.isnan(summary[c]) else "–"
                      for c in COLUMNS]


def print_table(header, rows):
    print("| " + " | ".join(header) + " |")
    print("|" + "---|" * len(header))
    for row in rows:
        print("| " + " | ".join(row) + " |")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=Path("data/sim"))
    args = parser.parse_args()

    print_table(["position RMSE (m)"] + COLUMNS,
                [table_row(label, summarize(table))
                 for label, table in load_errors(args.data).items()])


if __name__ == "__main__":
    main()
