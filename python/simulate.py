"""Generate a ground-truth vehicle trajectory and simulated GPS/IMU data as CSV."""
import argparse
import math
from pathlib import Path

import numpy as np

SIM_HZ = 1000  # truth integration rate, finer than the filter's 100 Hz step
OUT_HZ = 100   # rate of truth.csv and imu.csv
GPS_HZ = 1
T_END = 60     # s
V0 = 10.0      # m/s

GPS_SIGMA = 3.0     # m
ACCEL_SIGMA = 0.1   # m/s^2
GYRO_SIGMA = 0.01   # rad/s
GYRO_BIAS = 0.01    # rad/s
OUTLIER_OFFSET = 30.0  # m

TURN_RATE = 0.2     # rad/s


def schedule(turn_rate):
    """Driving script: (start time [s], longitudinal accel [m/s^2], yaw rate [rad/s])."""
    return [
        (0, 0.0, 0.0),          # straight
        (10, 0.0, turn_rate),   # left turn
        (20, 1.0, 0.0),         # accelerate
        (30, 0.0, 0.0),         # straight
        (40, 0.0, -turn_rate),  # right turn
        (50, -1.0, 0.0),        # brake
    ]


def simulate_truth(turn_rate=TURN_RATE, gyro_bias=GYRO_BIAS):
    """Integrate the vehicle kinematics. Returns the truth columns and the true accel."""
    n = T_END * SIM_HZ + 1
    dt = 1 / SIM_HZ

    a = np.zeros(n)
    w = np.zeros(n)
    for t_start, a_seg, w_seg in schedule(turn_rate):
        a[t_start * SIM_HZ:] = a_seg
        w[t_start * SIM_HZ:] = w_seg

    x, y, v, psi = np.zeros(n), np.zeros(n), np.zeros(n), np.zeros(n)
    v[0] = V0
    for k in range(n - 1):
        x[k + 1] = x[k] + v[k] * math.cos(psi[k]) * dt
        y[k + 1] = y[k] + v[k] * math.sin(psi[k]) * dt
        v[k + 1] = v[k] + a[k] * dt
        psi[k + 1] = psi[k] + w[k] * dt

    k = np.arange(0, n, SIM_HZ // OUT_HZ)
    truth = {"t": k / SIM_HZ, "x": x[k], "y": y[k], "v": v[k], "psi": psi[k], "psi_dot": w[k],
             "b_yaw": np.full(len(k), gyro_bias)}
    return truth, a[k]


def simulate_gps(truth, rng, outage, outlier_rate):
    """Sample the true position at GPS_HZ and add white Gaussian noise.

    Every random number is drawn whether or not outliers or an outage are enabled,
    so a given seed produces the same base noise in every scenario.
    """
    k = np.arange(0, len(truth["t"]), OUT_HZ // GPS_HZ)
    n = len(k)
    t = truth["t"][k]
    x = truth["x"][k] + rng.normal(0, GPS_SIGMA, n)
    y = truth["y"][k] + rng.normal(0, GPS_SIGMA, n)

    is_outlier = rng.random(n) < outlier_rate
    angle = rng.uniform(0, 2 * math.pi, n)
    x += is_outlier * OUTLIER_OFFSET * np.cos(angle)
    y += is_outlier * OUTLIER_OFFSET * np.sin(angle)

    keep = np.ones(n, dtype=bool)
    if outage:
        keep = (t < outage[0]) | (t >= outage[1])
    return {"t": t[keep], "x": x[keep], "y": y[keep]}


def simulate_imu(truth, a, rng):
    """Longitudinal accel and yaw rate at OUT_HZ, with white noise and a constant gyro bias."""
    n = len(a)
    return {
        "t": truth["t"],
        "a": a + rng.normal(0, ACCEL_SIGMA, n),
        "yaw_rate": truth["psi_dot"] + truth["b_yaw"] + rng.normal(0, GYRO_SIGMA, n),
    }


def save_csv(path, columns):
    """Write a dict of equal-length arrays as CSV with a header row."""
    np.savetxt(path, np.column_stack(list(columns.values())), delimiter=",",
               header=",".join(columns), comments="", fmt="%.6f")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("data/sim"))
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--gps_outage", type=float, nargs=2, metavar=("START", "END"),
                        help="drop GPS fixes with START <= t < END")
    parser.add_argument("--outlier_rate", type=float, default=0.0,
                        help="fraction of GPS fixes displaced by 30 m")
    parser.add_argument("--turn_rate", type=float, default=TURN_RATE,
                        help="yaw rate of the two turns in rad/s")
    parser.add_argument("--gyro_bias", type=float, default=GYRO_BIAS,
                        help="constant bias of the yaw rate gyro in rad/s")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    rng = np.random.default_rng(args.seed)
    truth, a = simulate_truth(args.turn_rate, args.gyro_bias)
    save_csv(args.out / "truth.csv", truth)
    save_csv(args.out / "gps.csv", simulate_gps(truth, rng, args.gps_outage, args.outlier_rate))
    save_csv(args.out / "imu.csv", simulate_imu(truth, a, rng))


if __name__ == "__main__":
    main()
