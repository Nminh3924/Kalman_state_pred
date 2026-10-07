"""Convert a KITTI raw OXTS recording to the CSV files the filters read.

The recorded RTK-GPS/INS solution serves as ground truth. The GPS the filters
see is made from it, sampled at 1 Hz with noise added like in the simulator.
The forward acceleration and the yaw rate are passed on unchanged as the IMU.
"""
import argparse
from pathlib import Path

import numpy as np
import pandas as pd

from simulate import GPS_SIGMA, save_csv, simulate_gps

# Columns of an OXTS sample that are used here (see oxts/dataformat.txt).
LAT, LON, ALT, YAW, VF, AF, WU, POS_ACCURACY = 0, 1, 2, 5, 8, 14, 22, 23


def load_oxts(oxts):
    """Time in seconds since the first sample, and the 30 values of every sample."""
    stamps = pd.to_datetime((oxts / "timestamps.txt").read_text().splitlines())
    t = (stamps - stamps[0]).total_seconds().to_numpy()
    samples = np.array([[float(v) for v in path.read_text().split()]
                        for path in sorted((oxts / "data").glob("*.txt"))])
    # Keep only samples whose timestamp moves forward.
    keep = t > np.maximum.accumulate(np.concatenate(([-np.inf], t[:-1])))
    return t[keep], samples[keep], int((~keep).sum())


def geodetic_to_enu(lat, lon, alt):
    """East and north in metres from the first sample, on the WGS84 ellipsoid."""
    a = 6378137.0                    # equatorial radius
    e2 = 6.69437999014e-3            # eccentricity squared
    lat, lon = np.radians(lat), np.radians(lon)
    n = a / np.sqrt(1 - e2 * np.sin(lat) ** 2)
    ecef = np.column_stack([(n + alt) * np.cos(lat) * np.cos(lon),
                            (n + alt) * np.cos(lat) * np.sin(lon),
                            (n * (1 - e2) + alt) * np.sin(lat)])
    dx, dy, dz = (ecef - ecef[0]).T
    lat0, lon0 = lat[0], lon[0]
    east = -np.sin(lon0) * dx + np.cos(lon0) * dy
    north = -np.sin(lat0) * np.cos(lon0) * dx - np.sin(lat0) * np.sin(lon0) * dy + np.cos(lat0) * dz
    return east, north


def report(t, samples, truth, dropped):
    """Print what was read, and checks of the conversion against independent columns."""
    dt = np.diff(t)
    x, y, v, psi = truth["x"], truth["y"], truth["v"], truth["psi"]
    step = np.hypot(np.diff(x), np.diff(y))
    print(f"{len(t)} samples over {t[-1]:.1f} s ({1 / np.median(dt):.0f} Hz, longest gap {dt.max():.2f} s, "
          f"{dropped} dropped), reported position accuracy {np.median(samples[:, POS_ACCURACY]):.2f} m")
    print(f"path {step.sum():.0f} m, speed {v.min():.1f} to {v.max():.1f} m/s, "
          f"turning faster than 0.05 rad/s {np.mean(np.abs(truth['psi_dot']) > 0.05):.0%} of the time")

    print("checks:")
    print(f"  ENU path length vs integral of forward speed: {step.sum():.1f} m vs {np.sum(v[:-1] * dt):.1f} m")
    moving = step / dt > 2.0
    course = np.arctan2(np.diff(y), np.diff(x))[moving]
    off = np.remainder(course - psi[:-1][moving] + np.pi, 2 * np.pi) - np.pi
    print(f"  direction of travel minus yaw: mean {np.degrees(off.mean()):+.2f} deg, "
          f"std {np.degrees(off.std()):.2f} deg")
    # Compare the IMU channels with derivatives of the truth over 1 s windows.
    k = np.arange(0, len(t) - 100, 100)
    span = t[k + 100] - t[k]
    mean_a = np.array([samples[i:i + 100, AF].mean() for i in k])
    mean_w = np.array([samples[i:i + 100, WU].mean() for i in k])
    da = mean_a - (v[k + 100] - v[k]) / span
    dw = mean_w - (psi[k + 100] - psi[k]) / span
    print(f"  forward accel minus d(speed)/dt, 1 s means: mean {da.mean():+.3f}, std {da.std():.3f} m/s^2")
    print(f"  yaw rate minus d(yaw)/dt, 1 s means: mean {dw.mean():+.5f}, std {dw.std():.5f} rad/s")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--oxts", type=Path, required=True, help="the oxts folder of a recording")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--gps_outage", type=float, nargs=2, metavar=("START", "END"),
                        help="drop GPS fixes with START <= t < END")
    parser.add_argument("--outlier_rate", type=float, default=0.0,
                        help="fraction of GPS fixes displaced by 30 m")
    parser.add_argument("--quiet", action="store_true", help="do not print the report")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    t, samples, dropped = load_oxts(args.oxts)
    east, north = geodetic_to_enu(samples[:, LAT], samples[:, LON], samples[:, ALT])
    truth = {"t": t, "x": east, "y": north, "v": samples[:, VF], "psi": np.unwrap(samples[:, YAW]),
             "psi_dot": samples[:, WU],
             "b_yaw": np.full(len(t), np.nan)}  # the true gyro bias is not known
    if not args.quiet:
        report(t, samples, truth, dropped)

    rng = np.random.default_rng(args.seed)
    save_csv(args.out / "truth.csv", truth)
    save_csv(args.out / "gps.csv", simulate_gps(truth, rng, args.gps_outage, args.outlier_rate))
    save_csv(args.out / "imu.csv", {"t": t, "a": samples[:, AF], "yaw_rate": samples[:, WU]})
    if not args.quiet:
        print(f"wrote truth.csv, gps.csv (1 Hz, sigma {GPS_SIGMA:g} m, seed {args.seed}) and imu.csv to {args.out}")


if __name__ == "__main__":
    main()
