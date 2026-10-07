"""Run every filter on many simulated runs; tabulate position RMSE and run time."""
import argparse
import shlex
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd

from evaluate import COLUMNS, SOURCES, load_errors, print_table, summarize, table_row

FILTERS = ["kf", "ekf", "ukf"]
SIMULATE = Path(__file__).parent / "simulate.py"


def simulate(out, seed, extra=(), generator=SIMULATE):
    """Write one run to a directory: simulated, or made by another generator script."""
    subprocess.run([sys.executable, generator, "--seed", str(seed), "--out", out, *extra], check=True)


def run_filter(main, name, data, extra=()):
    """Run one filter on the data in a directory. Returns its run time in microseconds per step."""
    done = subprocess.run([main, name, data, *extra], check=True, capture_output=True, text=True)
    return float(done.stdout.split()[1])


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        epilog="Any other argument is passed on to the generator, for example --turn_rate 0.6.")
    parser.add_argument("--runs", type=int, default=100)
    parser.add_argument("--generator", type=Path, default=SIMULATE,
                        help="script that writes one run; kitti_convert.py for recorded data")
    parser.add_argument("--main_args", default="",
                        help='arguments for every filter, as one string: --main_args="--gyro_sigma 0.05"')
    parser.add_argument("--main", type=Path, default=Path("build/main"), help="filter executable")
    parser.add_argument("--gate", type=float,
                        help="also run every filter with this NIS gate, for example 9.21")
    args, simulate_args = parser.parse_known_args()

    # (filter, output label, extra arguments)
    common = shlex.split(args.main_args)
    variants = [(name, name, common) for name in FILTERS]
    if args.gate is not None:
        variants += [(name, f"{name}_gated",
                      [*common, "--gate", str(args.gate), "--label", f"{name}_gated"])
                     for name in FILTERS]

    errors = {}  # source label -> one error table per run
    micros = {}  # output label -> microseconds per step of each run
    with tempfile.TemporaryDirectory() as tmp:
        for seed in range(args.runs):
            simulate(tmp, seed, simulate_args, args.generator)
            for name, label, extra in variants:
                micros.setdefault(label, []).append(run_filter(args.main, name, tmp, extra))
            for label, table in load_errors(Path(tmp)).items():
                errors.setdefault(label, []).append(table)

    print(f"Position RMSE in metres, {args.runs} runs pooled"
          + (f", {args.generator.name} {' '.join(simulate_args)}" if simulate_args else "")
          + (f", filters {args.main_args}" if args.main_args else "")
          + (f", gate {args.gate:g}" if args.gate is not None else ""))
    print()
    rows = []
    for label, file in SOURCES:
        if label not in errors:
            continue
        output_label = file.removeprefix("estimate_").removesuffix(".csv")
        us = f"{np.median(micros[output_label]):.2f}" if output_label in micros else "–"
        rows.append(table_row(label, summarize(pd.concat(errors[label]))) + [us])
    print_table([""] + COLUMNS + ["µs/step"], rows)


if __name__ == "__main__":
    main()
