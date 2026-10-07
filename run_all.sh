#!/usr/bin/env bash
# Rebuilds everything in results/: build, unit tests, simulated data, all
# filters, the Monte Carlo analyses, the plots and the demo animation.
#
#   ./run_all.sh           simulator only
#   ./run_all.sh --kitti   also fetch three KITTI sequences (about 40 MB) and evaluate on them
#
# Needs CMake, Eigen 3, GoogleTest and the Python packages of python/requirements.txt.
set -euo pipefail
cd "$(dirname "$0")"

PY=python3
if [ -x .venv/bin/python ]; then PY=.venv/bin/python; fi
MAIN=build/main
TABLES=results/tables
mkdir -p "$TABLES"

# Runs the three filters on one data directory. Extra arguments go to each filter.
run_filters() {
    local dir=$1
    shift
    rm -f "$dir"/estimate_*.csv "$dir"/forecast_*.csv
    for filter in kf ekf ukf; do
        "$MAIN" "$filter" "$dir" "$@" > /dev/null
    done
}

echo "== Build and unit tests =="
cmake -S cpp -B build > /dev/null
cmake --build build
ctest --test-dir build --output-on-failure

echo "== One simulated run of each scenario: plots =="
$PY python/simulate.py --out data/sim
run_filters data/sim
"$MAIN" ekf_nobias data/sim > /dev/null
$PY python/evaluate.py --data data/sim | tee "$TABLES/single_run.md"
$PY python/plot.py --data data/sim --out results

$PY python/simulate.py --turn_rate 0.6 --out data/sim_sharp
run_filters data/sim_sharp
$PY python/plot.py --data data/sim_sharp --out results/sharp_turn

$PY python/simulate.py --gps_outage 30 40 --out data/sim_outage
run_filters data/sim_outage
$PY python/plot.py --data data/sim_outage --out results/outage

$PY python/simulate.py --outlier_rate 0.05 --out data/sim_outlier_no_gate
run_filters data/sim_outlier_no_gate
$PY python/plot.py --data data/sim_outlier_no_gate --out results/outliers/no_gate

$PY python/simulate.py --outlier_rate 0.05 --out data/sim_outlier_gated
run_filters data/sim_outlier_gated --gate 9.21
$PY python/plot.py --data data/sim_outlier_gated --out results/outliers/gated

echo "== Monte Carlo on the simulator =="
$PY python/monte_carlo.py --runs 200 | tee "$TABLES/comparison.md"
$PY python/monte_carlo.py --runs 200 --turn_rate 0.6 | tee "$TABLES/sharp_turn.md"
$PY python/monte_carlo.py --runs 200 --gps_outage 30 40 | tee "$TABLES/outage.md"
$PY python/monte_carlo.py --runs 200 --gate 9.21 --outlier_rate 0.05 | tee "$TABLES/outliers.md"
$PY python/consistency.py --runs 100 | tee "$TABLES/consistency.md"
$PY python/tune.py --runs 30 | tee "$TABLES/tuning.md"
$PY python/forecast.py --runs 50 --plot results/forecast.png --window 4 26 | tee "$TABLES/forecast.md"

echo "== Demo animation =="
$PY python/simulate.py --gps_outage 30 40 --out data/sim_demo
"$MAIN" ekf data/sim_demo --forecast 10 > /dev/null
$PY python/animate.py --data data/sim_demo --out results/demo.gif

if [ "${1:-}" = "--kitti" ]; then
    echo "== KITTI =="
    # Noise settings for this sensor, tuned on drive 0027 only (see README).
    KITTI="--accel_sigma 0.4 --gyro_sigma 0.02 --forecast_accel 2 --forecast_yaw 0.2"
    $PY data/download_kitti.py
    for drive in 2011_09_30_drive_0027 2011_09_30_drive_0033 2011_09_30_drive_0034; do
        dir=data/kitti/$drive
        $PY python/kitti_convert.py --oxts "$dir/oxts" --out "$dir" | tee "$TABLES/kitti_${drive}_data.md"
        # shellcheck disable=SC2086
        run_filters "$dir" $KITTI
        $PY python/plot.py --data "$dir" --out "results/kitti/$drive"
        $PY python/monte_carlo.py --runs 20 --generator python/kitti_convert.py --main_args="$KITTI" \
            --oxts "$dir/oxts" --quiet | tee "$TABLES/kitti_${drive}.md"
        $PY python/forecast.py --runs 10 --generator python/kitti_convert.py --main_args="$KITTI" \
            --oxts "$dir/oxts" --quiet --plot "results/kitti/$drive/forecast.png" --window 20 50 \
            | tee "$TABLES/kitti_${drive}_forecast.md"
    done
fi

echo "== Done: see results/ =="
