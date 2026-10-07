#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "filters/ctrv_ekf.h"
#include "filters/ctrv_ukf.h"
#include "filters/cv_kalman_filter.h"
#include "forecast.h"
#include "io.h"
#include "run.h"

const double GPS_VAR = 3.0 * 3.0;      // m^2, matches GPS_SIGMA in simulate.py
const double BIAS_VAR0 = 0.05 * 0.05;  // (rad/s)^2, prior on the gyro bias

// Sigma point parameters. alpha = 1 spreads the points over the actual
// uncertainty, and with kappa = 0 no weight is negative, which keeps P valid.
const double UKF_ALPHA = 1.0;
const double UKF_BETA = 2.0;
const double UKF_KAPPA = 0.0;

struct Options {
    double sigma_a = 3.0;       // m/s^2, process noise of kf
    // Noise of one IMU sample as ekf and ukf model it. The defaults are the
    // simulator's; a real sensor needs its own values, which also have to cover
    // what the motion model leaves out.
    double accel_sigma = 0.1;   // m/s^2
    double gyro_sigma = 0.01;   // rad/s
    double gate = NO_GATE;      // GPS fixes with a larger NIS are rejected
    std::string label;          // names the output file; defaults to the filter name
    int forecast_every = 0;     // forecast from every n-th estimate; 0 = no forecasts
    ForecastOptions forecast;   // the maneuver noise is used by ekf and ukf only
};

// One forecast with where it was made from. `held` tells the kinds apart:
// 0 = straight ahead at constant speed, 1 = the current yaw rate is held (a
// turn continues), 2 = the current acceleration is held as well.
struct ForecastRow {
    double t0;
    int held;
    Forecast f;
};

static std::unique_ptr<Filter> makeFilter(const std::string& name, const Options& o) {
    const double accel_var = o.accel_sigma * o.accel_sigma;
    const double gyro_var = o.gyro_sigma * o.gyro_sigma;
    if (name == "kf") return std::make_unique<CvKalmanFilter>(o.sigma_a, GPS_VAR);
    if (name == "ekf") return std::make_unique<CtrvEkf>(accel_var, gyro_var, GPS_VAR, BIAS_VAR0);
    if (name == "ekf_nobias") return std::make_unique<CtrvEkf>(accel_var, gyro_var, GPS_VAR, 0.0);
    if (name == "ukf") {
        return std::make_unique<CtrvUkf>(accel_var, gyro_var, GPS_VAR, BIAS_VAR0,
                                         UKF_ALPHA, UKF_BETA, UKF_KAPPA);
    }
    throw std::invalid_argument("unknown filter: " + name);
}

// Runs one filter on the measurements in <data_dir>, writes its estimates to
// <data_dir>/estimate_<label>.csv and prints its run time per step.
int main(int argc, char** argv) {
    const std::string usage =
        "usage: main <kf|ekf|ekf_nobias|ukf> [data_dir] [--sigma_a X] [--accel_sigma X]\n"
        "            [--gyro_sigma X] [--gate X] [--label NAME]\n"
        "            [--forecast N] [--forecast_accel X] [--forecast_yaw X]\n";
    if (argc < 2) {
        std::cerr << usage;
        return 1;
    }
    const std::string name = argv[1];
    int next = 2;
    std::filesystem::path dir = "data/sim";
    if (next < argc && std::string(argv[next]).rfind("--", 0) != 0) dir = argv[next++];

    Options options;
    options.label = name;
    // Chosen on simulated runs so that the 95% ellipses hold about 95% of the
    // true positions at every horizon.
    options.forecast.accel_density = 1.0;
    options.forecast.yaw_rate_density = 0.06;
    for (; next + 1 < argc; next += 2) {
        const std::string flag = argv[next];
        const std::string value = argv[next + 1];
        if (flag == "--sigma_a") {
            options.sigma_a = std::stod(value);
        } else if (flag == "--accel_sigma") {
            options.accel_sigma = std::stod(value);
        } else if (flag == "--gyro_sigma") {
            options.gyro_sigma = std::stod(value);
        } else if (flag == "--gate") {
            options.gate = std::stod(value);
        } else if (flag == "--label") {
            options.label = value;
        } else if (flag == "--forecast") {
            options.forecast_every = std::stoi(value);
        } else if (flag == "--forecast_accel") {
            options.forecast.accel_density = std::stod(value);
        } else if (flag == "--forecast_yaw") {
            options.forecast.yaw_rate_density = std::stod(value);
        } else {
            break;
        }
    }
    if (next != argc) {
        std::cerr << usage;
        return 1;
    }

    const std::unique_ptr<Filter> filter = makeFilter(name, options);
    const std::vector<Measurement> measurements = loadMeasurements(dir);

    // Forecasts are made while the filter runs, from every n-th estimate.
    const std::vector<std::string> names = filter->stateNames();
    const auto bias = std::find(names.begin(), names.end(), "b_yaw");
    std::vector<ForecastRow> forecasts;
    int reported = 0;
    Observer observer = nullptr;
    if (options.forecast_every > 0) {
        observer = [&](double t, const Filter& f, const Eigen::Vector2d& u) {
            if (reported++ % options.forecast_every != 0) return;
            // Straight ahead means a true yaw rate of zero, which the gyro
            // would report as its bias.
            const double b = bias == names.end() ? 0.0 : f.state()(bias - names.begin());
            std::vector<Eigen::Vector2d> inputs{Eigen::Vector2d(0.0, b)};
            if (name != "kf") {  // the constant-velocity filter has nothing else to hold
                inputs.push_back(Eigen::Vector2d(0.0, u(1)));
                inputs.push_back(u);
            }
            for (size_t held = 0; held < inputs.size(); ++held) {
                for (const Forecast& ahead : forecast(f, inputs[held], options.forecast)) {
                    forecasts.push_back({t, static_cast<int>(held), ahead});
                }
            }
        };
    }

    const auto start = std::chrono::steady_clock::now();
    const std::vector<Estimate> estimates = run(*filter, measurements, options.gate, observer);
    const std::chrono::duration<double, std::micro> elapsed = std::chrono::steady_clock::now() - start;
    std::cout << options.label << ' ' << elapsed.count() / estimates.size() << " us/step\n";

    // Columns: t, the states, the upper triangle of P as P00, P01, ..., and the NIS.
    const int n = names.size();
    std::ofstream out(dir / ("estimate_" + options.label + ".csv"));
    out << "t";
    for (const std::string& s : names) out << ',' << s;
    for (int i = 0; i < n; ++i) {
        for (int j = i; j < n; ++j) out << ",P" << i << j;
    }
    out << ",nis\n" << std::setprecision(10);
    for (const Estimate& e : estimates) {
        out << e.t;
        for (int i = 0; i < n; ++i) out << ',' << e.x(i);
        for (int i = 0; i < n; ++i) {
            for (int j = i; j < n; ++j) out << ',' << e.P(i, j);
        }
        out << ',' << e.nis << '\n';
    }

    if (options.forecast_every > 0) {
        std::ofstream ahead(dir / ("forecast_" + options.label + ".csv"));
        ahead << "t0,held,horizon,x,y,Pxx,Pxy,Pyy\n" << std::setprecision(10);
        for (const ForecastRow& row : forecasts) {
            ahead << row.t0 << ',' << row.held << ',' << row.f.horizon << ',' << row.f.position(0) << ','
                  << row.f.position(1) << ',' << row.f.P(0, 0) << ',' << row.f.P(0, 1) << ','
                  << row.f.P(1, 1) << '\n';
        }
    }
}
