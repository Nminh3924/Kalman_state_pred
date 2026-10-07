#pragma once

#include <vector>

#include <Eigen/Dense>

#include "filters/filter.h"

// Where the vehicle is expected to be `horizon` seconds ahead, and how sure.
struct Forecast {
    double horizon;
    Eigen::Vector2d position;
    Eigen::Matrix2d P;  // covariance of the position
};

struct ForecastOptions {
    std::vector<double> horizons{0.5, 1.0, 1.5, 2.0, 2.5, 3.0};  // s, increasing
    double step = 0.01;                                         // s, prediction step
    // A forecast holds the input, but the driver will not. For the filters
    // that are driven by the IMU, what the driver may do is modelled as white
    // noise on the acceleration and on the yaw rate, with these densities.
    double accel_density = 0.0;     // m/s^2 per sqrt(Hz)
    double yaw_rate_density = 0.0;  // rad/s per sqrt(Hz)
};

// Predicts ahead from the filter's current estimate without any measurement,
// holding the IMU input u = [a, yaw_rate]. The filter itself is not changed.
std::vector<Forecast> forecast(const Filter& filter, const Eigen::Vector2d& u,
                               const ForecastOptions& options);
