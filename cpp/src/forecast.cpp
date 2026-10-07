#include "forecast.h"

#include <cmath>

std::vector<Forecast> forecast(const Filter& filter, const Eigen::Vector2d& u,
                               const ForecastOptions& options) {
    const std::unique_ptr<Filter> ahead = filter.clone();
    // White noise of density d acts over a step of length h like one sample of
    // variance d^2 / h. This replaces the IMU noise: the input is now a guess.
    ahead->setInputNoise(options.accel_density * options.accel_density / options.step,
                         options.yaw_rate_density * options.yaw_rate_density / options.step);

    std::vector<Forecast> forecasts;
    long steps = 0;
    for (const double horizon : options.horizons) {
        for (; steps < std::lround(horizon / options.step); ++steps) ahead->predict(u, options.step);
        forecasts.push_back({horizon, ahead->state().head<2>(),
                             ahead->covariance().topLeftCorner<2, 2>()});
    }
    return forecasts;
}
