#include <gtest/gtest.h>

#include "filters/ctrv_ekf.h"
#include "filters/cv_kalman_filter.h"
#include "forecast.h"

namespace {

const Eigen::Vector2d NO_IMU = Eigen::Vector2d::Zero();

// Filters at the origin, moving along +x at 10 m/s: 50 m in the 5 s between two fixes.
CtrvEkf ekfAtOrigin() {
    CtrvEkf ekf(0.1 * 0.1, 0.01 * 0.01, 3.0 * 3.0, 0.05 * 0.05);
    EXPECT_TRUE(ekf.initialize(Eigen::Vector2d(-50, 0), Eigen::Vector2d(0, 0), 5.0, NO_IMU));
    return ekf;
}

CvKalmanFilter kfAtOrigin() {
    CvKalmanFilter kf(3.0, 3.0 * 3.0);
    EXPECT_TRUE(kf.initialize(Eigen::Vector2d(-50, 0), Eigen::Vector2d(0, 0), 5.0, NO_IMU));
    return kf;
}

}  // namespace

TEST(Forecast, HoldingTheYawRateFollowsACircle) {
    const CtrvEkf ekf = ekfAtOrigin();

    const std::vector<Forecast> f = forecast(ekf, Eigen::Vector2d(0.0, 0.2), ForecastOptions());

    // 10 m/s at 0.2 rad/s is a circle of radius 50 m; after 3 s the vehicle has
    // turned 0.6 rad: (50 sin 0.6, 50 (1 - cos 0.6)).
    ASSERT_EQ(f.size(), 6u);
    EXPECT_DOUBLE_EQ(f.back().horizon, 3.0);
    EXPECT_NEAR(f.back().position(0), 28.232, 0.1);
    EXPECT_NEAR(f.back().position(1), 8.733, 0.1);
}

TEST(Forecast, WithoutYawRateGoesStraight) {
    const CtrvEkf ekf = ekfAtOrigin();

    const std::vector<Forecast> f = forecast(ekf, Eigen::Vector2d::Zero(), ForecastOptions());

    ASSERT_EQ(f.size(), 6u);
    EXPECT_DOUBLE_EQ(f.front().horizon, 0.5);
    EXPECT_NEAR(f.front().position(0), 5.0, 1e-9);
    EXPECT_NEAR(f.back().position(0), 30.0, 1e-9);
    EXPECT_NEAR(f.back().position(1), 0.0, 1e-9);
}

TEST(Forecast, ConstantVelocityFilterGoesStraightWhateverTheYawRate) {
    const CvKalmanFilter kf = kfAtOrigin();

    const std::vector<Forecast> f = forecast(kf, Eigen::Vector2d(0.0, 0.2), ForecastOptions());

    ASSERT_EQ(f.size(), 6u);
    EXPECT_NEAR(f.back().position(0), 30.0, 1e-9);
    EXPECT_NEAR(f.back().position(1), 0.0, 1e-9);
}

TEST(Forecast, LeavesTheFilterUntouched) {
    const CtrvEkf ekf = ekfAtOrigin();
    const Eigen::VectorXd x = ekf.state();
    const Eigen::MatrixXd P = ekf.covariance();

    ASSERT_FALSE(forecast(ekf, Eigen::Vector2d(0.0, 0.2), ForecastOptions()).empty());

    EXPECT_EQ(ekf.state(), x);
    EXPECT_EQ(ekf.covariance(), P);
}

TEST(Forecast, PositionVarianceOfTheConstantVelocityFilter) {
    const CvKalmanFilter kf = kfAtOrigin();

    const std::vector<Forecast> f = forecast(kf, Eigen::Vector2d::Zero(), ForecastOptions());

    // Along x after h = 1 s: var(x) + 2 h cov(x, v) + h^2 var(v) + sigma_a^2 h^3 / 3
    //                      = 9 + 2 * 9/5 + 18/25 + 9/3.
    ASSERT_EQ(f.size(), 6u);
    EXPECT_DOUBLE_EQ(f[1].horizon, 1.0);
    EXPECT_NEAR(f[1].P(0, 0), 16.32, 1e-9);
}

TEST(Forecast, ManeuverNoiseOnTheAccelerationActsLikeTheConstantVelocityModel) {
    const CtrvEkf ekf = ekfAtOrigin();
    ForecastOptions options;
    options.accel_density = 3.0;

    const std::vector<Forecast> f = forecast(ekf, Eigen::Vector2d::Zero(), options);

    // Heading along x, so the same four terms as for the constant-velocity
    // filter with sigma_a = 3; summing in 0.01 s steps comes out a little low.
    ASSERT_EQ(f.size(), 6u);
    EXPECT_NEAR(f[1].P(0, 0), 16.32, 0.1);
}

TEST(Forecast, UncertaintyGrowsWithTheHorizon) {
    const CtrvEkf ekf = ekfAtOrigin();
    ForecastOptions options;
    options.accel_density = 1.0;
    options.yaw_rate_density = 0.1;

    const std::vector<Forecast> f = forecast(ekf, Eigen::Vector2d(0.0, 0.2), options);

    ASSERT_EQ(f.size(), 6u);
    for (size_t i = 1; i < f.size(); ++i) EXPECT_GT(f[i].P.trace(), f[i - 1].P.trace());
}
