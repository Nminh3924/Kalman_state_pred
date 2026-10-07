#pragma once

#include "filters/filter.h"
#include "filters/kalman_filter.h"

// Linear Kalman filter with the constant-velocity model. It ignores the IMU.
// State: [x, y, vx, vy].
class CvKalmanFilter : public Filter {
public:
    CvKalmanFilter(double sigma_a, double gps_var);

    bool initialize(const Eigen::Vector2d& z0, const Eigen::Vector2d& z1, double dt,
                    const Eigen::Vector2d& end_minus_mean) override;
    void predict(const Eigen::Vector2d& u, double dt) override;
    double update(const Eigen::Vector2d& z, double gate) override;

    std::unique_ptr<Filter> clone() const override { return std::make_unique<CvKalmanFilter>(*this); }
    void setInputNoise(double, double) override {}

    const Eigen::VectorXd& state() const override { return kf_.x; }
    const Eigen::MatrixXd& covariance() const override { return kf_.P; }
    std::vector<std::string> stateNames() const override { return {"x", "y", "vx", "vy"}; }

private:
    KalmanFilter kf_;
    double sigma_a_;
    double gps_var_;
};
