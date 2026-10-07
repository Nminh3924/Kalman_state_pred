#pragma once

#include "filters/filter.h"
#include "filters/kalman_filter.h"

// Extended Kalman filter with the CTRV model: the IMU drives the prediction,
// GPS fixes correct it. State: [x, y, v, psi, b_yaw].
class CtrvEkf : public Filter {
public:
    // Variances of the IMU noise, of the GPS noise per axis, and of the initial
    // gyro bias. A bias variance of zero switches bias estimation off.
    CtrvEkf(double accel_var, double gyro_var, double gps_var, double bias_var0);

    bool initialize(const Eigen::Vector2d& z0, const Eigen::Vector2d& z1, double dt,
                    const Eigen::Vector2d& end_minus_mean) override;
    void predict(const Eigen::Vector2d& u, double dt) override;
    double update(const Eigen::Vector2d& z, double gate) override;

    std::unique_ptr<Filter> clone() const override { return std::make_unique<CtrvEkf>(*this); }
    void setInputNoise(double accel_var, double gyro_var) override {
        Q_imu_ = Eigen::Vector2d(accel_var, gyro_var).asDiagonal();
    }

    const Eigen::VectorXd& state() const override { return kf_.x; }
    const Eigen::MatrixXd& covariance() const override { return kf_.P; }
    std::vector<std::string> stateNames() const override { return {"x", "y", "v", "psi", "b_yaw"}; }

private:
    KalmanFilter kf_;
    Eigen::Matrix2d Q_imu_;
    double gps_var_;
    double bias_var0_;
};
