#pragma once

#include "filters/filter.h"

// Unscented Kalman filter with the CTRV model. Same state, inputs and noise
// model as CtrvEkf, but it propagates sigma points instead of using Jacobians.
// State: [x, y, v, psi, b_yaw].
class CtrvUkf : public Filter {
public:
    // Variances of the IMU noise, of the GPS noise per axis and of the initial
    // gyro bias, followed by the sigma point parameters (see unscented.h).
    CtrvUkf(double accel_var, double gyro_var, double gps_var, double bias_var0,
            double alpha, double beta, double kappa);

    bool initialize(const Eigen::Vector2d& z0, const Eigen::Vector2d& z1, double dt,
                    const Eigen::Vector2d& end_minus_mean) override;
    void predict(const Eigen::Vector2d& u, double dt) override;
    double update(const Eigen::Vector2d& z, double gate) override;

    std::unique_ptr<Filter> clone() const override { return std::make_unique<CtrvUkf>(*this); }
    void setInputNoise(double accel_var, double gyro_var) override {
        Q_imu_ = Eigen::Vector2d(accel_var, gyro_var).asDiagonal();
    }

    const Eigen::VectorXd& state() const override { return x_; }
    const Eigen::MatrixXd& covariance() const override { return P_; }
    std::vector<std::string> stateNames() const override { return {"x", "y", "v", "psi", "b_yaw"}; }

private:
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
    Eigen::Matrix2d Q_imu_;
    double gps_var_;
    double bias_var0_;
    double alpha_;
    double beta_;
    double kappa_;
};
