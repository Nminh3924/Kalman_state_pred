#include "filters/ctrv_ekf.h"

#include "models/ctrv.h"

CtrvEkf::CtrvEkf(double accel_var, double gyro_var, double gps_var, double bias_var0)
    : Q_imu_(Eigen::Vector2d(accel_var, gyro_var).asDiagonal()),
      gps_var_(gps_var),
      bias_var0_(bias_var0) {}

bool CtrvEkf::initialize(const Eigen::Vector2d& z0, const Eigen::Vector2d& z1, double dt,
                         const Eigen::Vector2d& end_minus_mean) {
    const std::optional<ctrv::Gaussian> init = ctrv::initialEstimate(z0, z1, dt, end_minus_mean, gps_var_, bias_var0_);
    if (!init) return false;
    kf_.x = init->x;
    kf_.P = init->P;
    return true;
}

void CtrvEkf::predict(const Eigen::Vector2d& u, double dt) {
    // Linearize around the current estimate, before it is moved.
    const ctrv::State x = kf_.x;
    const ctrv::Covariance F = ctrv::F(x, dt);
    const Eigen::Matrix<double, 5, 2> G = ctrv::G(dt);

    kf_.x = ctrv::f(x, u, dt);
    kf_.P = F * kf_.P * F.transpose() + G * Q_imu_ * G.transpose();
    kf_.x(ctrv::PSI) = ctrv::wrapAngle(kf_.x(ctrv::PSI));
}

double CtrvEkf::update(const Eigen::Vector2d& z, double gate) {
    // The GPS measurement is linear in the state, so this is the plain KF update.
    const double nis = kf_.update(z, ctrv::H(), gps_var_ * Eigen::Matrix2d::Identity(), gate);
    kf_.x(ctrv::PSI) = ctrv::wrapAngle(kf_.x(ctrv::PSI));
    return nis;
}
