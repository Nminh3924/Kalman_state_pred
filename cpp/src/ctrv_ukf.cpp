#include "filters/ctrv_ukf.h"

#include "filters/unscented.h"
#include "models/ctrv.h"

CtrvUkf::CtrvUkf(double accel_var, double gyro_var, double gps_var, double bias_var0,
                 double alpha, double beta, double kappa)
    : Q_imu_(Eigen::Vector2d(accel_var, gyro_var).asDiagonal()),
      gps_var_(gps_var),
      bias_var0_(bias_var0),
      alpha_(alpha),
      beta_(beta),
      kappa_(kappa) {}

bool CtrvUkf::initialize(const Eigen::Vector2d& z0, const Eigen::Vector2d& z1, double dt,
                         const Eigen::Vector2d& end_minus_mean) {
    const std::optional<ctrv::Gaussian> init = ctrv::initialEstimate(z0, z1, dt, end_minus_mean, gps_var_, bias_var0_);
    if (!init) return false;
    x_ = init->x;
    P_ = init->P;
    return true;
}

void CtrvUkf::predict(const Eigen::Vector2d& u, double dt) {
    // Push every sigma point through the motion model...
    SigmaPoints s = sigmaPoints(x_, P_, alpha_, beta_, kappa_);
    for (int i = 0; i < s.points.cols(); ++i) s.points.col(i) = ctrv::f(s.points.col(i), u, dt);

    // ...and read the new mean and covariance off the transformed points.
    const ctrv::State mean = ctrv::weightedMean(s.points, s.mean_weights);
    const Eigen::MatrixXd d = ctrv::residuals(s.points, mean);
    // The IMU noise enters f linearly, so it adds to the covariance as G Q G'.
    const Eigen::Matrix<double, 5, 2> G = ctrv::G(dt);
    x_ = mean;
    P_ = d * s.cov_weights.asDiagonal() * d.transpose() + G * Q_imu_ * G.transpose();
}

double CtrvUkf::update(const Eigen::Vector2d& z, double gate) {
    // Push every sigma point through the measurement model.
    const SigmaPoints s = sigmaPoints(x_, P_, alpha_, beta_, kappa_);
    Eigen::MatrixXd predicted(2, s.points.cols());
    for (int i = 0; i < s.points.cols(); ++i) predicted.col(i) = ctrv::h(s.points.col(i));

    const Eigen::Vector2d z_mean = predicted * s.mean_weights;
    const Eigen::MatrixXd dz = predicted.colwise() - z_mean;
    const Eigen::MatrixXd dx = ctrv::residuals(s.points, x_);

    const Eigen::Matrix2d S = dz * s.cov_weights.asDiagonal() * dz.transpose()
                              + gps_var_ * Eigen::Matrix2d::Identity();        // innovation covariance
    const Eigen::Matrix2d S_inv = S.inverse();
    const Eigen::Vector2d y = z - z_mean;                                       // innovation
    const double nis = y.dot(S_inv * y);
    if (nis > gate) return nis;

    const Eigen::MatrixXd Pxz = dx * s.cov_weights.asDiagonal() * dz.transpose();  // state-measurement covariance
    const Eigen::MatrixXd K = Pxz * S_inv;                                      // Kalman gain
    x_ += K * y;
    P_ -= K * S * K.transpose();
    x_(ctrv::PSI) = ctrv::wrapAngle(x_(ctrv::PSI));
    return nis;
}
