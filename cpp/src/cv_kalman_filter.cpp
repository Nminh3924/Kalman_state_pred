#include "filters/cv_kalman_filter.h"

#include "models/constant_velocity.h"

CvKalmanFilter::CvKalmanFilter(double sigma_a, double gps_var)
    : sigma_a_(sigma_a), gps_var_(gps_var) {}

// The IMU correction is not used: this filter ignores the IMU altogether.
bool CvKalmanFilter::initialize(const Eigen::Vector2d& z0, const Eigen::Vector2d& z1, double dt,
                                const Eigen::Vector2d&) {
    const Eigen::Vector2d v = (z1 - z0) / dt;
    const double v_var = 2 * gps_var_ / (dt * dt);
    kf_.x = Eigen::Vector4d(z1(0), z1(1), v(0), v(1));
    kf_.P = Eigen::Vector4d(gps_var_, gps_var_, v_var, v_var).asDiagonal();
    // The later fix gives the position and also enters the velocity, so the
    // errors of the two are correlated.
    kf_.P(0, 2) = kf_.P(2, 0) = gps_var_ / dt;
    kf_.P(1, 3) = kf_.P(3, 1) = gps_var_ / dt;
    return true;
}

void CvKalmanFilter::predict(const Eigen::Vector2d&, double dt) {
    kf_.predict(cv_model::F(dt), cv_model::Q(dt, sigma_a_));
}

double CvKalmanFilter::update(const Eigen::Vector2d& z, double gate) {
    return kf_.update(z, cv_model::H(), gps_var_ * Eigen::Matrix2d::Identity(), gate);
}
