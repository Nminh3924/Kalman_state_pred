#pragma once

#include <Eigen/Dense>

// Constant-velocity motion model. State: [x, y, vx, vy].
namespace cv_model {

inline Eigen::Matrix4d F(double dt) {
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    m(0, 2) = dt;
    m(1, 3) = dt;
    return m;
}

// Process noise for continuous white-noise acceleration of density sigma_a.
// One step of 2*dt gives the same covariance as two steps of dt, so the tuning
// does not depend on the step size (the dt^4/4 piecewise-constant form does).
inline Eigen::Matrix4d Q(double dt, double sigma_a) {
    const double q = sigma_a * sigma_a;
    const double pp = q * dt * dt * dt / 3;
    const double pv = q * dt * dt / 2;
    const double vv = q * dt;
    Eigen::Matrix4d m;
    m << pp, 0, pv, 0,
         0, pp, 0, pv,
         pv, 0, vv, 0,
         0, pv, 0, vv;
    return m;
}

// GPS measures position only.
inline Eigen::Matrix<double, 2, 4> H() {
    Eigen::Matrix<double, 2, 4> m = Eigen::Matrix<double, 2, 4>::Zero();
    m(0, 0) = 1;
    m(1, 1) = 1;
    return m;
}

}  // namespace cv_model
