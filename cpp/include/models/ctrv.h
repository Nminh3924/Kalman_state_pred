#pragma once

#include <cmath>
#include <optional>

#include <Eigen/Dense>

// CTRV motion model driven by the IMU.
// State x = [x, y, v, psi, b_yaw], input u = [a, yaw_rate] as measured.
namespace ctrv {

enum Index { X, Y, V, PSI, B_YAW };

using State = Eigen::Matrix<double, 5, 1>;
using Covariance = Eigen::Matrix<double, 5, 5>;

struct Gaussian {
    State x;
    Covariance P;
};

// Maps an angle into [-pi, pi].
inline double wrapAngle(double angle) { return std::remainder(angle, 2 * M_PI); }

// One Euler step of the motion model. The heading is deliberately not wrapped
// here, so that f stays smooth and can be differentiated numerically.
inline State f(const State& x, const Eigen::Vector2d& u, double dt) {
    State next = x;
    next(X) += x(V) * std::cos(x(PSI)) * dt;
    next(Y) += x(V) * std::sin(x(PSI)) * dt;
    next(V) += u(0) * dt;
    next(PSI) += (u(1) - x(B_YAW)) * dt;
    return next;
}

// Jacobian of f with respect to the state.
inline Covariance F(const State& x, double dt) {
    Covariance m = Covariance::Identity();
    m(X, V) = std::cos(x(PSI)) * dt;
    m(X, PSI) = -x(V) * std::sin(x(PSI)) * dt;
    m(Y, V) = std::sin(x(PSI)) * dt;
    m(Y, PSI) = x(V) * std::cos(x(PSI)) * dt;
    m(PSI, B_YAW) = -dt;
    return m;
}

// Jacobian of f with respect to the input; maps IMU noise into the state.
inline Eigen::Matrix<double, 5, 2> G(double dt) {
    Eigen::Matrix<double, 5, 2> m = Eigen::Matrix<double, 5, 2>::Zero();
    m(V, 0) = dt;
    m(PSI, 1) = dt;
    return m;
}

// Measurement model: GPS measures position only.
inline Eigen::Vector2d h(const State& x) { return x.head<2>(); }

// Jacobian of h.
inline Eigen::Matrix<double, 2, 5> H() {
    Eigen::Matrix<double, 2, 5> m = Eigen::Matrix<double, 2, 5>::Zero();
    m(0, X) = 1;
    m(1, Y) = 1;
    return m;
}

// The filters linearize (or sample) around the heading estimate, so the initial
// heading must be known at least this well. A worse start can settle on the
// mirrored solution (v < 0, heading off by pi), which fits GPS equally well.
const double MAX_INITIAL_HEADING_SIGMA = 0.2;  // rad

// First estimate from two GPS fixes taken dt seconds apart: position of the
// later fix, speed and heading from the displacement. The displacement gives
// their means over the interval; end_minus_mean = [speed, heading], measured by
// the IMU, turns those into the values at the later fix. Returns nothing while
// the fixes are too close together to give a usable heading.
inline std::optional<Gaussian> initialEstimate(const Eigen::Vector2d& z0, const Eigen::Vector2d& z1,
                                               double dt, const Eigen::Vector2d& end_minus_mean,
                                               double gps_var, double bias_var0) {
    const Eigen::Vector2d d = z1 - z0;
    const double heading_var = 2 * gps_var / d.squaredNorm();
    if (heading_var > MAX_INITIAL_HEADING_SIGMA * MAX_INITIAL_HEADING_SIGMA) return std::nullopt;

    const double distance = d.norm();
    const Eigen::Vector2d along = d / distance;         // direction of travel
    const Eigen::Vector2d across(-along(1), along(0));  // its normal

    Gaussian g;
    g.x << z1(0), z1(1), distance / dt + end_minus_mean(0),
        wrapAngle(std::atan2(d(1), d(0)) + end_minus_mean(1)), 0.0;
    State var;
    var << gps_var, gps_var, 2 * gps_var / (dt * dt), heading_var, bias_var0;
    g.P = var.asDiagonal();
    // The later fix gives the position and also enters the speed and the
    // heading, so the errors of those estimates are correlated.
    g.P.block<2, 1>(X, V) = gps_var * along / dt;
    g.P.block<2, 1>(X, PSI) = gps_var * across / distance;
    g.P.block<1, 2>(V, X) = g.P.block<2, 1>(X, V).transpose();
    g.P.block<1, 2>(PSI, X) = g.P.block<2, 1>(X, PSI).transpose();
    // The heading part of end_minus_mean comes from the gyro, so the unknown
    // bias b has put b * dt / 2 into the heading.
    g.P(PSI, PSI) += bias_var0 * dt * dt / 4;
    g.P(PSI, B_YAW) = g.P(B_YAW, PSI) = -bias_var0 * dt / 2;
    return g;
}

// Weighted mean of states stored as columns. The heading is averaged on the
// circle, so that headings on either side of +-pi do not average to zero.
inline State weightedMean(const Eigen::MatrixXd& points, const Eigen::VectorXd& weights) {
    State mean = points * weights;
    const Eigen::ArrayXd psi = points.row(PSI).transpose().array();
    mean(PSI) = std::atan2((weights.array() * psi.sin()).sum(), (weights.array() * psi.cos()).sum());
    return mean;
}

// Each column of points minus x, with the heading difference wrapped.
inline Eigen::MatrixXd residuals(const Eigen::MatrixXd& points, const State& x) {
    Eigen::MatrixXd d = points.colwise() - x;
    for (int i = 0; i < d.cols(); ++i) d(PSI, i) = wrapAngle(d(PSI, i));
    return d;
}

}  // namespace ctrv
