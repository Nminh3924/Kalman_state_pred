#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "filters/gate.h"

// Common interface, so that one loop can drive every filter.
class Filter {
public:
    virtual ~Filter() = default;

    // Tries to set the first estimate from two GPS fixes taken dt seconds apart.
    // The fixes give the mean velocity in between. end_minus_mean = [speed,
    // heading] is how far the values at the later fix are above those means
    // according to the IMU: it matters when the vehicle accelerates or turns.
    // Returns false if the fixes do not carry enough information; the caller
    // then tries another pair.
    virtual bool initialize(const Eigen::Vector2d& z0, const Eigen::Vector2d& z1, double dt,
                            const Eigen::Vector2d& end_minus_mean) = 0;
    // Advances the estimate by dt using the IMU input u = [a, yaw_rate].
    virtual void predict(const Eigen::Vector2d& u, double dt) = 0;
    // Corrects the estimate with a GPS position fix z = [x, y] and returns its
    // NIS. A fix whose NIS exceeds gate is ignored as an outlier.
    virtual double update(const Eigen::Vector2d& z, double gate) = 0;

    // A copy with the same estimate and settings.
    virtual std::unique_ptr<Filter> clone() const = 0;
    // Replaces the noise assumed on one sample of the IMU input [a, yaw_rate].
    // A filter that does not use the IMU ignores this.
    virtual void setInputNoise(double accel_var, double gyro_var) = 0;

    virtual const Eigen::VectorXd& state() const = 0;
    virtual const Eigen::MatrixXd& covariance() const = 0;
    virtual std::vector<std::string> stateNames() const = 0;
};
