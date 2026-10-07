#pragma once

#include <Eigen/Dense>

#include "filters/gate.h"

// Linear Kalman filter. The motion and measurement models are passed in as
// matrices, so the same struct works for any linear model.
struct KalmanFilter {
    Eigen::VectorXd x;  // state estimate
    Eigen::MatrixXd P;  // covariance of the estimate

    void predict(const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q);

    // Corrects the estimate with measurement z. Returns the normalized
    // innovation squared (NIS): the squared Mahalanobis distance between z and
    // the measurement the filter expected. A measurement whose NIS exceeds
    // gate is treated as an outlier and leaves the estimate untouched.
    double update(const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R,
                  double gate = NO_GATE);
};
