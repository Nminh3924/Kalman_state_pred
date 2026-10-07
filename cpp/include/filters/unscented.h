#pragma once

#include <Eigen/Dense>

// Sigma points of the scaled unscented transform, with their weights.
struct SigmaPoints {
    Eigen::MatrixXd points;        // one point per column, n x (2n + 1)
    Eigen::VectorXd mean_weights;  // 2n + 1
    Eigen::VectorXd cov_weights;   // 2n + 1
};

// Deterministic sample of the Gaussian (x, P): the mean itself, plus one point
// on each side of it along every column of the Cholesky factor of P.
// alpha scales the spread, beta weights the centre point in the covariance
// (2 is optimal for a Gaussian), kappa is a secondary scaling term.
SigmaPoints sigmaPoints(const Eigen::VectorXd& x, const Eigen::MatrixXd& P,
                        double alpha, double beta, double kappa);
