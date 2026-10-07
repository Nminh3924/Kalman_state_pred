#include "filters/unscented.h"

SigmaPoints sigmaPoints(const Eigen::VectorXd& x, const Eigen::MatrixXd& P,
                        double alpha, double beta, double kappa) {
    const int n = x.size();
    const double lambda = alpha * alpha * (n + kappa) - n;

    // Matrix square root: L L' = (n + lambda) P.
    const Eigen::MatrixXd L = Eigen::LLT<Eigen::MatrixXd>((n + lambda) * P).matrixL();

    SigmaPoints s;
    s.points = x.replicate(1, 2 * n + 1);
    s.points.block(0, 1, n, n) += L;
    s.points.block(0, n + 1, n, n) -= L;

    s.mean_weights = Eigen::VectorXd::Constant(2 * n + 1, 0.5 / (n + lambda));
    s.cov_weights = s.mean_weights;
    s.mean_weights(0) = lambda / (n + lambda);
    s.cov_weights(0) = s.mean_weights(0) + 1 - alpha * alpha + beta;
    return s;
}
