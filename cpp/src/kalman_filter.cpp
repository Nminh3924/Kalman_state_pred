#include "filters/kalman_filter.h"

void KalmanFilter::predict(const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q) {
    x = F * x;
    P = F * P * F.transpose() + Q;
}

double KalmanFilter::update(const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& R,
                            double gate) {
    const Eigen::VectorXd y = z - H * x;                    // innovation
    const Eigen::MatrixXd S = H * P * H.transpose() + R;    // innovation covariance
    const Eigen::MatrixXd S_inv = S.inverse();
    const double nis = y.dot(S_inv * y);
    if (nis > gate) return nis;

    const Eigen::MatrixXd K = P * H.transpose() * S_inv;    // Kalman gain
    x += K * y;
    P = (Eigen::MatrixXd::Identity(x.size(), x.size()) - K * H) * P;
    return nis;
}
