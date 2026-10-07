#include <cmath>

#include <gtest/gtest.h>

#include "filters/unscented.h"

TEST(SigmaPoints, PlacesOnePointOnEachSideAlongEveryAxis) {
    Eigen::Matrix2d P;
    P << 4, 0,
         0, 9;

    const SigmaPoints s = sigmaPoints(Eigen::Vector2d(1, 2), P, 1.0, 2.0, 1.0);

    // n = 2 and lambda = alpha^2 (n + kappa) - n = 1, so the offsets are the
    // columns of sqrt(3 P) = diag(sqrt(12), sqrt(27)).
    ASSERT_EQ(s.points.cols(), 5);
    Eigen::Matrix<double, 2, 5> expected;
    expected << 1, 1 + std::sqrt(12.0), 1, 1 - std::sqrt(12.0), 1,
                2, 2, 2 + std::sqrt(27.0), 2, 2 - std::sqrt(27.0);
    EXPECT_TRUE(s.points.isApprox(expected)) << s.points;
}

TEST(SigmaPoints, WeightsFollowTheScaledUnscentedTransform) {
    const SigmaPoints s = sigmaPoints(Eigen::Vector2d(1, 2), Eigen::Matrix2d::Identity(), 1.0, 2.0, 1.0);

    // lambda = 1: the centre point weighs lambda / (n + lambda) = 1/3 in the mean
    // and 1/3 + (1 - alpha^2 + beta) = 7/3 in the covariance; the others 1 / (2 * 3).
    ASSERT_EQ(s.mean_weights.size(), 5);
    ASSERT_EQ(s.cov_weights.size(), 5);
    EXPECT_NEAR(s.mean_weights(0), 1.0 / 3, 1e-12);
    EXPECT_NEAR(s.cov_weights(0), 7.0 / 3, 1e-12);
    for (int i = 1; i < 5; ++i) {
        EXPECT_NEAR(s.mean_weights(i), 1.0 / 6, 1e-12);
        EXPECT_NEAR(s.cov_weights(i), 1.0 / 6, 1e-12);
    }
}

TEST(SigmaPoints, ReproduceMeanAndCovarianceOfACorrelatedGaussian) {
    const Eigen::Vector3d x(1, -2, 0.5);
    Eigen::Matrix3d P;
    P << 4, 1, 0.5,
         1, 9, -2,
         0.5, -2, 3;

    const SigmaPoints s = sigmaPoints(x, P, 0.5, 2.0, 1.0);

    ASSERT_EQ(s.points.cols(), 7);
    const Eigen::Vector3d mean = s.points * s.mean_weights;
    const Eigen::MatrixXd d = s.points.colwise() - mean;
    const Eigen::Matrix3d cov = d * s.cov_weights.asDiagonal() * d.transpose();
    EXPECT_TRUE(mean.isApprox(x)) << mean.transpose();
    EXPECT_TRUE(cov.isApprox(P)) << cov;
}
