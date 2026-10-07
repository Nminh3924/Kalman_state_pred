#include <gtest/gtest.h>

#include "filters/kalman_filter.h"
#include "models/constant_velocity.h"

using Eigen::Matrix2d;
using Eigen::Matrix4d;
using Eigen::Vector2d;
using Eigen::Vector4d;

TEST(KalmanFilter, PredictMovesPositionByVelocityTimesDt) {
    KalmanFilter kf{Vector4d(1, 2, 3, -4), Matrix4d::Zero()};

    kf.predict(cv_model::F(0.5), Matrix4d::Zero());

    EXPECT_TRUE(kf.x.isApprox(Vector4d(2.5, 0, 3, -4))) << kf.x.transpose();
    EXPECT_TRUE(kf.P.isZero()) << kf.P;
}

TEST(KalmanFilter, PredictPropagatesCovarianceAndAddsProcessNoise) {
    KalmanFilter kf{Vector4d::Zero(), Matrix4d::Identity()};

    kf.predict(cv_model::F(1.0), 0.5 * Matrix4d::Identity());

    // Per axis: F P F' = [[1,1],[0,1]] [[1,0],[1,1]] = [[2,1],[1,1]], then 0.5 on the diagonal.
    Matrix4d expected;
    expected << 2.5, 0, 1, 0,
                0, 2.5, 0, 1,
                1, 0, 1.5, 0,
                0, 1, 0, 1.5;
    EXPECT_TRUE(kf.P.isApprox(expected)) << kf.P;
}

TEST(KalmanFilter, UpdateWithEqualVariancesLandsHalfway) {
    KalmanFilter kf{Vector4d::Zero(), Matrix4d::Identity()};

    kf.update(Vector2d(2, 4), cv_model::H(), Matrix2d::Identity());

    // Prior and measurement are equally uncertain, so the gain is 1/2.
    EXPECT_TRUE(kf.x.isApprox(Vector4d(1, 2, 0, 0))) << kf.x.transpose();
    Matrix4d expected = Matrix4d::Identity();
    expected(0, 0) = 0.5;
    expected(1, 1) = 0.5;
    EXPECT_TRUE(kf.P.isApprox(expected)) << kf.P;
}

TEST(KalmanFilter, UpdateReturnsNormalizedInnovationSquared) {
    KalmanFilter kf{Vector4d::Zero(), Matrix4d::Identity()};

    const double nis = kf.update(Vector2d(2, 4), cv_model::H(), Matrix2d::Identity());

    // Innovation (2, 4) with S = P + R = 2 I: (4 + 16) / 2.
    EXPECT_NEAR(nis, 10.0, 1e-12);
}

TEST(KalmanFilter, UpdateIgnoresMeasurementBeyondTheGate) {
    KalmanFilter kf{Vector4d::Zero(), Matrix4d::Identity()};

    const double nis = kf.update(Vector2d(2, 4), cv_model::H(), Matrix2d::Identity(), 9.0);

    // NIS is 10, above the gate of 9: nothing may change.
    EXPECT_NEAR(nis, 10.0, 1e-12);
    EXPECT_TRUE(kf.x.isZero()) << kf.x.transpose();
    EXPECT_TRUE(kf.P.isIdentity()) << kf.P;
}

TEST(KalmanFilter, UpdateAppliesMeasurementWithinTheGate) {
    KalmanFilter kf{Vector4d::Zero(), Matrix4d::Identity()};

    kf.update(Vector2d(2, 4), cv_model::H(), Matrix2d::Identity(), 11.0);

    EXPECT_TRUE(kf.x.isApprox(Vector4d(1, 2, 0, 0))) << kf.x.transpose();
}

TEST(KalmanFilter, UpdateWithTinyRSnapsToMeasurement) {
    KalmanFilter kf{Vector4d::Zero(), Matrix4d::Identity()};

    kf.update(Vector2d(5, -7), cv_model::H(), 1e-12 * Matrix2d::Identity());

    EXPECT_NEAR(kf.x(0), 5, 1e-9);
    EXPECT_NEAR(kf.x(1), -7, 1e-9);
}

TEST(KalmanFilter, CovarianceStaysSymmetricPositiveDefiniteOver1000Steps) {
    KalmanFilter kf{Vector4d::Zero(), 100 * Matrix4d::Identity()};
    const Matrix2d R = 9 * Matrix2d::Identity();

    for (int i = 0; i < 1000; ++i) {
        kf.predict(cv_model::F(1.0), cv_model::Q(1.0, 1.0));
        kf.update(Vector2d(i, -i), cv_model::H(), R);
    }

    // The filter has really run: a posterior position variance is below R.
    ASSERT_LT(kf.P(0, 0), 9.0);
    EXPECT_LT((kf.P - kf.P.transpose()).norm(), 1e-9 * kf.P.norm());
    EXPECT_EQ(Eigen::LLT<Eigen::MatrixXd>(kf.P).info(), Eigen::Success);
}
