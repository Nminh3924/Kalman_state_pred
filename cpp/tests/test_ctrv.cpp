#include <random>

#include <gtest/gtest.h>

#include "models/ctrv.h"

TEST(Ctrv, StepIntegratesSpeedAndInputsOverDt) {
    ctrv::State x;
    x << 1, 2, 10, 0, 0;

    const ctrv::State next = ctrv::f(x, Eigen::Vector2d(1.0, 0.2), 0.1);

    ctrv::State expected;
    expected << 2, 2, 10.1, 0.02, 0;
    EXPECT_TRUE(next.isApprox(expected)) << next.transpose();
}

TEST(Ctrv, MovesAlongItsHeading) {
    ctrv::State x;
    x << 0, 0, 10, M_PI / 2, 0;

    const ctrv::State next = ctrv::f(x, Eigen::Vector2d::Zero(), 0.1);

    EXPECT_NEAR(next(ctrv::X), 0.0, 1e-12);
    EXPECT_NEAR(next(ctrv::Y), 1.0, 1e-12);
}

TEST(Ctrv, SubtractsGyroBiasFromMeasuredYawRate) {
    ctrv::State x;
    x << 0, 0, 10, 0, 0.05;

    const ctrv::State next = ctrv::f(x, Eigen::Vector2d(0.0, 0.2), 0.1);

    EXPECT_NEAR(next(ctrv::PSI), 0.015, 1e-12);
    EXPECT_NEAR(next(ctrv::B_YAW), 0.05, 1e-12);
}

TEST(Ctrv, WrapAngleMapsIntoPlusMinusPi) {
    EXPECT_NEAR(ctrv::wrapAngle(0.5), 0.5, 1e-12);
    EXPECT_NEAR(ctrv::wrapAngle(M_PI + 0.1), -M_PI + 0.1, 1e-12);
    EXPECT_NEAR(ctrv::wrapAngle(-M_PI - 0.1), M_PI - 0.1, 1e-12);
    EXPECT_NEAR(ctrv::wrapAngle(6 * M_PI + 0.3), 0.3, 1e-12);
}

TEST(Ctrv, MeasurementIsThePosition) {
    ctrv::State x;
    x << 7, -3, 10, 1, 0.05;

    EXPECT_TRUE(ctrv::h(x).isApprox(Eigen::Vector2d(7, -3))) << ctrv::h(x).transpose();
}

TEST(Ctrv, WeightedMeanAveragesHeadingAcrossTheWrap) {
    Eigen::MatrixXd points(5, 2);
    points.col(0) << 1, 2, 3, M_PI - 0.1, 0.01;
    points.col(1) << 3, 4, 5, -M_PI + 0.1, 0.03;

    const ctrv::State mean = ctrv::weightedMean(points, Eigen::Vector2d(0.5, 0.5));

    // The headings sit 0.1 rad on either side of +-pi: their mean is pi, not 0.
    EXPECT_NEAR(std::abs(mean(ctrv::PSI)), M_PI, 1e-9);
    EXPECT_NEAR(mean(ctrv::X), 2, 1e-12);
    EXPECT_NEAR(mean(ctrv::Y), 3, 1e-12);
    EXPECT_NEAR(mean(ctrv::V), 4, 1e-12);
    EXPECT_NEAR(mean(ctrv::B_YAW), 0.02, 1e-12);
}

TEST(Ctrv, ResidualsWrapTheHeadingDifference) {
    Eigen::MatrixXd points(5, 1);
    points.col(0) << 5, 5, 5, M_PI - 0.1, 0.5;
    ctrv::State x;
    x << 1, 2, 3, -M_PI + 0.1, 0.1;

    const Eigen::MatrixXd d = ctrv::residuals(points, x);

    // Going the short way round, pi - 0.1 is 0.2 rad behind -pi + 0.1.
    ASSERT_EQ(d.cols(), 1);
    ctrv::State expected;
    expected << 4, 3, 2, -0.2, 0.4;
    EXPECT_TRUE(d.col(0).isApprox(expected)) << d.col(0).transpose();
}

TEST(Ctrv, AnalyticJacobiansMatchNumericalDifferentiation) {
    std::mt19937 rng(0);
    std::uniform_real_distribution<double> unit(-1.0, 1.0);
    const double eps = 1e-6;

    for (int trial = 0; trial < 100; ++trial) {
        ctrv::State x;
        x << 100 * unit(rng), 100 * unit(rng), 15 + 15 * unit(rng), M_PI * unit(rng), 0.1 * unit(rng);
        const Eigen::Vector2d u(3 * unit(rng), unit(rng));
        const double dt = 0.5 + 0.49 * unit(rng);

        Eigen::Matrix<double, 5, 5> F_numeric;
        for (int i = 0; i < 5; ++i) {
            ctrv::State d = ctrv::State::Zero();
            d(i) = eps;
            F_numeric.col(i) = (ctrv::f(x + d, u, dt) - ctrv::f(x - d, u, dt)) / (2 * eps);
        }
        Eigen::Matrix<double, 5, 2> G_numeric;
        for (int i = 0; i < 2; ++i) {
            Eigen::Vector2d d = Eigen::Vector2d::Zero();
            d(i) = eps;
            G_numeric.col(i) = (ctrv::f(x, u + d, dt) - ctrv::f(x, u - d, dt)) / (2 * eps);
        }

        EXPECT_LT((ctrv::F(x, dt) - F_numeric).cwiseAbs().maxCoeff(), 1e-5) << "trial " << trial;
        EXPECT_LT((ctrv::G(dt) - G_numeric).cwiseAbs().maxCoeff(), 1e-5) << "trial " << trial;
    }
}
