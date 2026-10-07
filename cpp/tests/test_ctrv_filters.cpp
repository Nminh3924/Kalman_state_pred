#include <cmath>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "filters/ctrv_ekf.h"
#include "filters/ctrv_ukf.h"
#include "filters/kalman_filter.h"
#include "models/ctrv.h"
#include "run.h"
#include "synthetic.h"

namespace {

// Initialization without a correction from the IMU.
const Eigen::Vector2d NO_IMU = Eigen::Vector2d::Zero();

// Noise levels of the simulator: accel 0.1 m/s^2, gyro 0.01 rad/s, GPS 3 m.
const double ACCEL_VAR = 0.1 * 0.1;
const double GYRO_VAR = 0.01 * 0.01;
const double GPS_VAR = 3.0 * 3.0;
const double BIAS_VAR0 = 0.05 * 0.05;

CtrvEkf makeEkf(double bias_var0 = BIAS_VAR0) {
    return CtrvEkf(ACCEL_VAR, GYRO_VAR, GPS_VAR, bias_var0);
}

CtrvUkf makeUkf(double accel_var = ACCEL_VAR, double gyro_var = GYRO_VAR, double gps_var = GPS_VAR,
                double bias_var0 = BIAS_VAR0) {
    return CtrvUkf(accel_var, gyro_var, gps_var, bias_var0, 1.0, 2.0, 0.0);
}

double angleDiff(double a, double b) { return std::atan2(std::sin(a - b), std::cos(a - b)); }

// Behaviour every CTRV filter must show, checked for both the EKF and the UKF.
class CtrvFilter : public testing::TestWithParam<std::string> {
protected:
    std::unique_ptr<Filter> make() const {
        if (GetParam() == "ekf") return std::make_unique<CtrvEkf>(makeEkf());
        return std::make_unique<CtrvUkf>(makeUkf());
    }
};

}  // namespace

TEST_P(CtrvFilter, InitializesSpeedAndHeadingFromTwoFixes) {
    const std::unique_ptr<Filter> filter = make();

    ASSERT_TRUE(filter->initialize(Eigen::Vector2d(0, 0), Eigen::Vector2d(30, 40), 5.0, NO_IMU));

    // 50 m in 5 s is 10 m/s, heading atan2(40, 30).
    ctrv::State expected;
    expected << 30, 40, 10, 0.9272952180016122, 0;
    EXPECT_TRUE(filter->state().isApprox(expected)) << filter->state().transpose();
}

TEST_P(CtrvFilter, InitializationAppliesTheImuCorrectionToSpeedAndHeading) {
    const std::unique_ptr<Filter> filter = make();

    // The fixes say 10 m/s on average along atan2(40, 30); the IMU says the
    // vehicle ended 2 m/s faster and 0.1 rad further round than those averages.
    ASSERT_TRUE(filter->initialize(Eigen::Vector2d(0, 0), Eigen::Vector2d(30, 40), 5.0,
                                   Eigen::Vector2d(2.0, 0.1)));

    ctrv::State expected;
    expected << 30, 40, 12, 1.0272952180016122, 0;
    EXPECT_TRUE(filter->state().isApprox(expected)) << filter->state().transpose();
}

TEST_P(CtrvFilter, InitialCovarianceLinksPositionToSpeedAndHeading) {
    const std::unique_ptr<Filter> filter = make();

    ASSERT_TRUE(filter->initialize(Eigen::Vector2d(0, 0), Eigen::Vector2d(30, 40), 5.0, NO_IMU));

    // The later fix sets the position and also enters the speed and the heading,
    // so their errors are correlated. With GPS variance 9, travel direction
    // (0.6, 0.8), its normal (-0.8, 0.6), 50 m and 5 s between the fixes:
    //   cov(position, v)   = 9 * (0.6, 0.8) / 5
    //   cov(position, psi) = 9 * (-0.8, 0.6) / 50
    const Eigen::MatrixXd& P = filter->covariance();
    EXPECT_NEAR(P(ctrv::X, ctrv::V), 1.08, 1e-12);
    EXPECT_NEAR(P(ctrv::Y, ctrv::V), 1.44, 1e-12);
    EXPECT_NEAR(P(ctrv::X, ctrv::PSI), -0.144, 1e-12);
    EXPECT_NEAR(P(ctrv::Y, ctrv::PSI), 0.108, 1e-12);
    EXPECT_TRUE(P.isApprox(P.transpose())) << P;
}

TEST_P(CtrvFilter, InitialCovarianceAccountsForTheGyroBiasInTheImuCorrection) {
    const std::unique_ptr<Filter> filter = make();

    ASSERT_TRUE(filter->initialize(Eigen::Vector2d(0, 0), Eigen::Vector2d(30, 40), 5.0, NO_IMU));

    // The heading correction comes from the gyro, so an unknown bias b puts
    // b * dt / 2 into the initial heading. With bias variance 0.05^2 and dt = 5 s:
    //   var(psi) = 2 * 9 / 50^2 + 0.05^2 * 5^2 / 4,   cov(psi, bias) = -0.05^2 * 5 / 2
    const Eigen::MatrixXd& P = filter->covariance();
    EXPECT_NEAR(P(ctrv::PSI, ctrv::PSI), 0.0072 + 0.015625, 1e-12);
    EXPECT_NEAR(P(ctrv::PSI, ctrv::B_YAW), -0.00625, 1e-12);
    EXPECT_NEAR(P(ctrv::B_YAW, ctrv::PSI), -0.00625, 1e-12);
}

TEST_P(CtrvFilter, RefusesToInitializeWhenFixesAreTooCloseToGiveAHeading) {
    const std::unique_ptr<Filter> filter = make();

    // 5 m apart with 3 m GPS noise: the direction between the fixes is mostly noise.
    EXPECT_FALSE(filter->initialize(Eigen::Vector2d(0, 0), Eigen::Vector2d(3, 4), 0.5, NO_IMU));
}

TEST_P(CtrvFilter, ReportsNisOfAnAcceptedFix) {
    const std::unique_ptr<Filter> filter = make();
    ASSERT_TRUE(filter->initialize(Eigen::Vector2d(0, 0), Eigen::Vector2d(30, 40), 5.0, NO_IMU));

    const double nis = filter->update(Eigen::Vector2d(33, 40), NO_GATE);

    // Position variance 9 plus GPS variance 9 gives S = 18 I; the fix is 3 m
    // off, so NIS = 9 / 18 and the estimate moves half way.
    EXPECT_NEAR(nis, 0.5, 1e-9);
    EXPECT_NEAR(filter->state()(ctrv::X), 31.5, 1e-9);
}

TEST_P(CtrvFilter, IgnoresFixFarOutsideTheGate) {
    const std::unique_ptr<Filter> filter = make();
    ASSERT_TRUE(filter->initialize(Eigen::Vector2d(0, 0), Eigen::Vector2d(30, 40), 5.0, NO_IMU));
    const Eigen::VectorXd before = filter->state();

    // 100 m off: NIS = 100^2 / 18, far beyond the 99% gate of 9.21.
    const double nis = filter->update(Eigen::Vector2d(130, 40), 9.21);

    EXPECT_NEAR(nis, 10000.0 / 18.0, 1e-6);
    EXPECT_TRUE(filter->state().isApprox(before)) << filter->state().transpose();
}

TEST_P(CtrvFilter, HeadingStaysWrappedAndContinuousOverSeveralLaps) {
    const std::unique_ptr<Filter> filter = make();

    const std::vector<Estimate> est = run(*filter, circle(100, 0.0));  // 20 rad = 3.2 laps

    ASSERT_FALSE(est.empty());
    for (size_t i = 1; i < est.size(); ++i) {
        const double psi = est[i].x(ctrv::PSI);
        EXPECT_LE(std::abs(psi), M_PI) << "t = " << est[i].t;
        // Once settled. The heading still crosses +-pi at t = 47 s and 79 s.
        if (est[i].t > 30.0) {
            EXPECT_LT(std::abs(angleDiff(psi, 0.2 * est[i].t)), 0.05) << "t = " << est[i].t;
            EXPECT_LT(std::abs(angleDiff(psi, est[i - 1].x(ctrv::PSI))), 0.05) << "t = " << est[i].t;
        }
    }
}

TEST_P(CtrvFilter, RecoversForwardMotionWhenFirstFixesPointBackwards) {
    // GPS noise can make the first two fixes point against the direction of travel.
    std::vector<Measurement> measurements = straightLineSpeedingUp(20);
    int fix = 0;
    for (Measurement& m : measurements) {
        if (m.type != Measurement::Type::GPS) continue;
        if (fix == 0) m.z = Eigen::Vector2d(6.0, 0.0);
        if (fix == 1) m.z = Eigen::Vector2d(2.0, -3.0);
        ++fix;
    }
    const std::unique_ptr<Filter> filter = make();

    const std::vector<Estimate> est = run(*filter, measurements);

    // Truth at t = 20 s: x = 10*20 + 0.5*10^2 = 250 m, v = 10 + 10 = 20 m/s, heading 0.
    ASSERT_FALSE(est.empty());
    const Eigen::VectorXd& x = est.back().x;
    EXPECT_NEAR(x(ctrv::X), 250.0, 3.0);
    EXPECT_NEAR(x(ctrv::V), 20.0, 1.0);
    EXPECT_LT(std::abs(angleDiff(x(ctrv::PSI), 0.0)), 0.1);
}

TEST_P(CtrvFilter, EstimatesConstantGyroBias) {
    const std::unique_ptr<Filter> filter = make();

    const std::vector<Estimate> est = run(*filter, circle(100, 0.01));

    ASSERT_FALSE(est.empty());
    EXPECT_NEAR(est.back().x(ctrv::B_YAW), 0.01, 0.002);
}

INSTANTIATE_TEST_SUITE_P(EkfAndUkf, CtrvFilter, testing::Values("ekf", "ukf"),
                         [](const testing::TestParamInfo<std::string>& info) { return info.param; });

TEST(CtrvEkf, LeavesBiasAtZeroWhenItsInitialVarianceIsZero) {
    CtrvEkf ekf = makeEkf(0.0);

    const std::vector<Estimate> est = run(ekf, circle(100, 0.01));

    ASSERT_FALSE(est.empty());
    EXPECT_EQ(est.back().x(ctrv::B_YAW), 0.0);
}

TEST(CtrvUkf, PredictMatchesLinearizedPropagationWhenUncertaintyIsSmall) {
    // With a tight prior the model is almost linear across the sigma points, so
    // the UKF must agree with propagating through the analytic Jacobians.
    const double accel_var = 0.04, gyro_var = 0.01;
    CtrvUkf ukf = makeUkf(accel_var, gyro_var, 1e-4, 1e-8);  // tight in position and in bias
    ASSERT_TRUE(ukf.initialize(Eigen::Vector2d(0, 0), Eigen::Vector2d(30, 40), 5.0, NO_IMU));
    const ctrv::State x0 = ukf.state();
    const ctrv::Covariance P0 = ukf.covariance();
    const Eigen::Vector2d u(1.0, 0.3);
    const double dt = 0.5;

    ukf.predict(u, dt);

    const ctrv::Covariance F = ctrv::F(x0, dt);
    const Eigen::Matrix<double, 5, 2> G = ctrv::G(dt);
    const Eigen::Matrix2d Q_imu = Eigen::Vector2d(accel_var, gyro_var).asDiagonal();
    const ctrv::State x_expected = ctrv::f(x0, u, dt);
    const ctrv::Covariance P_expected = F * P0 * F.transpose() + G * Q_imu * G.transpose();
    EXPECT_LT((ukf.state() - x_expected).norm(), 1e-6) << ukf.state().transpose();
    EXPECT_LT((ukf.covariance() - P_expected).norm(), 1e-6 * P_expected.norm()) << ukf.covariance();
}

TEST(CtrvUkf, UpdateMatchesKalmanUpdateForTheLinearGpsMeasurement) {
    // GPS is linear in the state, so the sigma point update must give exactly
    // what the linear Kalman update gives from the same prior.
    CtrvUkf ukf = makeUkf();
    ASSERT_TRUE(ukf.initialize(Eigen::Vector2d(0, 0), Eigen::Vector2d(30, 40), 5.0, NO_IMU));
    for (int i = 0; i < 3; ++i) ukf.predict(Eigen::Vector2d(1.0, 0.3), 1.0);  // build correlations
    KalmanFilter kf{ukf.state(), ukf.covariance()};
    const Eigen::Vector2d z = ukf.state().head<2>() + Eigen::Vector2d(4.0, -2.5);

    const double nis_ukf = ukf.update(z, NO_GATE);
    const double nis_kf = kf.update(z, ctrv::H(), GPS_VAR * Eigen::Matrix2d::Identity());

    EXPECT_NEAR(nis_ukf, nis_kf, 1e-9);
    EXPECT_LT((ukf.state() - kf.x).norm(), 1e-9) << ukf.state().transpose();
    EXPECT_LT((ukf.covariance() - kf.P).norm(), 1e-9 * kf.P.norm()) << ukf.covariance();
}
