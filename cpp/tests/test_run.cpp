#include <cmath>

#include <gtest/gtest.h>

#include "filters/ctrv_ekf.h"
#include "filters/cv_kalman_filter.h"
#include "models/ctrv.h"
#include "run.h"
#include "synthetic.h"

namespace {

// Initialization without a correction from the IMU.
const Eigen::Vector2d NO_IMU = Eigen::Vector2d::Zero();

// The estimate at time t.
const Estimate& at(const std::vector<Estimate>& estimates, double t) {
    for (const Estimate& e : estimates) {
        if (std::abs(e.t - t) < 1e-9) return e;
    }
    ADD_FAILURE() << "no estimate at t = " << t;
    return estimates.front();
}

}  // namespace

TEST(CvKalmanFilter, InitializesVelocityFromFirstTwoFixes) {
    CvKalmanFilter filter(3.0, 9.0);

    ASSERT_TRUE(filter.initialize(Eigen::Vector2d(0, 0), Eigen::Vector2d(3, 4), 0.5, NO_IMU));

    EXPECT_TRUE(filter.state().isApprox(Eigen::Vector4d(3, 4, 6, 8))) << filter.state().transpose();
}

TEST(CvKalmanFilter, InitialCovarianceLinksPositionToVelocity) {
    CvKalmanFilter filter(3.0, 9.0);

    ASSERT_TRUE(filter.initialize(Eigen::Vector2d(0, 0), Eigen::Vector2d(3, 4), 0.5, NO_IMU));

    // The later fix sets the position and also enters the velocity:
    // cov(position, velocity) = 9 / 0.5 per axis, velocity variance 2 * 9 / 0.5^2.
    Eigen::Matrix4d expected;
    expected << 9, 0, 18, 0,
                0, 9, 0, 18,
                18, 0, 72, 0,
                0, 18, 0, 72;
    EXPECT_TRUE(filter.covariance().isApprox(expected)) << filter.covariance();
}

TEST(Run, ReturnsOneEstimatePerTimestampStartingAtSecondFix) {
    CvKalmanFilter filter(3.0, 9.0);

    const std::vector<Estimate> est = run(filter, straightLine(5));

    ASSERT_EQ(est.size(), 401u);  // t = 1.00, 1.01, ..., 5.00
    EXPECT_DOUBLE_EQ(est.front().t, 1.0);
    EXPECT_DOUBLE_EQ(est.back().t, 5.0);
    for (size_t i = 1; i < est.size(); ++i) EXPECT_GT(est[i].t, est[i - 1].t);
}

TEST(Run, CvKalmanFilterStaysOnNoiseFreeStraightLine) {
    CvKalmanFilter filter(3.0, 9.0);

    const std::vector<Estimate> est = run(filter, straightLine(5));

    ASSERT_FALSE(est.empty());
    for (const Estimate& e : est) {
        EXPECT_NEAR(e.x(0), 10.0 * e.t, 1e-6) << "t = " << e.t;
        EXPECT_NEAR(e.x(1), 0.0, 1e-6) << "t = " << e.t;
    }
}

TEST(Run, RecordsNisAtGpsUpdatesOnly) {
    CvKalmanFilter filter(3.0, 9.0);

    const std::vector<Estimate> est = run(filter, straightLine(5));

    // The fix at t = 1 s initializes the filter; those at 2, 3, 4, 5 s update it.
    ASSERT_EQ(est.size(), 401u);
    int updates = 0;
    for (size_t i = 0; i < est.size(); ++i) {
        if (i > 0 && i % 100 == 0) {
            EXPECT_NEAR(est[i].nis, 0.0, 1e-9) << "t = " << est[i].t;  // noise-free: no innovation
            ++updates;
        } else {
            EXPECT_TRUE(std::isnan(est[i].nis)) << "t = " << est[i].t;
        }
    }
    EXPECT_EQ(updates, 4);
}

TEST(Run, GateKeepsAnOutlierFixFromPullingTheEstimate) {
    std::vector<Measurement> measurements = straightLine(10);
    for (Measurement& m : measurements) {
        if (m.type == Measurement::Type::GPS && m.t == 5.0) m.z(1) = 30.0;  // 30 m sideways
    }
    CvKalmanFilter gated(3.0, 9.0);
    CvKalmanFilter open(3.0, 9.0);

    const std::vector<Estimate> with_gate = run(gated, measurements, 9.21);
    const std::vector<Estimate> without_gate = run(open, measurements);

    // Right after the bad fix at t = 5 s:
    ASSERT_FALSE(with_gate.empty());
    ASSERT_FALSE(without_gate.empty());
    EXPECT_NEAR(at(with_gate, 5.0).x(1), 0.0, 1e-6);
    EXPECT_GT(at(with_gate, 5.0).nis, 9.21);
    EXPECT_GT(at(without_gate, 5.0).x(1), 5.0);
    // One rejected fix does not interrupt the output.
    EXPECT_DOUBLE_EQ(with_gate.back().t, 10.0);
    EXPECT_EQ(with_gate.size(), 801u);  // t = 2.00 ... 10.00
}

TEST(Run, WithAGateReportsNothingUntilTheFilterHasAcceptedAFix) {
    CvKalmanFilter filter(3.0, 9.0);

    const std::vector<Estimate> est = run(filter, straightLine(5), 9.21);

    // Initialized at t = 1 s, confirmed by the fix at t = 2 s.
    ASSERT_FALSE(est.empty());
    EXPECT_DOUBLE_EQ(est.front().t, 2.0);
}

TEST(Run, NeverReportsAnEstimateBuiltOnAnOutlierFix) {
    // An outlier among the fixes used for initialization puts the filter 30 m
    // off while it believes it is right.
    std::vector<Measurement> measurements = straightLine(20);
    for (Measurement& m : measurements) {
        if (m.type == Measurement::Type::GPS && m.t == 1.0) m.z(1) = 30.0;
    }
    CvKalmanFilter filter(3.0, 9.0);

    const std::vector<Estimate> est = run(filter, measurements, 9.21);

    // The fix at 2 s contradicts that start, so the filter starts again from
    // the fixes at 2 s and 3 s and is confirmed by the one at 4 s.
    ASSERT_FALSE(est.empty());
    EXPECT_DOUBLE_EQ(est.front().t, 4.0);
    EXPECT_DOUBLE_EQ(est.back().t, 20.0);
    for (const Estimate& e : est) {
        EXPECT_NEAR(e.x(0), 10.0 * e.t, 1e-6) << "t = " << e.t;
        EXPECT_NEAR(e.x(1), 0.0, 1e-6) << "t = " << e.t;
    }
}

TEST(Run, RestartsAConfirmedFilterThatRejectsThreeFixesInARow) {
    // From t = 10 s on the fixes sit 30 m to the side: the filter, not the
    // GPS, is what no longer matches.
    std::vector<Measurement> measurements = straightLine(20);
    for (Measurement& m : measurements) {
        if (m.type == Measurement::Type::GPS && m.t >= 10.0) m.z(1) = 30.0;
    }
    // A small process noise, so that the covariance does not grow enough in
    // two seconds to let the offset fixes back in by itself.
    CvKalmanFilter filter(0.1, 9.0);

    const std::vector<Estimate> est = run(filter, measurements, 9.21);

    // Rejected at 10, 11 and 12 s; started again from the fixes at 12 s and
    // 13 s; confirmed, and reporting again, at 14 s.
    ASSERT_FALSE(est.empty());
    EXPECT_NEAR(at(est, 11.0).x(1), 0.0, 1e-6);
    int reported_while_restarting = 0;
    for (const Estimate& e : est) {
        if (e.t > 12.0 && e.t < 14.0) ++reported_while_restarting;
    }
    EXPECT_EQ(reported_while_restarting, 0);
    EXPECT_NEAR(at(est, 14.0).x(1), 30.0, 1e-6);
    EXPECT_NEAR(est.back().x(0), 200.0, 1e-6);
    EXPECT_NEAR(est.back().x(1), 30.0, 1e-6);
}

TEST(Run, InitializesWithTheSpeedAtTheLaterFixWhenTheVehicleAccelerates) {
    // From 2 m/s at 2 m/s^2 along +x: x = 2 t + t^2, so the fixes are at
    // 0, 3, 8, 15, 24, ... m. The first pair far enough apart for a heading is
    // (0 s, 4 s): mean speed 6 m/s, but 10 m/s at the later fix.
    std::vector<Measurement> measurements;
    for (int k = 0; k <= 600; ++k) {
        const double t = k / 100.0;
        measurements.push_back({t, Measurement::Type::IMU, Eigen::Vector2d(2.0, 0.0)});
        if (k % 100 == 0) {
            measurements.push_back({t, Measurement::Type::GPS, Eigen::Vector2d(2.0 * t + t * t, 0.0)});
        }
    }
    CtrvEkf ekf(0.1 * 0.1, 0.01 * 0.01, 3.0 * 3.0, 0.05 * 0.05);

    const std::vector<Estimate> est = run(ekf, measurements);

    ASSERT_FALSE(est.empty());
    EXPECT_DOUBLE_EQ(est.front().t, 4.0);
    EXPECT_NEAR(est.front().x(ctrv::V), 10.0, 1e-9);
    EXPECT_NEAR(est.front().x(ctrv::PSI), 0.0, 1e-9);
}

TEST(Run, PairsAFixWithTheOldestFixOfTheLastFiveSeconds) {
    // Standing still for 7 s, then 30 m away at 8 s. A long baseline gives the
    // best heading, but going back too far lets IMU errors add up: the partner
    // is the fix at 3 s.
    std::vector<Measurement> measurements;
    for (int k = 0; k <= 800; ++k) {
        const double t = k / 100.0;
        measurements.push_back({t, Measurement::Type::IMU, Eigen::Vector2d(0.0, 0.0)});
        if (k % 100 == 0) {
            measurements.push_back({t, Measurement::Type::GPS, Eigen::Vector2d(k == 800 ? 30.0 : 0.0, 0.0)});
        }
    }
    CtrvEkf ekf(0.1 * 0.1, 0.01 * 0.01, 3.0 * 3.0, 0.05 * 0.05);

    const std::vector<Estimate> est = run(ekf, measurements);

    // 30 m in 5 s; speed variance 2 * 9 / 5^2.
    ASSERT_EQ(est.size(), 1u);
    EXPECT_NEAR(est.front().x(ctrv::V), 6.0, 1e-9);
    EXPECT_NEAR(est.front().P(ctrv::V, ctrv::V), 0.72, 1e-9);
}

TEST(Run, GoesFurtherBackWhenTheVehicleIsTooSlowForFiveSeconds) {
    // At 3 m/s the vehicle covers 15 m in 5 s, short of the 21.2 m a heading
    // needs. The first pair far enough apart is (0 s, 8 s), 24 m.
    std::vector<Measurement> measurements;
    for (int k = 0; k <= 900; ++k) {
        const double t = k / 100.0;
        measurements.push_back({t, Measurement::Type::IMU, Eigen::Vector2d(0.0, 0.0)});
        if (k % 100 == 0) measurements.push_back({t, Measurement::Type::GPS, Eigen::Vector2d(3.0 * t, 0.0)});
    }
    CtrvEkf ekf(0.1 * 0.1, 0.01 * 0.01, 3.0 * 3.0, 0.05 * 0.05);

    const std::vector<Estimate> est = run(ekf, measurements);

    ASSERT_FALSE(est.empty());
    EXPECT_DOUBLE_EQ(est.front().t, 8.0);
    EXPECT_NEAR(est.front().x(ctrv::V), 3.0, 1e-9);
    EXPECT_NEAR(est.front().P(ctrv::V, ctrv::V), 2 * 9.0 / 64.0, 1e-9);
}

TEST(Run, TellsTheObserverAboutEveryReportedEstimate) {
    // The IMU reports a yaw rate equal to the time, so each call can be checked.
    std::vector<Measurement> measurements = straightLine(3);
    for (Measurement& m : measurements) {
        if (m.type == Measurement::Type::IMU) m.z(1) = m.t;
    }
    CvKalmanFilter filter(3.0, 9.0);
    std::vector<double> times;
    std::vector<double> yaw_rates;

    const std::vector<Estimate> est = run(filter, measurements, NO_GATE,
        [&](double t, const Filter&, const Eigen::Vector2d& u) {
            times.push_back(t);
            yaw_rates.push_back(u(1));
        });

    ASSERT_EQ(times.size(), est.size());
    for (size_t i = 0; i < est.size(); ++i) {
        EXPECT_DOUBLE_EQ(times[i], est[i].t);
        EXPECT_DOUBLE_EQ(yaw_rates[i], est[i].t) << "the IMU sample of that time";
    }
}
