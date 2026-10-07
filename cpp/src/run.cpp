#include "run.h"

#include <limits>

// Gating assumes the estimate is right and the fix is wrong. Two rules cover
// the opposite case:
//  - A freshly initialized filter is on probation until it accepts a fix. It
//    reports nothing, and one rejected fix restarts it. This keeps an outlier
//    among the initializing fixes from ever reaching the output.
//  - A confirmed filter that rejects this many fixes in a row is taken to be
//    lost rather than unlucky, and is restarted too.
const int MAX_CONSECUTIVE_REJECTIONS = 3;

// The two fixes that initialize a filter should be far apart, for a good
// heading, but not so far back that the IMU errors bridging them add up.
const double INITIALIZATION_LOOKBACK = 5.0;  // s

namespace {

// Running integrals of the IMU input u = [a, yaw_rate], kept while the filter
// waits to be initialized. They tell how speed and heading changed between two
// fixes, which the fixes themselves cannot.
struct ImuIntegral {
    double t = 0.0;                                   // integrated up to here
    Eigen::Vector2d u = Eigen::Vector2d::Zero();      // latest sample, held until the next one
    Eigen::Vector2d once = Eigen::Vector2d::Zero();   // integral of u: change of [speed, heading]
    Eigen::Vector2d twice = Eigen::Vector2d::Zero();  // integral of `once`

    void advance(double to) {
        const double h = to - t;
        twice += once * h + 0.5 * u * h * h;
        once += u * h;
        t = to;
    }
};

// A GPS fix together with the IMU integrals at its time.
struct Fix {
    double t;
    Eigen::Vector2d z;
    Eigen::Vector2d once;
    Eigen::Vector2d twice;
};

}  // namespace

std::vector<Estimate> run(Filter& filter, const std::vector<Measurement>& measurements, double gate,
                          const Observer& observer) {
    std::vector<Estimate> estimates;
    ImuIntegral imu;
    std::vector<Fix> fixes;  // fixes seen while waiting to be initialized
    bool initialized = false;
    double t = 0.0;  // time the filter state refers to
    const double no_fix = std::numeric_limits<double>::quiet_NaN();
    double nis = no_fix;  // NIS of the fix applied at time t, if any
    int rejections = 0;   // fixes rejected in a row
    const bool gated = gate < NO_GATE;
    bool confirmed = false;  // has accepted a fix since it was initialized

    const auto report = [&] {
        estimates.push_back({t, filter.state(), filter.covariance(), nis});
        if (observer) observer(t, filter, imu.u);
    };

    for (const Measurement& m : measurements) {
        const bool is_imu = m.type == Measurement::Type::IMU;

        if (!initialized) {
            imu.advance(m.t);
            if (is_imu) {
                imu.u = m.z;
                continue;
            }
            // Pair this fix with the oldest one inside the lookback window. If
            // the filter turns that pair down (a slow vehicle has not moved far
            // enough), try ever older ones.
            const Fix last{m.t, m.z, imu.once, imu.twice};
            int i = static_cast<int>(fixes.size()) - 1;
            while (i > 0 && last.t - fixes[i - 1].t <= INITIALIZATION_LOOKBACK) --i;
            for (; i >= 0 && !initialized; --i) {
                const Fix& earlier = fixes[i];
                const double dt = last.t - earlier.t;
                // Value at the later fix minus the mean in between, for speed and heading.
                const Eigen::Vector2d end_minus_mean = last.once - (last.twice - earlier.twice) / dt;
                initialized = filter.initialize(earlier.z, last.z, dt, end_minus_mean);
            }
            fixes.push_back(last);
            if (initialized) {
                t = m.t;
                nis = no_fix;
                rejections = 0;
                confirmed = !gated;  // without a gate nothing can contradict the start
            }
            continue;
        }

        // Time moves on: record the finished estimate, then predict up to m.t
        // with the input that was valid during that interval.
        if (m.t > t) {
            if (confirmed) report();
            filter.predict(imu.u, m.t - t);
            t = m.t;
            nis = no_fix;
        }

        if (is_imu) {
            imu.u = m.z;
            continue;
        }
        nis = filter.update(m.z, gate);
        if (nis <= gate) {
            confirmed = true;
            rejections = 0;
        } else if (!confirmed || ++rejections == MAX_CONSECUTIVE_REJECTIONS) {
            // Start over, with this fix as the first of the new start.
            imu.t = m.t;
            imu.once.setZero();
            imu.twice.setZero();
            fixes = {Fix{m.t, m.z, imu.once, imu.twice}};
            initialized = false;
        }
    }

    if (initialized && confirmed) report();
    return estimates;
}
