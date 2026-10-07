#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "io.h"

// Noise-free measurements with a known analytic ground truth, for driving
// filters in tests. IMU at 100 Hz, GPS at 1 Hz, IMU first at equal timestamps
// (the order loadMeasurements returns).

// Straight line along +x at 10 m/s: position (10 t, 0).
inline std::vector<Measurement> straightLine(int seconds) {
    std::vector<Measurement> m;
    for (int k = 0; k <= seconds * 100; ++k) {
        const double t = k / 100.0;
        m.push_back({t, Measurement::Type::IMU, Eigen::Vector2d(0.0, 0.0)});
        if (k % 100 == 0) m.push_back({t, Measurement::Type::GPS, Eigen::Vector2d(10.0 * t, 0.0)});
    }
    return m;
}

// Straight line along +x: 10 m/s until t = 10 s, then speeding up at 1 m/s^2.
inline std::vector<Measurement> straightLineSpeedingUp(int seconds) {
    std::vector<Measurement> m;
    for (int k = 0; k <= seconds * 100; ++k) {
        const double t = k / 100.0;
        const double s = std::max(0.0, t - 10.0);  // time spent accelerating
        m.push_back({t, Measurement::Type::IMU, Eigen::Vector2d(t >= 10.0 ? 1.0 : 0.0, 0.0)});
        if (k % 100 == 0) {
            m.push_back({t, Measurement::Type::GPS, Eigen::Vector2d(10.0 * t + 0.5 * s * s, 0.0)});
        }
    }
    return m;
}

// Circle of radius 50 m at 10 m/s and 0.2 rad/s: heading 0.2 t, position
// (50 sin(0.2 t), 50 (1 - cos(0.2 t))). The gyro reads 0.2 + gyro_bias.
inline std::vector<Measurement> circle(int seconds, double gyro_bias) {
    std::vector<Measurement> m;
    for (int k = 0; k <= seconds * 100; ++k) {
        const double t = k / 100.0;
        m.push_back({t, Measurement::Type::IMU, Eigen::Vector2d(0.0, 0.2 + gyro_bias)});
        if (k % 100 == 0) {
            m.push_back({t, Measurement::Type::GPS,
                         Eigen::Vector2d(50.0 * std::sin(0.2 * t), 50.0 * (1.0 - std::cos(0.2 * t)))});
        }
    }
    return m;
}
