#pragma once

#include <filesystem>
#include <vector>

#include <Eigen/Dense>

struct Measurement {
    enum class Type { GPS, IMU };

    double t;
    Type type;
    Eigen::VectorXd z;  // GPS: [x, y]   IMU: [a, yaw_rate]
};

// Reads gps.csv and imu.csv from dir and returns all measurements sorted by time.
// At equal timestamps the IMU sample comes before the GPS fix.
std::vector<Measurement> loadMeasurements(const std::filesystem::path& dir);
