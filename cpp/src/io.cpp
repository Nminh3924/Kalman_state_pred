#include "io.h"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

// Appends one measurement per data row of a CSV file whose first column is time.
static void readCsv(const std::filesystem::path& path, Measurement::Type type,
                    std::vector<Measurement>& out) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("cannot open " + path.string());

    std::string line;
    std::getline(file, line);  // header
    while (std::getline(file, line)) {
        std::stringstream row(line);
        std::string cell;
        std::vector<double> values;
        while (std::getline(row, cell, ',')) values.push_back(std::stod(cell));

        Measurement m;
        m.t = values[0];
        m.type = type;
        m.z = Eigen::Map<Eigen::VectorXd>(values.data() + 1, values.size() - 1);
        out.push_back(m);
    }
}

std::vector<Measurement> loadMeasurements(const std::filesystem::path& dir) {
    std::vector<Measurement> measurements;
    readCsv(dir / "imu.csv", Measurement::Type::IMU, measurements);
    readCsv(dir / "gps.csv", Measurement::Type::GPS, measurements);
    // stable_sort keeps the IMU sample ahead of a GPS fix with the same timestamp.
    std::stable_sort(measurements.begin(), measurements.end(),
                     [](const Measurement& a, const Measurement& b) { return a.t < b.t; });
    return measurements;
}
