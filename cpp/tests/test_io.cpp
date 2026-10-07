#include <filesystem>
#include <fstream>
#include <stdexcept>

#include <gtest/gtest.h>

#include "io.h"

TEST(Io, LoadsGpsAndImuMergedInTimeOrder) {
    const auto dir = std::filesystem::path(testing::TempDir()) / "kalman_io_test";
    std::filesystem::create_directories(dir);
    std::ofstream(dir / "gps.csv") << "t,x,y\n0.0,1.0,2.0\n1.0,3.0,4.0\n";
    std::ofstream(dir / "imu.csv") << "t,a,yaw_rate\n0.0,0.1,0.2\n0.5,0.3,0.4\n1.0,0.5,0.6\n";

    const std::vector<Measurement> m = loadMeasurements(dir);

    const double times[] = {0.0, 0.0, 0.5, 1.0, 1.0};
    const Measurement::Type types[] = {Measurement::Type::IMU, Measurement::Type::GPS,
                                       Measurement::Type::IMU, Measurement::Type::IMU,
                                       Measurement::Type::GPS};
    ASSERT_EQ(m.size(), 5u);
    for (size_t i = 0; i < m.size(); ++i) {
        EXPECT_EQ(m[i].t, times[i]) << "row " << i;
        EXPECT_EQ(m[i].type, types[i]) << "row " << i;
    }
    EXPECT_TRUE(m[1].z.isApprox(Eigen::Vector2d(1.0, 2.0))) << m[1].z.transpose();
    EXPECT_TRUE(m[2].z.isApprox(Eigen::Vector2d(0.3, 0.4))) << m[2].z.transpose();
}

TEST(Io, ThrowsWhenDataDirectoryIsMissing) {
    EXPECT_THROW(loadMeasurements("/no/such/directory"), std::runtime_error);
}
