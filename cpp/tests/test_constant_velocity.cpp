#include <gtest/gtest.h>

#include "models/constant_velocity.h"

TEST(ConstantVelocity, VelocityVarianceGrowsLinearlyWithTime) {
    const Eigen::Matrix4d Q = cv_model::Q(2.0, 3.0);

    // sigma_a^2 * dt = 9 * 2
    EXPECT_DOUBLE_EQ(Q(2, 2), 18.0);
    EXPECT_DOUBLE_EQ(Q(3, 3), 18.0);
}

TEST(ConstantVelocity, OneLongStepGivesSameNoiseAsTwoShortSteps) {
    const Eigen::Matrix4d F = cv_model::F(1.0);
    const Eigen::Matrix4d Q = cv_model::Q(1.0, 3.0);

    const Eigen::Matrix4d two_steps = F * Q * F.transpose() + Q;

    EXPECT_TRUE(two_steps.isApprox(cv_model::Q(2.0, 3.0))) << two_steps;
}
