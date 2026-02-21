/**
 * @file navigation_tests.cpp
 * @brief Unit testing for EKF navigation filtering
 */

#include <gtest/gtest.h>
#include "gnc_algorithms.hpp"

using namespace relnav;

// ----------------------------------------------------------------------------
// measurement_function
// ----------------------------------------------------------------------------

TEST(MeasurementFunction, OnVbar) {
    Vec6 x = Vec6::Zero();
    x(1) = -1000.0;

    Vec3 z = measurement_function(x);

    EXPECT_NEAR(z(0), 1000.0, 1e-6);
    EXPECT_NEAR(z(1), -M_PI / 2.0, 1e-6);
    EXPECT_NEAR(z(2), 0.0, 1e-6);
}

TEST(MeasurementFunction, OffAxis) {
    Vec6 x = Vec6::Zero();
    x(0) = 100.0;
    x(1) = -100.0;
    x(2) = 0.0;

    Vec3 z = measurement_function(x);

    EXPECT_NEAR(z(0), std::sqrt(20000.0), 1e-6);
    EXPECT_NEAR(z(1), std::atan2(-100.0, 100.0), 1e-6);
    EXPECT_NEAR(z(2), 0.0, 1e-6);
}