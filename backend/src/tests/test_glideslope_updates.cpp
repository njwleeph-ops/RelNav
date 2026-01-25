/**
 * @file test_glideslope_updates.cpp
 * @brief Unit testing for glideslope guidance functions
 */

#include <iostream>
#include <cmath>

#include <gtest/gtest.h>
#include "gnc_algorithms.hpp"

using namespace relnav;

/// Test approach angle computation
TEST(CorridorAngle, OnAxisReturnsZero) {
    Vec3 r{0, -500, 0};
    Vec3 axis{0, -1, 0};
    double angle = glideslope_approach_angle(r, axis);
    EXPECT_NEAR(angle, 0.0, 1e-6);
}

TEST(CorridorAngle, FortyFiveDegrees) {
    Vec3 r{500, -500, 0};
    Vec3 axis{0, -1, 0};
    double angle = glideslope_approach_angle(r, axis);
    EXPECT_NEAR(angle, M_PI / 4, 1e-6);
}

TEST(CorridorAngle, NinetyDegrees) {
    Vec3 r{500, 0, 0};
    Vec3 axis{0, -1, 0};
    double angle = glideslope_approach_angle(r, axis);
    EXPECT_NEAR(angle, M_PI / 2, 1e-6);
}


