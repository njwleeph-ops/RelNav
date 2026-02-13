/**
 * @file monte_carlo_tests.cpp
 * @brief Unit testing for Monte Carlo shtuff
 */

#include <gtest/gtest.h>
#include <cmath>

#include "monte_carlo.hpp"

using namespace relnav;

// ----------------------------------------------------------------------------
// sample_initial_state
// ----------------------------------------------------------------------------

TEST(SampleInitialState, NominalWithZeroUncertainty) {
    Vec6 x0_nominal;
    x0_nominal << 0, -500, 0, 0, 0, 0;

    UncertaintyModel uncertainty;
    uncertainty.pos_error = Vec3::Zero();
    uncertainty.vel_error = Vec3::Zero();

    std::mt19937 rng(42);
    Vec6 x0 = sample_initial_state(x0_nominal, uncertainty, rng);

    EXPECT_NEAR(x0.norm(), x0_nominal.norm(), 1e-10);
}

TEST(SampleInitialState, DspersesWithUncertainty) {
    Vec6 x0_nominal;
    x0_nominal << 0, -500, 0, 0, 0, 0;

    UncertaintyModel uncertainty;
    uncertainty.pos_error = Vec3{10.0, 10.0, 10.0};
    uncertainty.vel_error = Vec3{0.1, 0.1, 0.1};

    std::mt19937 rng(42);

    double max_pos_diff = 0.0;

    for (int i = 0; i < 100; ++i) {
        Vec6 x0 = sample_initial_state(x0_nominal, uncertainty, rng);
        double pos_diff = (x0.head<3>() - x0_nominal.head<3>()).norm();
        max_pos_diff = std::max(max_pos_diff, pos_diff);
    }

    EXPECT_GT(max_pos_diff, 5.0);
}

// ----------------------------------------------------------------------------
// disperse_thrust
// ----------------------------------------------------------------------------

TEST(DisperseThrust, ZeroThrustStaysZero) {
    Vec3 u = Vec3::Zero();
    UncertaintyModel uncertainty;

    std::mt19937 rng(42);
    Vec3 u_dispersed = disperse_thrust(u, uncertainty, rng);

    EXPECT_NEAR(u_dispersed.norm(), 0.0, 1e-10);
}

TEST(DisperseThrust, MagnitudeChanges) {
    Vec3 u{0.01, 0.03, 0.0};
    UncertaintyModel uncertainty;
    uncertainty.thrust_mag_error = 0.1;

    std::mt19937 rng(42);

    double max_diff = 0.0;
    
    for (int i = 0; i < 100; ++i) {
        Vec3 u_dispersed = disperse_thrust(u, uncertainty, rng);
        double diff = std::abs(u_dispersed.norm() - u.norm());
        max_diff = std::max(diff, max_diff);
    }

    EXPECT_GT(max_diff, 0.005);
}

TEST(DisperseThrust, DirectionChanges) {
    Vec3 u{0.01, 0.03, 0.0};
    UncertaintyModel uncertainty;
    uncertainty.thrust_pointing_error = 0.05;

    std::mt19937 rng(42);

    double max_angle = 0.0;

    for (int i = 0; i < 100; ++i) {
        Vec3 u_dispersed = disperse_thrust(u, uncertainty, rng);
        double dot = u.normalized().dot(u_dispersed.normalized());
        double angle = std::acos(std::clamp(dot, -1.0, 1.0));
        max_angle = std::max(max_angle, angle);
    }

    EXPECT_GT(max_angle, 0.01);
}