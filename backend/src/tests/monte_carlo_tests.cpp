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

// ----------------------------------------------------------------------------
// run_sample
// ----------------------------------------------------------------------------

TEST(RunSample, SucceedsFromNominal) {
    Vec6 x0;
    x0 << 0, -500, 0, 0, 0, 0;

    double n = OrbitalParams(420e3).mean_motion();

    ApproachParams params;
    params.Q = Mat6::Identity();
    params.Q.block<3, 3>(0, 0) *= 10.0;
    params.Q.block<3, 3>(3, 3) *= 50.0;
    params.R = Mat3::Identity();
    params.corridor_params.k = 0.001;

    UncertaintyModel uncertainty;
    uncertainty.pos_error = Vec3::Zero();
    uncertainty.vel_error = Vec3::Zero();
    uncertainty.thrust_mag_error = 0.0;
    uncertainty.thrust_pointing_error = 0.0;

    std::mt19937 rng(42);
    Vec6 final_state;

    MonteCarloSampleResult result = run_sample(x0, n, params, uncertainty, rng, final_state);

    ApproachResult guidance_result = run_approach_guidance(x0, n, params);

    EXPECT_TRUE(result.success);
    EXPECT_LT(result.final_range, params.success_range);

    std::cout << guidance_result.success << " | " << guidance_result.final_range 
              << " | " << guidance_result.trajectory.count << std::endl;

    std::cout << "failure reason is " << failure_reason_to_string(result.failure_reason) << std::endl;
}

TEST(RunSample, TracksStatistics) {
    Vec6 x0;
    x0 << 0, -500, 0, 0, 0, 0;

    double n = OrbitalParams(420e3).mean_motion();
    ApproachParams params;
    params.Q = Mat6::Identity();
    params.R = Mat3::Identity();
    params.corridor_params.k = 0.001;

    UncertaintyModel uncertainty;

    std::mt19937 rng(42);
    Vec6 final_state;

    MonteCarloSampleResult result = run_sample(x0, n, params, uncertainty, rng, final_state);

    EXPECT_GT(result.total_dv, 0.0);
    EXPECT_GT(result.duration, 0.0);
}

// ----------------------------------------------------------------------------
// run_monte_carlo
// ----------------------------------------------------------------------------

TEST(RunMonteCarlo, ReturnsCorrectSampleCount) {
    Vec6 x0;
    x0 << 0, -500, 0, 0, 0, 0;

    double n = OrbitalParams(420e3).mean_motion();
    ApproachParams params;
    params.Q = Mat6::Identity();
    params.R = Mat3::Identity();
    params.corridor_params.k = 0.001;

    UncertaintyModel uncertainty;

    MonteCarloResult result = run_monte_carlo(x0, n, params, uncertainty, 50, 2, 42);

    EXPECT_EQ(result.n_samples, 50);
}

TEST(RunMonteCarlo, ComputesSuccessRate) {
    Vec6 x0;
    x0 << 0, -500, 0, 0, 0, 0;

    double n = OrbitalParams(420e3).mean_motion();
    ApproachParams params;
    params.Q = Mat6::Identity();
    params.R = Mat3::Identity();
    params.corridor_params.k = 0.001;

    UncertaintyModel uncertainty;
    uncertainty.pos_error =Vec3{5.0, 5.0, 5.0};
    uncertainty.vel_error = Vec3{0.01, 0.01, 0.01};

    MonteCarloResult result = run_monte_carlo(x0, n, params, uncertainty, 100, 4, 42);

    EXPECT_GE(result.success_rate, 0.0);
    EXPECT_LE(result.success_rate, 1.0);
    EXPECT_EQ(result.n_success, static_cast<int>(result.success_rate * result.n_samples));
}

TEST(RunMonteCarlo, ComputesStatistics) {
    Vec6 x0;
    x0 << 0, -500, 0, 0, 0, 0;

    double n = OrbitalParams(420e3).mean_motion();
    ApproachParams params;
    params.Q = Mat6::Identity();
    params.R = Mat3::Identity();
    params.corridor_params.k = 0.001;

    UncertaintyModel uncertainty;
    uncertainty.pos_error = Vec3{5.0, 5.0, 5.0};

    MonteCarloResult result = run_monte_carlo(x0, n, params, uncertainty, 50, 2, 42);

    EXPECT_GT(result.mean_dv, 0.0);
    EXPECT_GT(result.mean_duration, 0.0);
    EXPECT_GT(result.std_dv, 0.0);
}

TEST(RunMonteCarlo, HighUncertaintyLowersSuccessRate) {
    Vec6 x0;
    x0 << 0, -500, 0, 0, 0, 0;

    double n = OrbitalParams(420e3).mean_motion();
    ApproachParams params;
    params.Q = Mat6::Identity();
    params.R = Mat3::Identity();
    params.corridor_params.k = 0.001;
    params.timeout = 5000.0;

    // Low uncertainty
    UncertaintyModel low_uncertainty;
    low_uncertainty.pos_error = Vec3{1.0, 1.0, 1.0};
    low_uncertainty.vel_error = Vec3{0.001, 0.001, 0.001};
    low_uncertainty.thrust_mag_error = 0.01;

    // High uncertainty
    UncertaintyModel high_uncertainty;
    high_uncertainty.pos_error = Vec3{50.0, 50.0, 50.0};
    high_uncertainty.vel_error = Vec3{0.1, 0.1, 0.1};
    high_uncertainty.thrust_mag_error = 0.1;

    MonteCarloResult low_result = run_monte_carlo(x0, n, params, low_uncertainty, 100, 4, 42);
    MonteCarloResult high_result = run_monte_carlo(x0, n, params, high_uncertainty, 100, 4, 42);

    EXPECT_GE(low_result.success_rate, high_result.success_rate);
}

TEST(RunMonteCarlo, MultipleThreadsProduceSameCount) {
    Vec6 x0;
    x0 << 0, -500, 0, 0, 0, 0;

    double n = OrbitalParams(420e3).mean_motion();
    ApproachParams params;
    params.Q = Mat6::Identity();
    params.R = Mat3::Identity();
    params.corridor_params.k = 0.001;

    UncertaintyModel uncertainty;

    MonteCarloResult result_1_thread = run_monte_carlo(x0, n, params, uncertainty, 100, 1, 42);
    MonteCarloResult result_4_threads = run_monte_carlo(x0, n, params, uncertainty, 100, 4, 42);

    EXPECT_EQ(result_1_thread.n_samples, result_4_threads.n_samples);
    EXPECT_EQ(result_1_thread.success_rate, result_4_threads.success_rate);
}

// ----------------------------------------------------------------------------
// compute_statistics
// ----------------------------------------------------------------------------

TEST(ComputeStatistics, HandlesAllSuccess) {
    MonteCarloResult result;
    result.n_samples = 3;
    result.samples.resize(3);
    result.final_states.resize(3);

    result.samples[0] = {true, FailureReason::NONE, 1.0, 0.01, 0.5, 100.0, 0};
    result.samples[1] = {true, FailureReason::NONE, 2.0, 0.02, 0.6, 110.0, 1};
    result.samples[2] = {true, FailureReason::NONE, 1.5, 0.015, 0.55, 105.0, 0};

    compute_statistics(result);

    EXPECT_EQ(result.n_success, 3);
    EXPECT_NEAR(result.success_rate, 1.0, 1e-10);
    EXPECT_NEAR(result.mean_dv, 0.55, 1e-10);
    EXPECT_NEAR(result.mean_duration, 105.0, 1e-10);
}

TEST(ComputeStatistics, HandlesMixedResults) {
    MonteCarloResult result;
    result.n_samples = 4;
    result.samples.resize(4);
    result.final_states.resize(4);

    result.samples[0] = {true, FailureReason::NONE, 1.0, 0.01, 1.0, 100.0, 0};
    result.samples[1] = {false, FailureReason::POSITION_TIMEOUT, 50.0, 0.5, 2.0, 6000.0, 10};
    result.samples[2] = {true, FailureReason::NONE, 2.0, 0.02, 1.0, 100.0, 0};
    result.samples[3] = {false, FailureReason::EXCESS_VELOCITY, 40.0, 0.4, 2.0, 6000.0, 5};

    compute_statistics(result);

    EXPECT_EQ(result.n_success, 2);
    EXPECT_NEAR(result.success_rate, 0.5, 1e-10);
}

// ----------------------------------------------------------------------------
// FailureReason Tests
// ----------------------------------------------------------------------------

TEST(FailureReason, SingleSamplePositionTimeout) {
    Vec6 x0;
    x0 << 500, -3000, 500, 0, 0, 0;

    double n = OrbitalParams(420e3).mean_motion();
    ApproachParams params;
    params.u_max = 0.001;
    params.timeout = 500.0;     // Force timeout

    UncertaintyModel uncertainty;
    uncertainty.pos_error = Vec3::Zero();
    uncertainty.vel_error = Vec3::Zero();
    uncertainty.thrust_mag_error = 0.0;
    uncertainty.thrust_pointing_error = 0.0;

    std::mt19937 rng(42);
    Vec6 final_state;

    MonteCarloSampleResult result = run_sample(x0, n, params, uncertainty, rng, final_state);

    EXPECT_FALSE(result.success);
    EXPECT_STREQ(failure_reason_to_string(result.failure_reason), failure_reason_to_string(FailureReason::POSITION_TIMEOUT));
    EXPECT_GT(result.final_range, params.success_range);
}

TEST(FailureReason, SingleSampleGlideslopeTrapped) {
    Vec6 x0;
    x0 << 0, -100, 0, 0, 0.5, 0;

    double n = OrbitalParams(420e3).mean_motion();
    ApproachParams params;
    params.success_range = 10.0;
    params.success_velocity = 0.001;
    params.timeout = 6000.0;

    UncertaintyModel uncertainty;
    uncertainty.pos_error = Vec3::Zero();
    uncertainty.vel_error = Vec3::Zero();
    uncertainty.thrust_mag_error = 0.0;
    uncertainty.thrust_pointing_error = 0.0;

    std::mt19937 rng(42);
    Vec6 final_state;

    MonteCarloSampleResult result = run_sample(x0, n, params, uncertainty, rng, final_state);

    EXPECT_STREQ(failure_reason_to_string(result.failure_reason), failure_reason_to_string(FailureReason::GLIDESLOPE_TRAPPED));
}