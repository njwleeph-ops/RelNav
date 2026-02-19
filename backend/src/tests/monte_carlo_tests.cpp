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

TEST(RunMonteCarlo, StressedUncertaintiesProduceFailures) {
    Vec6 x0;
    x0 << 200, -500, 200, 0, 0, 0;

    double n = OrbitalParams(420e3).mean_motion();
    ApproachParams params;
    params.u_max = 0.005;
    params.timeout = 3000.0;

    UncertaintyModel uncertainty;
    uncertainty.pos_error = Vec3{100.0, 100.0, 100.0};
    uncertainty.vel_error = Vec3{0.1, 0.1, 0.1};
    uncertainty.thrust_mag_error = 0.15;
    uncertainty.thrust_pointing_error = 0.08;

    MonteCarloResult result = run_monte_carlo(x0, n, params, uncertainty, 100, 4, 42);

    int total_failures = result.failures.position_timeout + result.failures.excess_velocity;

    EXPECT_EQ(total_failures, result.n_samples - result.n_success);
    EXPECT_LT(result.success_rate, 1.0);
    
    // Check percentiles
    EXPECT_LE(result.dv_percentiles.min, result.dv_percentiles.p50);
    EXPECT_LE(result.dv_percentiles.p50, result.dv_percentiles.p75);
    EXPECT_LE(result.dv_percentiles.p75, result.dv_percentiles.p90);
    EXPECT_LE(result.dv_percentiles.p90, result.dv_percentiles.p95);
    EXPECT_LE(result.dv_percentiles.p95, result.dv_percentiles.p99);
    EXPECT_LE(result.dv_percentiles.p99, result.dv_percentiles.max);
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

TEST(FailureReason, SingleSampleExcessVelocity) {
    Vec6 x0;
    x0 << 0, -100, 0, 0, 1.0, 0;

    double n = OrbitalParams(420e3).mean_motion();
    ApproachParams params;
    params.success_range = 10.0;
    params.success_velocity = 0.0001;
    params.timeout = 1500.0;

    UncertaintyModel uncertainty;
    uncertainty.pos_error = Vec3::Zero();
    uncertainty.vel_error = Vec3::Zero();
    uncertainty.thrust_mag_error = 0.0;
    uncertainty.thrust_pointing_error = 0.0;

    std::mt19937 rng(42);
    Vec6 final_state;

    MonteCarloSampleResult result = run_sample(x0, n, params, uncertainty, rng, final_state);

    std::cout << "final_range = " << result.final_range << " | final_velocity = " << result.final_velocity << std::endl; 

    EXPECT_STREQ(failure_reason_to_string(result.failure_reason), failure_reason_to_string(FailureReason::EXCESS_VELOCITY));
}

// ----------------------------------------------------------------------------
// Percentile computation
// ----------------------------------------------------------------------------

TEST(Percentiles, BasiComputation) {
    MonteCarloResult mc;
    mc.n_samples = 100;
    mc.samples.resize(100);
    mc.final_states.resize(100);

    for (int i = 0; i < 100; ++i) {
        mc.samples[i].success = true;
        mc.samples[i].failure_reason = FailureReason::NONE;
        mc.samples[i].total_dv = static_cast<double>(i + 1);
        mc.samples[i].duration = static_cast<double>((i + 1) * 10);
        mc.samples[i].final_range = 1.0;
        mc.samples[i].final_velocity = 0.01;
        mc.samples[i].saturation_count = 0;
        mc.final_states[i] = Vec6::Zero();
    }

    compute_statistics(mc);

    EXPECT_NEAR(mc.dv_percentiles.min, 1.0, 0.01);
    EXPECT_NEAR(mc.dv_percentiles.max, 100.0, 0.01);
    EXPECT_NEAR(mc.dv_percentiles.p50, 50.5, 1.0);
    EXPECT_NEAR(mc.dv_percentiles.p95, 95.5, 1.0);
    EXPECT_NEAR(mc.dv_percentiles.p99, 99.5, 1.0);

    EXPECT_NEAR(mc.duration_percentiles.min, 10.0, 0.1);
    EXPECT_NEAR(mc.duration_percentiles.max, 1000.0, 0.1);
    EXPECT_NEAR(mc.duration_percentiles.p50, 505.0, 10.0);
}

TEST(Percentiles, SingleSample) {
    MonteCarloResult mc;
    mc.n_samples = 1;
    mc.samples.resize(1);
    mc.final_states.resize(1);

    mc.samples[0].success = true;
    mc.samples[0].failure_reason = FailureReason::NONE;
    mc.samples[0].total_dv = 5.0;
    mc.samples[0].duration = 3000.0;
    mc.samples[0].final_range = 2.0;
    mc.samples[0].final_velocity = 0.01;
    mc.samples[0].saturation_count = 10;
    mc.final_states[0] = Vec6::Zero();

    compute_statistics(mc);

    EXPECT_NEAR(mc.dv_percentiles.min, 5.0, 1e-10);
    EXPECT_NEAR(mc.dv_percentiles.max, 5.0, 1e-10);
    EXPECT_NEAR(mc.dv_percentiles.p50, 5.0, 1e-10);
    EXPECT_NEAR(mc.dv_percentiles.p99, 5.0, 1e-10);
}

// ----------------------------------------------------------------------------
// Failure breakdown tallying
// ----------------------------------------------------------------------------

TEST(FailureBreakdown, CorrectCounts) {
    MonteCarloResult mc;
    mc.n_samples = 10;
    mc.samples.resize(10);
    mc.final_states.resize(10);

    for (int i = 0; i < 10; ++i) {
        mc.samples[i].total_dv = 5.0;
        mc.samples[i].duration = 3000.0;
        mc.samples[i].final_range = 2.0;
        mc.samples[i].final_velocity = 0.01;
        mc.samples[i].saturation_count = 0;
        mc.final_states[i] = Vec6::Zero();
    }

    for (int i = 0; i < 5; ++i) {
        mc.samples[i].success = true;
        mc.samples[i].failure_reason = FailureReason::NONE;
    }

    for (int i = 5; i < 8; ++i) {
        mc.samples[i].success = false;
        mc.samples[i].failure_reason = FailureReason::POSITION_TIMEOUT;
    }

    mc.samples[8].success = false;
    mc.samples[8].failure_reason = FailureReason::EXCESS_VELOCITY;
    mc.samples[9].success = false;
    mc.samples[9].failure_reason = FailureReason::EXCESS_VELOCITY;

    compute_statistics(mc);

    EXPECT_EQ(mc.n_success, 5);
    EXPECT_NEAR(mc.success_rate, 0.5, 1e-10);
    EXPECT_EQ(mc.failures.position_timeout, 3);
    EXPECT_EQ(mc.failures.excess_velocity, 2);
}

// ----------------------------------------------------------------------------
// state_from_range
// ----------------------------------------------------------------------------

TEST(StateFromRange, VbarPlacesAlongY) {
    ApproachParams params;
    params.corridor_params.approach_axis = Vec3{0, -1, 0};

    Vec6 x0 = state_from_range(500.0, params);

    EXPECT_NEAR(x0(0), 0.0, 1e-10);
    EXPECT_NEAR(x0(1), -500.0, 1e-10);
    EXPECT_NEAR(x0(2), 0.0, 1e-10);

    // Velocity is unchanged
    EXPECT_NEAR(x0.tail<3>().norm(), 0.0, 1e-10);

    // Range is preserved
    EXPECT_NEAR(x0.head<3>().norm(), 500.0, 1e-10);
}

TEST(StateFromRange, RbarPlacesAlongX) {
    ApproachParams params;
    params.corridor_params.approach_axis = Vec3{-1, 0, 0};

    Vec6 x0 = state_from_range(500.0, params);

    EXPECT_NEAR(x0(1), 0.0, 1e-10);
    EXPECT_NEAR(x0(0), -500.0, 1e-10);
    EXPECT_NEAR(x0(2), 0.0, 1e-10);

    // Velocity unchanged
    EXPECT_NEAR(x0.tail<3>().norm(), 0.0, 1e-10);

    // Range preserved
    EXPECT_NEAR(x0.head<3>().norm(), 500.0, 1e-10);
}

TEST(StateFromRange, HbarPlacesAlongZ) {
    ApproachParams params;
    params.corridor_params.approach_axis = Vec3{0, 0, -1};

    Vec6 x0 = state_from_range(500.0, params);

    EXPECT_NEAR(x0(0), 0.0, 1e-10);
    EXPECT_NEAR(x0(2), -500.0, 1e-10);
    EXPECT_NEAR(x0(1), 0.0, 1e-10);

    // Velocity unchanged
    EXPECT_NEAR(x0.tail<3>().norm(), 0.0, 1e-10);

    // Range preserved
    EXPECT_NEAR(x0.head<3>().norm(), 500.0, 1e-10);
}

// ----------------------------------------------------------------------------
// Grid construction
// ----------------------------------------------------------------------------

TEST(Envelope, GridDimensionsMatchConfig) {
    ApproachParams params;
    params.corridor_params.approach_axis = Vec3{0, -1, 0};

    EnvelopeConfig config;
    config.range_steps = 5;
    config.u_max_steps = 4;
    config.samples_per_point = 10;

    double n = OrbitalParams(420e3).mean_motion();

    PerformancEnvelope envelope = compute_performance_envelope(
        params, UncertaintyModel(), config, n
    );

    EXPECT_EQ(envelope.range_values.size(), 5);
    EXPECT_EQ(envelope.u_max_values.size(), 4);
    EXPECT_EQ(envelope.grid.size(), 5);

    for (auto& row : envelope.grid) {
        EXPECT_EQ(row.size(), 4);
    }
}




