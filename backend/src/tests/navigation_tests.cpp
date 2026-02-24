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

// ----------------------------------------------------------------------------
// measurement_jacobian
// ----------------------------------------------------------------------------

TEST(MeasurementJacobian, MatchesFiniteDifference) {
    Vec6 x;
    x << 50.0, -800.0, 30.0, 0.1, -0.5, 0.02;

    Mat36 H = measurement_jacobian(x);

    double eps = 1e-7;
    Mat36 H_fd = Mat36::Zero();

    for (int j = 0; j < 6; ++j) {
        Vec6 x_plus = x;
        Vec6 x_minus = x;
        x_plus(j) += eps;
        x_minus(j) -= eps;

        Vec3 z_plus = measurement_function(x_plus);
        Vec3 z_minus = measurement_function(x_minus);

        H_fd.col(j) = (z_plus - z_minus) / (2.0 * eps);
    }

    EXPECT_NEAR((H - H_fd).norm(), 0.0, 1e-5);
}

// ----------------------------------------------------------------------------
// generate_measurement
// ----------------------------------------------------------------------------

TEST(GenerateMeasurement, ValidIntoRange) {
    Vec6 x = Vec6::Zero();
    x(1) = -500.0;
    SensorModel sensor = default_nav_config().sensor;
    std::mt19937 rng(42);

    Measurement meas = generate_measurement(x, sensor, rng);

    EXPECT_TRUE(meas.valid);
    EXPECT_NEAR(meas.z(0), 500.0, 10.0);
}

TEST(GenerateMeasurement, DropoutBeyondMaxRange) {
    Vec6 x = Vec6::Zero();
    x(1) = -5000.0;
    SensorModel sensor = default_nav_config().sensor;
    std::mt19937 rng(42);

    Measurement meas = generate_measurement(x, sensor, rng);

    EXPECT_FALSE(meas.valid);
}

TEST(GenerateMeasurement, DropoutBelowMinRange) {
    Vec6 x = Vec6::Zero();
    x(1) = -1.0;
    SensorModel sensor = default_nav_config().sensor;
    std::mt19937 rng(42);

    Measurement meas = generate_measurement(x, sensor, rng);

    EXPECT_FALSE(meas.valid);
}

// ----------------------------------------------------------------------------
// ekf_predict
// ----------------------------------------------------------------------------

TEST(EKFPredict, CovarianceGrows) {
    NavConfig config = default_nav_config();
    std::mt19937 rng(42);

    Vec6 x_true = Vec6::Zero();
    x_true << 0, -1000, 0, 0, -0.5, 0;

    NavFilterState state = initialize_filter(x_true, config.filter.P0, rng);
    double trace_before = state.P.trace();

    double n = OrbitalParams(420e3).mean_motion();

    NavFilterState predicted = ekf_predict(
        state, Vec3::Zero(), n, 1.0, config.filter.Q_process
    );

    EXPECT_GT(predicted.P.trace(), trace_before);
}

// ----------------------------------------------------------------------------
// ekf_update
// ----------------------------------------------------------------------------

TEST(EKFUpdate, CovarianceShrinks) {
    NavConfig config = default_nav_config();
    std::mt19937 rng(42);

    Vec6 x_true;
    x_true << 0, -1000, 0, 0, 0.5, 0;

    NavFilterState state = initialize_filter(x_true, config.filter.P0, rng);

    double n = OrbitalParams(420e3).mean_motion();

    // Predict to inflate P
    NavFilterState predicted = ekf_predict(
        state, Vec3::Zero(), n, 1.0, config.filter.Q_process
    );

    // Perfect measurement
    Measurement meas;
    meas.valid = true;
    meas.z = measurement_function(x_true);

    NavFilterState updated = ekf_update(predicted, meas, config.filter.R_meas);

    EXPECT_LT(updated.P.trace(), predicted.P.trace());
}

TEST(EKFUpdate, InvalidMeasurementNoOp) {
    NavConfig config = default_nav_config();
    std::mt19937 rng(42);

    Vec6 x_true;
    x_true << 0, -1000, 0, 0, 0.5, 0;

    NavFilterState state = initialize_filter(x_true, config.filter.P0, rng);

    Measurement meas;
    meas.valid = false;
    meas.z = Vec3::Zero();

    NavFilterState updated = ekf_update(state, meas, config.filter.R_meas);

    EXPECT_EQ(updated.x_hat, state.x_hat);
    EXPECT_EQ(updated.P, state.P);
}

// ----------------------------------------------------------------------------
// initialize_filter
// ----------------------------------------------------------------------------

TEST(InitializeFilter, EstimateOffsetFromTruth) {
    Vec6 x_true;
    x_true << 0, -1000, 0, 0, 0.5, 0;
    NavConfig config = default_nav_config();
    std::mt19937 rng(42);

    NavFilterState state = initialize_filter(x_true, config.filter.P0, rng);

    EXPECT_GT((state.x_hat - x_true).norm(), 1e-10);
    EXPECT_LT((state.x_hat - x_true).head<3>().norm(), 50.0);
}

// ----------------------------------------------------------------------------
// Filter convergence
// ----------------------------------------------------------------------------

TEST(FilterConvergence, ErrorDecreases) {
    NavConfig config = default_nav_config();
    std::mt19937 rng(42);
    double n = OrbitalParams(420e3).mean_motion();
    double dt = 1.0;

    Vec6 x_true;
    x_true << 0, -1000, 0, 0, 0.5, 0;

    NavFilterState state = initialize_filter(x_true, config.filter.P0, rng);

    double initial_error = (state.x_hat - x_true).head<3>().norm();

    // Run 60 steps with measurements every step
    for (int i = 0; i < 60; ++i) {
        state = ekf_predict(state, Vec3::Zero(), n, dt, config.filter.Q_process);
        x_true = cw_state_transition_matrix(n, dt) * x_true;

        Measurement meas = generate_measurement(x_true, config.sensor, rng);
        state = ekf_update(state, meas, config.filter.R_meas);
    }

    double final_error = (state.x_hat - x_true).head<3>().norm();

    EXPECT_LT(final_error, initial_error);
    EXPECT_LT(final_error, 10.0);
}