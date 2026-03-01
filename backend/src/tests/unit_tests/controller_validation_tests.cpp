/**
 * @file controller_validation_tests.cpp
 * @brief Unit testing for GNC controller validation shtuff
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include "../validation/controller_validation.hpp"

// --------------------------------------------------------------------------
// Helpers
// --------------------------------------------------------------------------

static DataParams make_default_params(const Axes axis) {
    DataParams params;
    params.dispersion.axis = axis;
    params.success_range = 5.0;
    params.u_max = 0.01;
    params.dt = 1.0;
    params.timeout = 6000.0;
    params.corridor_params.k = 0.001;
    params.corridor_params.corridor_angle = 0.175;
    
    switch(axis) {
        case Axes::VBAR:
            params.corridor_params.approach_axis = Vec3{0, -1, 0};
            break;
        case Axes::RBAR:
            params.corridor_params.approach_axis = Vec3{-1, 0, 0};
            break;
        case Axes::HBAR:
            params.corridor_params.approach_axis = Vec3{0, 0, -1};
            break;
        default:
            params.corridor_params.approach_axis = Vec3{0, -1, 0};
            break;
    }

    return params;
}

// --------------------------------------------------------------------------
// dispersed_initial_state
// --------------------------------------------------------------------------

class DisperseInitialState : public ::testing::Test{
    protected:
        DispersionModel vbar_disp;
        DispersionModel rbar_disp;
        DispersionModel hbar_disp;
        DispersionModel unlocked_disp;

        double success_range = 5.0;
        double max_range = 3000.0;
        unsigned int seed = 42;

        void SetUp() override {
            vbar_disp.axis = Axes::VBAR;
            rbar_disp.axis = Axes::RBAR;
            hbar_disp.axis = Axes::HBAR;
            unlocked_disp.axis_lock = false;
        }
};

TEST_F(DisperseInitialState, RangeWithinBounds) {
    std::mt19937 rng(seed);

    Vec6 x = dispersed_initial_state(
        success_range, max_range, vbar_disp, rng
    );

    double range = x.head<3>().norm();

    EXPECT_GE(range, success_range);
    EXPECT_LT(range, max_range);
}

TEST_F(DisperseInitialState, DisperseHonorsVBar) {
    std::mt19937 rng(seed);

    Vec6 x = dispersed_initial_state(
        success_range, max_range, vbar_disp, rng
    );

    EXPECT_LE(x(1), 0.0);
}

TEST_F(DisperseInitialState, DisperseHonorsRBar) {
    std::mt19937 rng(seed);

    Vec6 x = dispersed_initial_state(
        success_range, max_range, rbar_disp, rng
    );

    EXPECT_LE(x(0), 0.0);
}

TEST_F(DisperseInitialState, DisperseHonorsHBar) {
    std::mt19937 rng(seed);

    Vec6 x = dispersed_initial_state(
        success_range, max_range, hbar_disp, rng
    );

    EXPECT_LE(x(2), 0.0);
}

// --------------------------------------------------------------------------
// run_sample
// --------------------------------------------------------------------------

TEST(RunSample, ValuesStored) {
    DataParams params;

    unsigned int seed = 42;

    std::mt19937 rng(seed);

    Sample result = run_sample(params, rng);

    EXPECT_GT(result.total_dv, 0.0);
    EXPECT_GT(result.duration, 0.0);
}

TEST(RunSample, AlreadyAtTargetSucceeds) {
    DataParams params;
    
    Vec6 x0 = Vec6::Zero();
    x0(1) = -1.0;

    std::mt19937 rng(42);
    Sample sample = run_sample(params, rng, x0);

    EXPECT_TRUE(sample.success);
}

// --------------------------------------------------------------------------
// run_samples
// --------------------------------------------------------------------------

TEST(RunSamples, SuccessRateBounded) {
    DataParams params = make_default_params(Axes::VBAR);

    double Q_pos = 10.0;
    double Q_vel = 100.0;
    int n_samples = 100;
    int n_threads = 4;
    unsigned int seed = 42;

    DataPoint pt = run_samples(
        params, Q_pos, Q_vel, n_samples, n_threads, seed
    );

    EXPECT_GE(pt.success_rate, 0.0);
    EXPECT_LE(pt.success_rate, 1.0);
}

TEST(RunSamples, QValuesRecorded) {
    DataParams params = make_default_params(Axes::VBAR);

    double Q_pos = 10.0;
    double Q_vel = 100.0;
    int n_samples = 10;
    int n_threads = 1;
    unsigned int seed = 42;

    DataPoint pt = run_samples(
        params, Q_pos, Q_vel, n_samples, n_threads, seed
    );

    EXPECT_DOUBLE_EQ(pt.Q_pos, Q_pos);
    EXPECT_DOUBLE_EQ(pt.Q_vel, Q_vel);
}

// --------------------------------------------------------------------------
// compute_envelope
// --------------------------------------------------------------------------

TEST(ComputeEnvelope, GridDimensions) {
    DataParams params = make_default_params(Axes::VBAR);

    TestConfig config;
    config.Q_pos_min = 1.0;
    config.Q_pos_max = 100.0;
    config.Q_pos_steps = 3;
    config.Q_vel_min = 0.1;
    config.Q_vel_max = 10.0;
    config.Q_vel_steps = 4;
    config.samples = 10;
    config.seed = 42;

    EnvelopeData env = compute_envelope(config, params, 2);

    EXPECT_EQ(env.Q_pos_vals.size(), 3);
    EXPECT_EQ(env.Q_vel_vals.size(), 4);
    EXPECT_EQ(env.grid.size(), 3);

    for (int i = 0; i < 3; ++i) {
        EXPECT_EQ(env.grid[i].size(), 4);
    }
}

TEST(ComputeEnvelope, LogSpacingEndpoints)
{
    DataParams params = make_default_params(Axes::VBAR);

    TestConfig config;
    config.Q_pos_min = 1.0;
    config.Q_pos_max = 1000.0;
    config.Q_pos_steps = 4;
    config.Q_vel_min = 0.01;
    config.Q_vel_max = 100.0;
    config.Q_vel_steps = 5;
    config.samples = 5;
    config.seed = 42;

    EnvelopeData env = compute_envelope(config, params, 1);

    EXPECT_NEAR(env.Q_pos_vals.front(), 1.0, 1e-9);
    EXPECT_NEAR(env.Q_pos_vals.back(), 1000.0, 1e-6);
    EXPECT_NEAR(env.Q_vel_vals.front(), 0.01, 1e-9);
    EXPECT_NEAR(env.Q_vel_vals.back(), 100.0, 1e-6);
}

TEST(ComputeEnvelope, AllCellsPopulated)
{
    DataParams params = make_default_params(Axes::VBAR);

    TestConfig config;
    config.Q_pos_min = 10.0;
    config.Q_pos_max = 100.0;
    config.Q_pos_steps = 2;
    config.Q_vel_min = 1.0;
    config.Q_vel_max = 10.0;
    config.Q_vel_steps = 2;
    config.samples = 10;
    config.seed = 42;

    EnvelopeData env = compute_envelope(config, params, 2);

    for (int i = 0; i < 2; ++i)
    {
        for (int j = 0; j < 2; ++j)
        {
            EXPECT_EQ(env.grid[i][j].n_samples, 10)
                << "Cell [" << i << "][" << j << "] not populated";
            EXPECT_GT(env.grid[i][j].Q_pos, 0.0);
            EXPECT_GT(env.grid[i][j].Q_vel, 0.0);
        }
    }
}

TEST(ComputeEnvelope, SingleCellGrid)
{
    DataParams params = make_default_params(Axes::RBAR);

    TestConfig config;
    config.Q_pos_min = 50.0;
    config.Q_pos_max = 50.0;
    config.Q_pos_steps = 1;
    config.Q_vel_min = 5.0;
    config.Q_vel_max = 5.0;
    config.Q_vel_steps = 1;
    config.samples = 20;
    config.seed = 42;

    EnvelopeData env = compute_envelope(config, params, 1);

    EXPECT_EQ(env.grid.size(), 1);
    EXPECT_EQ(env.grid[0].size(), 1);
    EXPECT_EQ(env.grid[0][0].n_samples, 20);
}

TEST(ComputeEnvelope, ComputeTimeRecorded)
{
    DataParams params = make_default_params(Axes::VBAR);

    TestConfig config;
    config.Q_pos_min = 10.0;
    config.Q_pos_max = 100.0;
    config.Q_pos_steps = 2;
    config.Q_vel_min = 1.0;
    config.Q_vel_max = 10.0;
    config.Q_vel_steps = 2;
    config.samples = 5;
    config.seed = 42;

    EnvelopeData env = compute_envelope(config, params, 1);

    EXPECT_GT(env.total_compute_time, 0.0);
}