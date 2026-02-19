/**
 * @file monte_carlo.hpp
 * @brief Monte Carlo Dispersion Analysis - Declarations
 */

#ifndef MONTE_CARLO_HPP
#define MONTE_CARLO_HPP

#include <random>

#include "gnc_algorithms.hpp"
 
namespace relnav {

// ----------------------------------------------------------------------------
// Constants
// ----------------------------------------------------------------------------

constexpr int MAX_MC_SAMPLES = 10000;

// ----------------------------------------------------------------------------
// Failure Classification
// ----------------------------------------------------------------------------

enum class FailureReason {
NONE = 0,               // Success - no failure
POSITION_TIMEOUT,       // Timeout, chaser could never reach target position
EXCESS_VELOCITY,        // Guidance algorithm unable to clamp velocity effectively
};

inline const char* failure_reason_to_string(FailureReason r) {
    switch(r) {
        case FailureReason::NONE:
            return "none";
        case FailureReason::POSITION_TIMEOUT:
            return "position_timeout";
        case FailureReason::EXCESS_VELOCITY:
            return "excess_velocity";
        default:
            return "N/A";
    }
}


// ----------------------------------------------------------------------------
// Structs
// ----------------------------------------------------------------------------

/**
 * @brief Uncertainty model for Monte Carlo sampling
 * 
 * All values are 1-sigma (standard deviation).
 */
struct UncertaintyModel {
    Vec3 pos_error{5.0, 5.0, 5.0};               // Position uncertainty [m]
    Vec3 vel_error{0.01, 0.01, 0.01};            // Velocity uncertainty [m/s]
    double thrust_mag_error = 0.03;              // Thrust magnitude error
    double thrust_pointing_error = 0.0175;       // Thrust pointing error 

    UncertaintyModel() = default;
};

/**
 * @brief Monte Carlo data analysis for individual sample
 */
struct MonteCarloSampleResult
{
    bool success;
    FailureReason failure_reason = FailureReason::NONE;
    double final_range;
    double final_velocity;
    double total_dv;
    double duration;
    int saturation_count;
};

/**
 * @brief Percential stats for distribution
 */
struct PercentileStats
{
    double p50 = 0.0;
    double p75 = 0.0;
    double p90 = 0.0;
    double p95 = 0.0;
    double p99 = 0.0;
    double min = 0.0;
    double max = 0.0;
};

/**
 * @brief Failure mode breakdown
 */
struct FailureBreakdown
{
    int position_timeout = 0;
    int excess_velocity = 0;
    int glideslope_trapped = 0;
};

/**
 * @brief Monte Carlo data analysis for multiple samples
 */
struct MonteCarloResult
{
    int n_samples;
    int n_success;
    double success_rate;

    // Stats over samples
    double mean_dv;
    double std_dv;
    double mean_duration;
    double std_duration;
    double mean_final_range;
    double mean_saturation_count;

    // Percentiles
    PercentileStats dv_percentiles;
    PercentileStats duration_percentiles;

    // Failure breakdown
    FailureBreakdown failures;

    // Raw results
    std::vector<MonteCarloSampleResult> samples;
    std::vector<Vec6> final_states;
};

/**
 * @brief Point on Performance Envelope heatmap
 */
struct EnvelopePoint {
    double range = 0.0;
    double u_max = 0.0;

    // Primary map (Success ranges)
    double success_rate = 0.0;
    int n_success = 0;
    int n_samples = 0;

    // Secondary map (Fuel budgeting)
    double mean_dv = 0.0;
    double p95_dv = 0.0;
    double p99_dv = 0.0;

    // Tertiary map (Failure classification)
    FailureBreakdown failures;
    FailureReason dominant_failure = FailureReason::NONE;

    // Timeline
    double mean_duration = 0.0;
    double p95_duration = 0.0;
};

/**
 * @brief Configuration for Performance Envelope sweep
 */
struct EnvelopeConfig {
    // Range
    double range_min = 100.0;       // [m]
    double range_max = 3000.0;      // [m]
    int range_steps = 20;

    // Thrust
    double u_max_min = 0.001;       // [m/s^2]
    double u_max_max = 0.05;        // [m/s^2]
    int u_max_steps = 20;

    // MC config per cell
    int samples_per_point = 200;
    unsigned int seed = 42;         // For reproducibility
};

/**
 * @brief Complete Performance Envelope result
 */
struct PerformanceEnvelope {
    // Axes
    std::vector<double> range_vals;
    std::vector<double> u_max_vals;

    // Result grid: [range_idx][u_max_idx]
    std::vector<std::vector<EnvelopePoint>> grid;

    // Configuration that produced this result
    EnvelopeConfig config;
    ApproachParams params;
    UncertaintyModel uncertainty;
    std::string approach_axis;

    // Timing
    double total_compute_time = 0.0;    // [s]
};

// ----------------------------------------------------------------------------
// Sampling
// ----------------------------------------------------------------------------

/**
 * @brief Sample dispersed initial states
 * @param x0_nominal Nominal initial state
 * @param uncertainty Uncertainty parameters
 * @param n_samples Number of dispersed samples
 * @param seed Characteristic seed for RNG for reproducibility
 */
Vec6 sample_initial_state(
    const Vec6& x0_nominal,
    const UncertaintyModel& uncertainty,
    std::mt19937& rng
);

/**
 * @brief Apply thrust dispersion to control vector
 * @param u_nominal Nominal thrust control vector
 * @param uncertainty Uncertainty parameters
 * @param rng Random number generator 
 */
Vec3 disperse_thrust(
    const Vec3& u_nominal,
    const UncertaintyModel& uncertainty,
    std::mt19937& rng
);

// ----------------------------------------------------------------------------
// Function Declarations
// ----------------------------------------------------------------------------

/**
 * @brief Run single sample with closed-loop guidance and control
 */
MonteCarloSampleResult run_sample(
    const Vec6& x0,
    double n,
    const ApproachParams& params,
    const UncertaintyModel& uncertainty,
    std::mt19937& rng,
    Vec6& final_state
);

/**
 * @brief Run full Monte Carlo dispersion analysis
 * @param x0_nominal Nominal initial state
 * @param n Mean motion [rad/s]
 * @param params Approach guidance parameters
 * @param uncertainty Uncertainty model
 * @param n_samples Number of Monte Carlo samples
 * @param n_threads Number of threads 
 * @param seed Random seed for reproducibility
 * @return Monte Carlo results with statistics and validation
 */
MonteCarloResult run_monte_carlo(
    const Vec6& x0_nominal,
    double n,
    const ApproachParams& params,
    const UncertaintyModel& uncertainty,
    int n_samples = 10000,
    int n_threads = 0,
    unsigned int seed = 42
);

/**
 * @brief Compute statistics from raw results
 */
void compute_statistics(MonteCarloResult& result);

/**
 * @brief Construct initial state from range and approach axis 
 * pulled from approach parameters
 */
Vec6 state_from_range(double range, const ApproachParams& params);

/**
 * @brief Run MC at a single grid point
 * @param range Initial range
 * @param u_max Max thrust 
 * @param n Mean motion
 * @param params Approach parameters
 * @param uncertainty Constant uncertainty model
 * @param n_samples Number of MC samples at grid point
 * @param seed RNG seed for reproducibility
 * @return Relevant data for Performance Envelope from MC campaign
 */
EnvelopePoint run_envelope_point(
    double range,
    double u_max,
    double n,
    const ApproachParams& params,
    const UncertaintyModel& uncertainty,
    int n_samples,
    unsigned int seed
);

/**
 * @brief Compute a 2D Performance Envelope
 * 
 * Sweeps initial range and u_max settings, runs a full MC campaign
 * for each pair. Initial range changed based on approach axis setting
 * 
 * @param params Approach parameters with u_max overriden for each cell
 * @param uncertainty Uncertainty model held constant
 * @param config Sweep grid configuration
 * @param n Mean motion
 * @param n_threads Thread count (0 = auto-detect)
 */
PerformanceEnvelope compute_performance_envelope(
    const ApproachParams& params,
    const UncertaintyModel& uncertainty,
    const EnvelopeConfig& config,
    double n,
    int n_threads = 0
);

}

#endif 
