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
GLIDESLOPE_TRAPPED         // Oscillations caused by velocity limiting causing chaser to converge at incorrect range (WIP)
};

inline const char* failure_reason_to_string(FailureReason r) {
    switch(r) {
        case FailureReason::NONE:
            return "none";
        case FailureReason::POSITION_TIMEOUT:
            return "position_timeout";
        case FailureReason::EXCESS_VELOCITY:
            return "excess_velocity";
        case FailureReason::GLIDESLOPE_TRAPPED:
            return "glideslope_trapped";
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
// Monte Carlo Results
// ----------------------------------------------------------------------------

/**
 * @brief Monte Carlo data analysis for individual sample
 */
struct MonteCarloSampleResult {
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
struct PercentileStats {
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
struct FailureBreakdown {
    int position_timeout = 0;
    int excess_velocity = 0;
    int glideslope_trapped = 0;
};

/**
 * @brief Monte Carlo data analysis for multiple samples
 */
struct MonteCarloResult {
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

}

#endif 
