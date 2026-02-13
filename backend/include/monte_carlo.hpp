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
    UncertaintyModel(const Vec3& pos_err, const Vec3& vel_err);
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
    double final_range;
    double final_velocity;
    double total_dv;
    double duration;
    int saturation_count;
    double max_estimation_error;
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

    // Raw results
    std::array<MonteCarloSampleResult, MAX_MC_SAMPLES> samples;
    std::array<Vec6, MAX_MC_SAMPLES> final_states;
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
    const UncertaintyModel& uncertainty,
    const ApproachParams& params,
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
