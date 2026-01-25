/**
 * @file monte_carlo.hpp
 * @brief Monte Carlo Dispersion Analysis - Declarations
 * 
 * Quantifies approach corridor compliance under navigation
 * and thruster uncertainties through ensemble propagation.
 */

#ifndef MONTE_CARLO_HPP
#define MONTE_CARLO_HPP

#include <random>

#include "cw_dynamics.hpp"
#include "gnc_algorithms.hpp"
 
namespace relnav {

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

/**
 * @brief Approach corridor geometry for compliance checking
 */
struct ApproachCorridor {
    enum class Type { Cylinder, Cone, Box };

    Type type = Type::Cylinder;
    double radius = 200.0;                  // [m] for cylinder
    double cone_half_angle = 0.175;         // [rad]
    Vec3 box_half_width{100, 100, 100};     // [m] for box

    ApproachCorridor() = default;
    ApproachCorridor(Type t, double rad, double cone_angle, const Vec3& box_half);

    /**
     * @brief Check if position is inside corridor
     * @param position Position in LVLH frame [m]
     * @return true if inside corridor
     */
    bool is_inside(const Vec3 &position) const;
};

/**
 * @brief Monte Carlo analysis results
 */
struct MonteCarloResult {
    std::vector<double> times;                  // Time points [s]
    std::vector<std::vector<Vec6>> ensemble;    // [sample][time] states
    std::vector<Vec6> mean;                     // Mean state at each time
    std::vector<Mat6> covariance;               // Covariance at each time
    std::vector<Vec3> sigma3_pos;               // 3 standard deviation bounds
    std::vector<double> corridor_compliance;    // Fraction inside corridor

    // Validation against analytical covariance
    std::vector<Mat6> analytical_cov;  
    double max_cov_error = 0;       // Max error in 3 standard deviations [%]
    double mean_cov_error = 0;      // Mean error in 3 standard deviations [%]

    int n_samples = 0;
};

/**
 * @brief Target Monte Carlo analysis results
 */
struct TargetedMonteCarloResult {
    Vec3 target;                            // Target's location
    std::vector<Vec6> final_states;         // Final state of each samples
    std::vector<bool> inside_corridor;      // Whether each sample hit corridor
    Vec3 mean_arrival;                      
    Vec3 sigma3_arrival;
    double corridor_success_rate;
    int n_samples;
};

// ----------------------------------------------------------------------------
// Function Declarations
// ----------------------------------------------------------------------------

/**
 * @brief Sample initial states from uncertainty distribution
 * @param x0_nominal Nominal initial state
 * @param uncertainty Uncertainty model
 * @param n_samples Number of samples to generate
 * @param seed Random seed
 * @return Vector of sampled initial states
 */
std::vector<Vec6> sample_initial_states(
    const Vec6& x0_nominal,
    const UncertaintyModel& uncertainty,
    int n_samples,
    unsigned int seed = 42
);

/**
 * @brief Compute analytical covariance propagation
 * 
 * P(t) = Φ(t) * P0 * Φ(t)^T
 * 
 * @param P0 Initial covariance matrix
 * @param n Mean motion [rad/s]
 * @param t Propagation time [s]
 * @return Propagated covariance matrix
 */
Mat6 propagate_covariance_analytical(const Mat6& P0, double n, double t);

/**
 * @brief Run full Monte Carlo dispersion analysis
 * @param x0_nominal Nominal initial state
 * @param duration Propagation duration [s]
 * @param n Mean motion [rad/s]
 * @param uncertainty Uncertainty model
 * @param corridor Approach corridor for compliance checking
 * @param n_samples Number of Monte Carlo samples
 * @param num_times Number of time evaluation points
 * @param seed Random seed for reproducibility
 * @return Monte Carlo results with statistics and validation
 */
MonteCarloResult run_monte_carlo(
    const Vec6& x0_nominal,
    double duration,
    double n,
    const UncertaintyModel& uncertainty = UncertaintyModel(),
    const ApproachCorridor& corridor = ApproachCorridor(),
    int n_samples = 10000,
    int num_times = 100,
    unsigned int seed = 42
);

/**
 * @brief Run Monte Carlo analysis on targeted two-impulse transfer
 * 
 * @param x0_nominal Nominal initial state
 * @param target Target position
 * @param tof Time of flight [s]
 * @param n Mean motion [rad/s]
 * @param uncertainty Nav/state uncertainty model
 * @param corridor Approach corridor geometry
 * @param n_samples Number of samples
 * @param seed Random seed for rng
 * @return Analysis results with corridor hit rate
 */
TargetedMonteCarloResult run_targeted_monte_carlo(
    const Vec6& x0_nominal,
    const Vec3& target,
    double tof,
    double n,
    const UncertaintyModel& uncertainty,
    const ApproachCorridor& corridor,
    int n_samples = 1000,
    unsigned int seed = 42
);

}

#endif 
