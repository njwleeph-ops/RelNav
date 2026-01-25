/**
 * @file gnc_algorithms.hpp
 * @brief GNC Algorithms for Proximity Operations - Declarations
 * 
 * Implements guidance and control algorithms:

 * - Glideslope guidance
 * - LQR optimal control
 * 
 * Reference: Fehse, "Automated Rendezvous and Docking of Spacecraft"
 */

#ifndef GNC_ALGORITHMS_HPP
#define GNC_ALGORITHMS_HPP

#include "cw_dynamics.hpp"

namespace relnav{

/**
 * @brief Compute full transfer trajectory
 * @param x0 Initial state
 * @param rf Target position
 * @param tof Time of flight [s]
 * @param n Mean motion [rad/s]
 * @param num_points Number of trajectory points
 * @return Transfer trajectory with maneuvers
 */
StateHistory compute_transfer_trajectory(
    const Vec6& x0,
    const Vec3& rf,
    double tof,
    double n,
    int num_points = 100
);

// ----------------------------------------------------------------------------
// Glideslope Guidance
// ----------------------------------------------------------------------------

/**
 * @brief Glideslope guidance parameters
 * 
 * Enforces: |v_approach| <= k * range
 */
struct GlideslopeParams {
    double k = 0.001;               // Velocity limit per unit range [1/s]
    Vec3 approach_axis{0, -1, 0};   // Approach direction (default: -v-bar)
    double corridor_angle = 0.175;  // Half-angle [rad]
    double min_range = 10.0;        // Minimum range to avoid singularity [m]
};

/**
 * @brief Result of glideslope constraint check
 */
struct GlideslopeCheck {
    bool violated;          // true if constraint violated
    double margin;          // Positive = safe, negative = violation
    double v_approach;      // Actual approach velocity [m/s]
    double v_max;           // Maximum allowed velocity [m/s]
    double approach_angle;  // Angle of approach [rad]
};

/**
 * @brief Compute maximum allowed approach velocity
 * @param position Current position in LVLH [m]
 * @param params Glideslope parameters
 * @return Maximum approach speed [m/s]
 */
double glideslope_velocity_limit(const Vec3& position, const GlideslopeParams& params);

/**
 * @brief Compute approach angle based on position relative to target
 * @param position Current position in LVLH [m]
 * @param params Glideslope parameters
 * @return Angle of approach [rad]
 */
double glideslope_approach_angle(const Vec3& position, const Vec3& axis);

/**
 * @brief Check if glideslope constraint is violated
 * @param x Current state
 * @param params Glideslope parameters
 * @return Check result with violation status and margins
 */
GlideslopeCheck check_glideslope_violation(const Vec6& x, const GlideslopeParams& params);

/**
 * @brief Compute glideslope braking command
 * @param x Current state
 * @param params Glideslope parameters
 * @return Delta-v command to satisfy constraint [m/s]
 */
Vec3 glideslope_velocity_guidance(const Vec6& x, const GlideslopeParams& params);

/**
 * @brief Computes waypoints according to position relative to approach corridor
 * @param x Current position state
 * @param params Glideslope parameters
 * @return Waypoint position [m]
 */
Vec3 glideslope_waypoint_guidance(const Vec3& x, const GlideslopeParams& params);


// ----------------------------------------------------------------------------
// LQR Optimal Control
// ----------------------------------------------------------------------------

/**
 * @brief LQR Simulation result
 */
struct LQRSimulationResult {
    StateHistory trajectory;
    std::vector<Vec3> control_history;
    double total_dv;
};

/**
 * @brief Solve continuous-time algebraic Riccati equation
 * @param A State matrix
 * @param B Control matrix
 * @param Q State cost matrix
 * @param R Control cost matrix
 * @param max_iter Maximum iterations
 * @param tol Convergence tolerance
 * @return Solution matrix P
 */
Mat6 solve_CARE(
    const Mat6& A,
    const Mat63& B,
    const Mat6& Q,
    const Eigen::Matrix3d& R,
    int max_iter = 1000,
    double tol = 1e-10
);

/**
 * @brief Compute LQR gain matrix
 * @param n Mean motion [rad/s]
 * @param Q State cost matrix
 * @param R Control cost matrix
 * @return 3x6 gain matrix K
 */
Eigen::Matrix<double, 3, 6> compute_lqr_gain(
    double n, 
    const Mat6& Q = Mat6::Identity(),
    const Eigen::Matrix3d& R = Eigen::Matrix3d::Identity()
);

/**
 * @brief Compute LQR control command
 * @param x Current state
 * @param K LQR gain matrix
 * @param x_ref Reference state (default: origin)
 * @param u_max Maximum control magnitude (negative = no limit)
 * @return Control acceleration [m/s^2]
 */
Vec3 lqr_control(
    const Vec6& x,
    const Eigen::Matrix<double, 3, 6>& K,
    const Vec6& x_ref = Vec6::Zero(),
    double u_max = -1.0
);

/**
 * @brief Simulate LQR-controlled approach
 * @param x0 Initial state
 * @param duration Simulation duration [s]
 * @param n Mean motion [rad/s]
 * @param Q State cost matrix
 * @param R Control cost matrix
 * @param u_max Maximum control acceleration [m/s^2]
 * @param dt Time step [s]
 * @return Simulation result with trajectory and control history
 */
LQRSimulationResult simulate_lqr_approach(
    const Vec6& x0,
    double duration,
    double n,
    const Mat6& Q = Mat6::Identity(),
    const Eigen::Matrix3d& R = Eigen::Matrix3d::Identity(),
    double u_max = 0.01,
    double dt = 10.0
);

}

#endif