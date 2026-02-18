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

namespace relnav {

// ----------------------------------------------------------------------------
// Constants
// ----------------------------------------------------------------------------

constexpr int MAX_WAYPOINTS = 20;

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
double glideslope_velocity_limit(const Vec3& position, const double k, const double min_range);

/**
 * @brief Compute approach angle based on position relative to target
 * @param position Current position in LVLH [m]
 * @param params Glideslope parameters
 * @return Angle of approach [rad]
 */
double glideslope_approach_angle(const Vec3& position, const Vec3& axis);

/**
 * @brief Check if position vector is within approach corridor
 * @param position Current position of chaser in LVLH [m]
 * @param corridor_angle Half angle of approach corridor [rad]
 * @param range Radial range for approach corridor [m] 
 * @return Whether inside approach corridor or not
 */
bool is_inside_corridor(const Vec3& position, const double& corridor_angle, const Vec3& axis);

/**
 * @brief Compute waypoint on approach corridor edge if outside allowed approach angle
 * @param position Current position of chaser in LVLH [m]
 * @param corridor_angle Half angle of approach corridor [rad]
 * @return Position of intermediate waypoint
 */
Vec3 compute_edge_waypoint(const Vec3& position, const double& corridor_angle, const Vec3& axis);

/**
 * @brief Check if glideslope constraint is violated
 * @param x Current state
 * @param params Glideslope parameters
 * @return Check result with violation status and margins
 */
GlideslopeCheck check_glideslope_violation(const Vec6& x, const GlideslopeParams& params);

/**
 * @brief Clamp control vector to respect glideslope
 * @param u Thrust vector
 * @param x State vector
 * @param params Glideslope parameters
 */
Vec3 apply_glideslope_constraint(const Vec3& u, const Vec6& x, double dt, const GlideslopeParams& params);


// ----------------------------------------------------------------------------
// LQR Optimal Control
// ----------------------------------------------------------------------------

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
    const Mat3& R,
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
Mat36 compute_lqr_gain(
    double n, 
    const Mat6& Q = Mat6::Identity(),
    const Mat3& R = Mat3::Identity()
);

/**
 * @brief Compute LQR control command
 * @param x Current state
 * @param K LQR gain matrix
 * @param x_ref Reference state (default: origin)
 * @param u_max Maximum control magnitude (negative = no limit)
 * @return Control acceleration [m/s^2]
 */
Vec3 compute_lqr_control(
    const Vec6& x,
    const Mat36& K,
    const Vec6& x_ref = Vec6::Zero()
);

/**
 * @brief Saturate control magnitude
 * @param u Thrust control vector
 * @param u_max Maximum thrust magnitude allowed
 */
Vec3 saturate_control(const Vec3& u, double u_max);

// ----------------------------------------------------------------------------
// Approach Guidance
// ----------------------------------------------------------------------------

/**
 * @brief Parameters for approach manuever (if outside approach corridor angle)
 */
struct ApproachParams {
    GlideslopeParams corridor_params;
    Mat6 Q = Mat6::Identity();
    Mat3 R = Mat3::Identity();
    double u_max = 0.01;
    double dt = 1.0;
    double timeout = 6000.0;
    double success_range = 5.0;
    double success_velocity = 0.05;
};

/**
 * @brief Result of approach maneuver
 */
struct ApproachResult {
    Trajectory trajectory;
    std::vector<Vec3> control_history;
    std::vector<Vec3> waypoint_history;
    int num_points;
    double total_dv;
    double final_range;
    double final_velocity;
    double duration;
    bool success;
    int saturation_count;
};

ApproachResult run_approach_guidance(
    const Vec6& x0,
    double n,
    const ApproachParams& params
);

}

#endif