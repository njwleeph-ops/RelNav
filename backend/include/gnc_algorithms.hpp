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

/// Guidance structs
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

/// Guidance function declarations

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
// Navigation EKF Filtering
// ----------------------------------------------------------------------------

/// Navigation structs

struct SensorModel {
    double range_noise;         // [m]
    double bearing_noise;       // [rad]
    double update_rate;         // [Hz]
    double max_range;           // dropout above [m]
    double min_range;           // saturation below [m]
};

struct Measurement {
    Vec3 z;                     // [range, azimuth, elevation]
    bool valid;         
};

struct NavFilterConfig {
    Mat6 Q_process;             // process noise 
    Mat3 R_meas;                // measurement noise
    Mat6 P0;                    // initial covariance
};

struct NavConfig {
    SensorModel sensor;
    NavFilterConfig filter;
};

struct NavFilterState {
    Vec6 x_hat;
    Mat6 P;
};

struct NavDiagnostics {
    double max_pos_error;
    double final_pos_error;
    int measurement_count;
    int dropout_count;
};

/// EKF navigation function declarations

/**
 * @brief Propagate estimate and covariance
 * @param state State and covariance to propagate
 * @param u_applied Thrust applied
 * @param n Mean motion
 * @param dt Timestep size
 * @param Q_process Process noise
 * @return Propagated state estimate and covariance estimate
 */
NavFilterState ekf_predict(
    const NavFilterState& state,
    const Vec3& u_applied,
    double n,
    double dt,
    const Mat6& Q_process
);

/**
 * @brief Fuse measurement into estimate via Kalman gain
 * @param predicted Estimated state and covariance after propagation
 * @param meas Measurements [range, azimuth, elevation]
 * @param R_meas Measurement noise
 * @return Estimated state with measurement 
 */
NavFilterState ekf_update(
    const NavFilterState& predicted,
    const Measurement& meas,
    const Mat3& R_meas
);

/**
 * @brief Nonlinear measurement model h(x)
 * @param x State vector [position, velocity]
 * @return h(x) = [range, azimuth, elevation]
 */
Vec3 measurement_function(const Vec6& x);

/**
 * @brief Jacobian evaluated at x
 * @param x State vector
 */
Mat36 measurement_jacobian(const Vec6& x);

/**
 * @brief Simulate sensor reading from true state
 * @param x_true True state vector at a given point of time
 * @param sensor Sensor noises to apply to true state
 * @param rng Randomizer
 */
Measurement generate_measurement(
    const Vec6& x_true,
    const SensorModel& sensor,
    std::mt19937& rng
);

/**
 * @brief Build initial filter state from true state and initial covariance
 * @param x_true True state vector
 * @param P0 Initial covariance
 * @param rng Randomizer
 */
NavFilterState initialize_filter(
    const Vec6& x_true,
    const Mat6& P0,
    std::mt19937& rng
);

NavConfig default_nav_config();

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
// Full Approach GNC controller
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
    const ApproachParams& params,
    const NavConfig* nav = nullptr,
    std::mt19937* rng = nullptr
);

}

#endif