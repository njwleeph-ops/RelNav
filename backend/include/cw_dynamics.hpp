/**
 * @file cw_dynamics.hpp
 * @brief Implements analytical and numerical propagation for spacecraft
 * relative motion in the LVLH (Hill) frame.
 * 
 * Coordinate Frame (LVLH):
 *   x (R-bar): Radial, positive away from Earth center
 *   y (V-bar): Along-track, positive in velocity direction
 *   z (H-bar): Cross-track, completes right-hand system
 * 
 * Reference: Clohessy & Wiltshire, "Terminal Guidance System for Satellite 
 *            Rendezvous", J. Aerospace Sciences, 1960
 */

#ifndef CW_DYNAMICS_HPP
#define CW_DYNAMICS_HPP

#include <array>
#include <functional>

#include <Eigen/Dense>

namespace relnav {
// ----------------------------------------------------------------------------
// Constants
// ----------------------------------------------------------------------------

constexpr double MU_EARTH = 3.986004418e14;  // Earth gravitational parameter [m^3/s^2]
constexpr double R_EARTH = 6.371e6;          // Earth mean radius [m]
constexpr int MAX_TRAJECTORY_POINTS = 10000; // Maximum points for a trajectory computation

// ----------------------------------------------------------------------------
// Type Aliases
// ----------------------------------------------------------------------------

using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat3 = Eigen::Matrix3d;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Mat63 = Eigen::Matrix<double, 6, 3>;
using Mat36 = Eigen::Matrix<double, 3, 6>;
using StateHistory = std::vector<std::pair<double, Vec6>>;

// ----------------------------------------------------------------------------
// Structs
// ----------------------------------------------------------------------------

/**
 * @brief State description at a single point
 */
struct StatePoint {
    double t;       // Time [s]
    Vec6 x;         // State description [x, y, z, v_x, v_y, v_z]
};

/**
 * @brief Trajectory information
 */
struct Trajectory {
    std::array<StatePoint, MAX_TRAJECTORY_POINTS> points;
    int count;
};

/**
 * @brief Orital parameters for target spacecraft
 */
struct OrbitalParams {
    double altitude;    // Altitude above Earth surface [m]
    explicit OrbitalParams(double alt = 420e3);
    double radius() const;
    double mean_motion() const;
    double period() const;
};

/**
 * @brief Result of propagator validation
 */
struct ValidationResult {
    double max_pos_err;     // Maximum position error [m]
    double max_vel_err;     // Maximum velocity error [m/s]
    double max_rel_pos_err;  // Maximum relative position error 
    bool passed;            // Whether validation passed
};

// Default ISS-like orbit
extern const OrbitalParams ISS_ORBIT;

// ----------------------------------------------------------------------------
// Function Declarations
// ----------------------------------------------------------------------------

/**
 * @brief Compute CW state matrix A
 * @param n Mean motion of target orbit [rad/s]
 * @return 6x6 state matrix
 */
Mat6 cw_state_matrix(double n);

/**
 * @brief Compute CW control matrix B
 * @return 6x3 control input matrix
 */
Mat63 cw_control_matrix();

/**
 * @brief Compute analytical state transition matrix 
 * @param n Mean motion [rad/s]
 * @param t Propagation time [s]
 * @return 6x6 state transition matrix
 */
Mat6 cw_state_transition_matrix(double n, double t);

// ----------------------------------------------------------------------------
// Propagation
// ----------------------------------------------------------------------------

/**
 * @brief Propagate state analytically using state transition matrix
 * @param x0 Initial state [x, y, z, v_x, v_y, v_z]
 * @param duration Propagation duration [s]
 * @param n Mean motion [rad/s]
 * @param out Output trajectory provided by caller
 * @param num_points Number of output points
 * @return State history as vector of (time, state) pairs
 */
void propagate_analytical(
    const Vec6& x0,
    double duration,
    double n,
    Trajectory& out,
    int num_points = 100
);

/**
 * @brief Single RK4 integrator step
 * @param x Current state
 * @param t Current time [s]
 * @param dt Time step [s]
 * @param n Mean motion [rad/s]
 * @param control_func Optional control function u(t, x)
 * @return State after one step
 */
Vec6 rk4_step(
    const Vec6& x,
    double t,
    double dt,
    double n,
    const Vec3& u = Vec3::Zero()
);

// ----------------------------------------------------------------------------
// Validation
// ----------------------------------------------------------------------------

/**
 * @brief Validate numerical vs analytical propagation
 * @param x0 Initial state
 * @param duration Propagation duration [s]
 * @param n Mean motion [rad/s]
 * @param num_points Number of comparison points
 * @return Validation results with error metrics
 */
ValidationResult validate_propagators(
    const Vec6& x0,
    double duration,
    double n,
    int num_steps
);

}

#endif 