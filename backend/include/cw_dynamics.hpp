/**
 * Implements analytical and numerical propagation for spacecraft
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

#include <vector>
#include <functional>

#include <Eigen/Dense>

namespace relnav {

// ----------------------------------------------------------------------------
// Constants
// ----------------------------------------------------------------------------

constexpr double MU_EARTH = 3.986004418e14;  // Earth gravitational parameter [m^3/s^2]
constexpr double R_EARTH = 6.371e6;          // Earth mean radius [m]

// ----------------------------------------------------------------------------
// Type Aliases
// ----------------------------------------------------------------------------

using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Mat63 = Eigen::Matrix<double, 6, 3>;
using StateHistory = std::vector<std::pair<double, Vec6>>;
using ControlFunc = std::function<Vec3(double, const Vec6&)>;

// ----------------------------------------------------------------------------
// Structs
// ----------------------------------------------------------------------------

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
    double max_pos_error;     // Maximum position error [m]
    double max_vel_error;     // Maximum velocity error [m/s]
    double max_rel_pos_error;  // Maximum relative position error 
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

/**
 * @brief Propagate state analytically using state transition matrix
 * @param x0 Initial state [x, y, z, v_x, v_y, v_z]
 * @param duration Propagation duration [s]
 * @param n Mean motion [rad/s]
 * @param num_points Number of output points
 * @return State history as vector of (time, state) pairs
 */
StateHistory propagate_analytical(
    const Vec6& x0,
    double duration,
    double n,
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
    const ControlFunc& control_func = nullptr
);

/**
 * @brief Propagate state numerically using RK4
 * @param x0 initial state
 * @param duration Propagation duration [s]
 * @param n Mean motion [rad/s]
 * @param num_points Number of output points
 * @param control_func Optional control function u(t, x)
 * @return State history
 */
StateHistory propagate_numerical(
    const Vec6& x0,
    double duration,
    double n,
    int num_popints = 100,
    const ControlFunc& control_func = nullptr
);

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
    int num_points = 100
);

}

#endif 