/**
 * @file cw_dynamics.cpp
 * @brief Clohessy-Wiltshire Relative Motion Dynamics - Implementation
 */

#include <cmath>
#include <algorithm>

#include "cw_dynamics.hpp"

namespace relnav {
// ----------------------------------------------------------------------------
// Global Constants
// ----------------------------------------------------------------------------

const OrbitalParams ISS_ORBIT(420e3);

// ----------------------------------------------------------------------------
// OrbitalParams Implementation
// ----------------------------------------------------------------------------


OrbitalParams::OrbitalParams(double alt) : altitude(alt) {}


double OrbitalParams::radius() const {
    return R_EARTH + altitude;
}


double OrbitalParams::mean_motion() const {
    return std::sqrt(MU_EARTH / std::pow(radius(), 3));
}


double OrbitalParams::period() const {
    return 2.0 * M_PI / mean_motion();
}

// ----------------------------------------------------------------------------
// CW Matrices
// ----------------------------------------------------------------------------

Mat6 cw_state_matrix(double n) {
    Mat6 A = Mat6::Zero();

    // Position derivatives = velocity
    A(0, 3) = 1.0;
    A(1, 4) = 1.0;
    A(2, 5) = 1.0;

    // Velocity derivatives
    A(3, 0) = 3.0 * n * n;
    A(3, 4) = 2.0 * n;
    A(4, 3) = -2.0 * n;
    A(5, 2) = -n * n;

    return A;
}

Mat63 cw_control_matrix() {
    Mat63 B = Mat63::Zero();
    B(3, 0) = 1.0;
    B(4, 1) = 1.0;
    B(5, 2) = 1.0;
    return B;
}

Mat6 cw_state_transition_matrix(double n, double t) {
    double nt = n * t;
    double c = std::cos(nt);
    double s = std::sin(nt);

    Mat6 Phi = Mat6::Zero();

    Phi(0, 0) = 4.0 - 3.0*c;
    Phi(1, 0) = 6.0 * (s - nt);
    Phi(1, 1) = 1.0;
    Phi(2, 2) = c;

    Phi(0, 3) = s / n;
    Phi(0, 4) = 2.0 * (1.0 - c) / n;
    Phi(1, 3) = 2.0 * (c - 1.0) / n;
    Phi(1, 4) = (4.0 * s - 3.0 * nt) / n;
    Phi(2, 5) = s / n;

    Phi(3, 0) = 3.0 * n * s;
    Phi(4, 0) = 6.0 * n * (c - 1.0);
    Phi(5, 2) = -n * s;

    Phi(3, 3) = c;
    Phi(3, 4) = 2.0 * s;
    Phi(4, 3) = -2.0 * s;
    Phi(4, 4) = 4.0 * c - 3.0;
    Phi(5, 5) = c;

    return Phi;
}

// ----------------------------------------------------------------------------
// Propagation
// ----------------------------------------------------------------------------

void propagate_analytical(
    const Vec6& x0,
    double duration,
    double n,
    Trajectory& out,
    int num_points)
{
    num_points = std::min(num_points, MAX_TRAJECTORY_POINTS);
    out.count = num_points;
        
    double dt = duration / (num_points - 1);

    for (int i = 0; i < num_points; ++i) {
        double t = i * dt;
        out.points[i].t = t;
        out.points[i].x = cw_state_transition_matrix(n, t) * x0;
    }
}

Vec6 rk4_step(
    const Vec6& x,
    double t,
    double dt,
    double n,
    const Vec3& u)
{
    Mat6 A = cw_state_matrix(n);
    Mat63 B = cw_control_matrix();

    auto derivatives = [&](const Vec6& xi) -> Vec6 {
        return A * xi + B * u;
    };

    Vec6 k1 = derivatives(x);
    Vec6 k2 = derivatives(x + dt / 2 * k1);
    Vec6 k3 = derivatives(x + dt / 2 * k2);
    Vec6 k4 = derivatives(x + dt * k3);

    return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}


// ----------------------------------------------------------------------------
// Validation
// ----------------------------------------------------------------------------
    
ValidationResult validate_propagators(
    const Vec6& x0,
    double duration,
    double n,
    int num_steps)
{
    ValidationResult result{0.0, 0.0, 0.0, true};

    double dt = duration / num_steps;
    Vec6 x_rk4 = x0;
    double t = 0.0;

    for (int i = 0; i <= num_steps; ++i) {
        Vec6 x_stm = cw_state_transition_matrix(n, t) * x0;

        Vec3 pos_err = x_stm.head<3>() - x_rk4.head<3>();
        Vec3 vel_err = x_stm.tail<3>() - x_rk4.tail<3>();

        result.max_pos_err = std::max(result.max_pos_err, pos_err.norm());
        result.max_vel_err = std::max(result.max_vel_err, vel_err.norm());

        double pos_mag = x_stm.head<3>().norm();

        if (pos_mag > 1e-10) {
            result.max_rel_pos_err = std::max(
                result.max_rel_pos_err,
                pos_err.norm() / pos_mag
            );
        }

        if (i < num_steps) {
            x_rk4 = rk4_step(x_rk4, t, dt, n);
            t += dt;
        }
    }

    result.passed = result.max_pos_err < 1e-7;
    return result;
}

}