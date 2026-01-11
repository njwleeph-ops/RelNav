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

StateHistory propagate_analytical(
    const Vec6& x0,
    double duration,
    double n,
    int num_points)
{
    StateHistory history;
    history.reserve(num_points);
        
    double dt = duration / (num_points - 1);

    for (int i = 0; i < num_points; ++i) {
        double t = i * dt;
         Mat6 Phi = cw_state_transition_matrix(n, t);
        Vec6 x = Phi * x0;
        history.emplace_back(t, x);
    }

    return history;
}

Vec6 rk4_step(
    const Vec6& x,
    double t,
    double dt,
    double n,
    const ControlFunc& control_func)
{
    Mat6 A = cw_state_matrix(n);
    Mat63 B = cw_control_matrix();

    auto derivatives = [&](double ti, const Vec6& xi) -> Vec6 {
        Vec6 x_dot = A * xi;

        if (control_func) {
            Vec3 u = control_func(ti, xi);
            x_dot += B * u;
        }

        return x_dot;
    };

    Vec6 k1 = derivatives(t, x);
    Vec6 k2 = derivatives(t + dt / 2, x + dt / 2 * k1);
    Vec6 k3 = derivatives(t + dt / 2, x + dt / 2 * k2);
    Vec6 k4 = derivatives(t + dt, x + dt * k3);

    return x + dt / 6.0 * (k1 + 2*k2 + 2*k3 + k4);
}

StateHistory propagate_numerical(
    const Vec6& x0,
    double duration,
    double n,
    int num_points,
    const ControlFunc& control_func) 
{
    StateHistory history;
    history.reserve(num_points);

    double dt = duration / (num_points - 1);
    Vec6 x = x0;

    for (int i = 0; i < num_points; ++i) {
        double t = i * dt;
        history.emplace_back(t, x);

        if (i < num_points - 1) {
            int substeps = 100;
            double subdt = dt / substeps;

            for (int j = 0; j < substeps; ++j) {
                x = rk4_step(x, t + j*subdt, subdt, n, control_func);
            }
        }
    }

    return history;
}

// ----------------------------------------------------------------------------
// Validation
// ----------------------------------------------------------------------------
    
ValidationResult validate_propagators(
    const Vec6& x0,
    double duration,
    double n,
    int num_points)
{
    auto analytical = propagate_analytical(x0, duration, n, num_points);
    auto numerical = propagate_numerical(x0, duration, n, num_points);

    ValidationResult result{0, 0, 0, true};

    for (size_t i = 0; i < analytical.size(); ++i) {
        Vec3 pos_err = analytical[i].second.head<3>() - numerical[i].second.head<3>();
        Vec3 vel_err = analytical[i].second.tail<3>() - numerical[i].second.tail<3>();

        double pos_err_mag = pos_err.norm();
        double vel_err_mag = vel_err.norm();
        double pos_mag = analytical[i].second.head<3>().norm();

        result.max_pos_error = std::max(result.max_pos_error, pos_err_mag);
        result.max_vel_error = std::max(result.max_vel_error, vel_err_mag);

        if (pos_mag > 1e-10) {
            result.max_rel_pos_error = std::max(result.max_rel_pos_error, pos_err_mag / pos_mag);
        }
    }

    // Pass if error < 0.1 micrometer
    result.passed = result.max_pos_error < 1e-7;
    return result;
}

}