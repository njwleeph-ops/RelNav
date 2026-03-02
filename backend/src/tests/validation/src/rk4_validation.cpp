/**
 * @file rk4_validation.cpp
 * @brief RK4 validation 
 */

#include <iostream>

#include "rk4_validation.hpp"


// ----------------------------------------------------------------------
// Functions
// ----------------------------------------------------------------------

Vec6 initial_state(const Approaches &approach_type) {
    switch(approach_type) {
        case 0:
            return Vec6{0, -500, 0, 0, 0, 0};
        case 1:
            return Vec6{-500, 0, 0, 0, 0, 0};
        case 2:
            return Vec6{100, 0, 0, 0, -2 * N * 100, 0};
        default:
            return Vec6{0, -500, 0, 0, 0, 0};
    }
}

std::vector<ValidationData> run_convergence_validation(
    const Vec6 &x0,
    const InitialCondition &ic,
    double n,
    double duration,
    const std::vector<double> &dt_values) 
{
    std::vector<ValidationData> results;
    Mat6 stm = cw_state_transition_matrix(n, duration);
    Vec6 x_stm = stm * x0;
    double t = 0.0;

    for (double dt : dt_values) {
        Vec6 x_rk4 = x0;
        int steps = static_cast<int>(duration / dt);
        double max_pos_error = 0.0;
        double max_vel_error = 0.0;

        for (int i = 0; i < steps; ++i) {
            x_rk4 = rk4_step(x_rk4, t, n, dt);
            t += dt;
        }

        double pos_error = (x_rk4.head<3>() - x_stm.head<3>()).norm();
        double vel_error = (x_rk4.tail<3>() - x_stm.tail<3>()).norm();
        double rel_pos_error = pos_error / x_stm.head<3>().norm();

        results.push_back(
            {
                ic_to_string(ic),
                dt,
                duration,
                pos_error,
                vel_error,
                rel_pos_error,

            }
        )
    }
}