/**
 * @file rk4_validation.hpp
 * @brief Validate RK4 stepping method with CW equations (Declarations)
 */

#include "cw_dynamics.hpp"

using namespace relnav;

// ----------------------------------------------------------------------
// Constants
// ----------------------------------------------------------------------

constexpr double N = OrbitalParams(420e3).mean_motion();

// ----------------------------------------------------------------------
// Objects
// ----------------------------------------------------------------------

enum class InitialCondition {
    VBAR,
    RBAR,
    FOOTBALL
};

struct ValidationData {
    InitialCondition ic;
    double dt;
    double duration;
    double n_periods = 0.0;
    double pos_error;
    double vel_erro;
    double rel_pos_error;
    double max_pos_error;
    double max_vel_error;
};

// ----------------------------------------------------------------------
// Function Declarations
// ----------------------------------------------------------------------

Vec6 initial_state(const Approaches &approach_type);

std::vector<ValidationData> run_convergence_validation(
    const Vec6& x0,
    const InitialCondition &ic,
    double n,
    double duration,
    const std::vector<double> &dt_values
);

std::vector<ValidationData> run_duration_sweep(
    const Vec6& x0,
    const InitialCondition &ic,
    double n,
    double dt,
    const std::vector<double> &n_periods_values
);

void export_csv(
    const std::vector<ValidationData> &results,
    const std::string &filepath
);


