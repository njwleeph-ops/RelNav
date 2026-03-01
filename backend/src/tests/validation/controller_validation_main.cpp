/**
 * @file controller_validation_main.cpp
 * @brief Main function for controller validations exporting to csv
 */

#include "controller_validation.hpp"

// --------------------------------------------------------------------
// User Input
// --------------------------------------------------------------------

const std::string AXIS_NAME = "vbar";
const std::string FILEPATH = "../src/tests/validation/csv_data/" + AXIS_NAME + "_sweep.csv";

constexpr double Q_POS_MIN = 1.0;
constexpr double Q_POS_MAX = 500.0;
constexpr double Q_VEL_MIN = 1.0;
constexpr double Q_VEL_MAX = 500.0;
constexpr int Q_POS_STEPS = 20;
constexpr int Q_VEL_STEPS = 20;

const Axes AXIS = Axes::VBAR;

constexpr int SAMPLES_PER_POINT = 200;
constexpr unsigned int SEED = 42;

constexpr double VEL_ERROR = 0.1;
constexpr double THRUST_MAG_ERROR = 0.05;
constexpr double THRUST_POINTING_ERROR = 0.0175;
constexpr bool AXIS_LOCK = true;

constexpr double GLIDESLOPE_K = 0.001;
constexpr double CORRIDOR_ANGLE = 0.175;
const Vec3 APPROACH_AXIS = Vec3{0, -1, 0};
constexpr double U_MAX = 0.05;
constexpr double SUCCESS_RANGE = 5.0;
constexpr double MAX_RANGE = 2000.0;
constexpr double TIMEOUT = 6000.0;

constexpr int THREAD_COUNT = 4;

int main() {
    TestConfig config;
    config.Q_pos_min = Q_POS_MIN;
    config.Q_pos_max = Q_POS_MAX;
    config.Q_pos_steps = Q_POS_STEPS;
    config.Q_vel_min = Q_VEL_MIN;
    config.Q_vel_max = Q_VEL_MAX;
    config.Q_vel_steps = Q_VEL_STEPS;
    config.samples = SAMPLES_PER_POINT;
    config.seed = SEED;

    DispersionModel dispersion;
    dispersion.vel_error = VEL_ERROR;
    dispersion.thrust_mag_error = THRUST_MAG_ERROR;
    dispersion.thrust_pointing_error = THRUST_POINTING_ERROR;
    dispersion.axis_lock = AXIS_LOCK;

    DataParams params;
    params.dispersion = dispersion;
    params.corridor_params.k = GLIDESLOPE_K;
    params.corridor_params.approach_axis = APPROACH_AXIS;
    params.corridor_params.corridor_angle = CORRIDOR_ANGLE;
    params.u_max = U_MAX;
    params.max_range = MAX_RANGE;
    params.timeout = TIMEOUT;

    double n = OrbitalParams(420e3).mean_motion();

    EnvelopeData env = compute_envelope(
        config, params, THREAD_COUNT, n);

    export_envelope_csv(
        env, FILEPATH);

    return 0;
}
