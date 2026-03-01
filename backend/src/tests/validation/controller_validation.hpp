/**
 * @file controller_validation.hpp
 * @brief Declarations for controller_validation because I am too lazy to deal with linker
 */

#ifndef CONTROLLER_VALIDATION_HPP
#define CONTROLLER_VALIDATION_HPP

#include <string>
#include <vector>
#include <optional>

#include "gnc_algorithms.hpp"

using namespace relnav;

// --------------------------------------------------------------------
// Constants
// --------------------------------------------------------------------

constexpr int MAX_SAMPLES = 500;

// --------------------------------------------------------------------
// Data Objects
// --------------------------------------------------------------------

enum class Axes {
    RBAR,
    VBAR,
    HBAR
};

struct TestConfig {
    double Q_pos_min;
    double Q_pos_max;
    int Q_pos_steps;

    double Q_vel_min;
    double Q_vel_max;
    int Q_vel_steps;

    int samples = 200;
    unsigned int seed = 42;
};

struct DispersionModel {
    double vel_error = 0.1;
    double thrust_mag_error = 0.05;
    double thrust_pointing_error = 0.0175;
    Axes axis = Axes::VBAR;
    bool axis_lock = true;
};

struct DataParams : ApproachParams {
    double max_range = 1000.0;
    DispersionModel dispersion;
    NavConfig nav = default_nav_config();
};

struct Sample {
    bool success;
    double final_range;
    double final_velocity;
    double total_dv;
    double duration;
    int saturation_count;
    Vec6 final_state;
    Vec6 initial_state;
};

struct DataPoint {
    double Q_pos;
    double Q_vel;

    double success_rate;
    int n_success;
    int n_samples;

    double mean_dv;
    double p95_dv;
    double p99_dv;

    double mean_duration;
    double p95_duration;

    double mean_saturation_count;
    int max_saturation_count;

    std::vector<Sample> samples;
};

struct EnvelopeData {
    std::vector<double> Q_pos_vals;
    std::vector<double> Q_vel_vals;

    std::vector<std::vector<DataPoint>> grid;

    TestConfig config;
    DataParams params;

    double total_compute_time = 0.0;
};

// --------------------------------------------------------------------
// Helpers
// --------------------------------------------------------------------

std::string axis_to_string(const Vec3& approach_axis);

/**
 * @brief Generate a randomly placed initial state along approach axis
 * @param success_range Minimum range, defined by being within approach corridor
 * @param max_range Sensor dropout range
 * @param approach_axis Approach axis name "vbar", "hbar", or "rbar"
 * @param dispersion Dispersion model with errors to disperse
 * @param rng Random number generator 
 * @return Initial state to pass to run_sample
 */
Vec6  dispersed_initial_state(
    double success_range,
    double max_range,
    const DispersionModel& dispersion,
    std::mt19937& rng
);

// --------------------------------------------------------------------
// Sample
// --------------------------------------------------------------------

Sample run_sample(
    const DataParams &params,
    std::mt19937 &rng,
    double n,
    std::optional<Vec6> override_initial_state = std::nullopt
);

DataPoint run_samples(
    const DataParams &params,
    double Q_pos,
    double Q_vel,
    int n_samples,
    int n_threads,
    unsigned int seed,
    double n
);

EnvelopeData compute_envelope(
    const TestConfig& config,
    const DataParams& params,
    int n_threads,
    double n
);

void export_envelope_csv(
    const EnvelopeData& env,
    const std::string& filepath
);

#endif