/**
 * @file controller_validation.cpp
 * @brief Parameter sweep to determine optimal Q, and R values
 */

#include <mutex>
#include <thread>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include "gnc_algorithms.hpp"

using namespace relnav;

// --------------------------------------------------------------------
// Constants
// --------------------------------------------------------------------

constexpr int MAX_SAMPLES = 500;

// --------------------------------------------------------------------
// Structs
// --------------------------------------------------------------------

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

struct EnvelopeData {
    std::vector<double> Q_pos_vals;
    std::vector<double> Q_vel_vals;

    std::vector<std::vector<TestPoint>> grid;

    TestConfig config;
    ApproachParams params;
    DispersionConfig dispersion_config;

    double total_compute_time = 0.0;
};

struct DispersionModel {
    double polar_error = M_PI / 100.0;
    double azimuth_error; 
    double vel_error;
    double thrust_mag_error;
    double thrust_pointing_error;
};

struct MonteCarloSample {
    bool success;
    double final_range;
    double final_velocity;
    double total_dv;
    double duration;
    int saturation_count;
    Vec6 final_state;
};

// --------------------------------------------------------------------
// Function Definitions
// --------------------------------------------------------------------

std::string axis_to_string(const Vec3& approach_axis) {
    if (approach_axis(0) == -1) {
        return std::string{"rbar"};
    } else if (approach_axis(1) == -1) {
        return std::string{"vbar"};
    } else {
        return std::string{"hbar"};
    }
}

Vec6 dispersed_initial_state(
    double success_range,
    double max_range,
    const std::string& approach_axis,
    const DispersionModel& dispersion,
    std::mt19937& rng)
{
    std::normal_distribution<double> dist(0.0, 1.0);

    double max_range_padding_scale = 0.8;
    double min_range_adjust_scale = 1.15;

    double range = max_range * max_range_padding_scale * dist(rng);

    if (range <= success_range) {
        range += min_range_adjust_scale * success_range;
    }

    double theta = approach_axis == "hbar" ?
        (3 * M_PI / 4) + dispersion.polar_error * dist(rng) 
        : (M_PI / 2) + dispersion.polar_error * dist(rng);

    std::clamp(theta, 0, M_PI);

    if (approach_axis == "vbar") {
        double phi = (M_PI / 2) + dispersion.azimuth_error * dist(rng);
    } else if (approach_axis == "hbar") {
        double phi = (M_PI) + dispersion.azimuth_error * dist(rng);
    } else {
        double phi = (2 * M_PI) + dispersion.azimuth_error * dist(rng);
    }

    Vec3 velocities;
    for (int i = 0; i < 2; ++i) {
        velocities(i) = dispersion.vel_error * dist(rng);
        std::clamp(velocities(i), -1.0, 1.0);
    }

    Vec6 x;
    x << range * std::sin(theta) * std::cos(phi),
         range * std::sin(theta) * std::sin(phi),
         range * std::cos(theta),
         velocities(0),
         velocities(1),
         velocities(2);

    return x;    
}

MonteCarloSample run_sample(
    const ApproachParams& params,
    const Dispersion& dispersion,
    std::mt19937& rng)
{
    MonteCarloSample sample;
    NavConfig nav = default_nav_config();
    double n = OrbitalParams(420e3).mean_motiion();

    Vec6 dispersed_sample = dispersed_initial_state(
        params.success_range,
        nav.sensor.max_range,
        dispersion,
        rng
    );

    ApproachResult approach_result = run_approach_guidance(
        dispersed_sample, n, params,
        nav, &rng, dispersion.thrust_mag_error,
        dispersion.thrust_pointing_error
    );

    sample.success = approach_result.success;
    sample.final_range = approach_result.final_range;
    sample.final_velocity = approach_result.final_velocity;
    sample.total_dv = approach_result.total_dv;
    sample.duration = approach_result.duration;
    sample.saturation_count = approach_result.saturation_count;
    sample.final_state = approach_result.trajectory.points[result.num_points - 1];

    return sample;
} 

DataPoint run_samples(
    int n_samples,
    int n_threads,
    const ApproachParams& params,
    const DispersionModel& dispersion)
{
    DataPoint pt;
    pt.n_samples = std::min(n_samples, MAX_SAMPLES);
    pt.max_saturation_count = 0;
    std::vector<MonteCarloSample> samples(pt.n_samples);

    if (n_threads <= 0) {
        n_threads = std::thread::hardware_concurrency();

        if (n_threads == 0) {
            n_threads = 4;
        }
    }

    std::mutex result_mutex;
    int next_sample = 0;

    auto worker = [&](unsigned int thread_seed) {
        std::mt19937 rng(thread_seed);

        while(true) {

            {
                std::lock_guard<std::mutex> lock(result_mutex);

                if (next_sample >= pt.n_samples) {
                    break;
                }

                sample_idx = next_sample++;
            }

            Vec6 x0 = dispersed_initial_state(
                params.success_range, 
                axis_to_string(params.corridor_params.approach_axis), 
                dispersion, 
                rng);
        
            MonteCarloSample sample = run_sample(
                x0, dispersion, rng
            );

            {
                std::lock_guard<std::mutex> lock(result_mutex);
                samples[sample_idx] = sample;
            }
        }
    };

    std::vector<std::thread> threads;

    for (int i = 0; i < n_threads; ++i) {
        threads.emplace_back(worker, seed + i * 1000);
    }

    for (auto& t : threads) {
        t.join();
    }

    pt.n_success = 0;
    
    for (int i = 0; i < pt.n_samples; ++i) {
        if (samples[i].success) {
            pt.n_success++;
        }
    }

    pt.success_rate = static_cast<double>(pt.n_success) / pt.n_samples;

    double sum_dv = 0.0;
    double sum_duration = 0.0;
    double sum_saturation = 0.0;
    int max_saturation_count = 0;

    std::vector<double> dv_vals(pt.n_samples);
    std::vector<double> duration_vals(pt.n_samples);

    for (int i = 0; i < pt.n_samples; ++i) {
        dv_vals[i] = samples[i].total_dv;
        duration_vals[i] = samples[i].duration;

        sum_dv += samples[i].total_dv;
        sum_duration += samples[i].duration;
        sum_saturation += samples[i].saturation_count;
        max_saturation_count = std::max(max_saturation_count, samples[i].saturation_count);
    }

    pt.mean_dv = sum_dv / pt.n_samples;
    pt.mean_duration = sum_duration / pt.n_samples;
    pt.mean_saturation_count = sum_saturation / pt.n_samples;

    std::sort(dv_vals.begin(), dv_vals.end());
    std::sort(duration_vals.begin(), duration_vals.end());

    auto percentile = [](const std::vector<double>& vals, double p) {
        size_t n = static_cast<size_t>(vals.size());
        double idx = p * (n - 1);
        int lo = static_cast<int>(std::floor(idx));
        int hi = std::min(lo + 1, n - 1);
        double frac = idx - lo;

        return vals[lo] * (1.0 - frac) + vals[hi] * frac;
    };

    pt.p95_dv = percentile(dv_vals, 0.95);
    pt.p99_dv = percentile(dv_vals, 0.99);
    pt.p95_duration = percentile(duration_vals, 0.95);

    return pt;
}

