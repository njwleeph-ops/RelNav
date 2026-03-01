/**
 * @file controller_validation.cpp
 * @brief Parameter sweep to determine optimal Q, and R values
 */

#include <mutex>
#include <thread>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <fstream>

#include "controller_validation.hpp"

using namespace relnav;

// --------------------------------------------------------------------
// Helpers
// --------------------------------------------------------------------

std::string axis_to_string(const Vec3 &approach_axis) {
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
    const DispersionModel &dispersion,
    std::mt19937 &rng)
{
    std::normal_distribution<double> dist(0.0, 1.0);

    double max_range_padding_scale = 0.8;
    double min_range_adjust_scale = 5.0;

    double range = max_range * max_range_padding_scale * std::clamp(dist(rng), 0.0, 1.0);

    if (range >= max_range) {
        range = 0.99 * max_range;
    }

    if (range < success_range) {
        range += min_range_adjust_scale * success_range;
    }

    double phi, theta;

    if (!dispersion.axis_lock) {
        phi = 2 * M_PI * dist(rng);
        theta = M_PI * dist(rng);
        theta = std::clamp(theta, 0.0, M_PI);
    } else {
        double val = std::clamp(std::abs(dist(rng)), 0.0, 1.0);

        double angle_base = M_PI * val;

        switch (dispersion.axis)
        {
        case Axes::RBAR:
            phi = angle_base + (M_PI / 2);
            theta = angle_base;
            break;
        case Axes::VBAR:
            phi = angle_base + M_PI;
            theta = angle_base;
            break;
        case Axes::HBAR:
            phi = 2 * angle_base;
            theta = (angle_base + M_PI) / 2;
            break;
        default:
            phi = angle_base + M_PI;
            theta = angle_base;
            break;
        }
    }

    Vec3 velocities;
    for (int i = 0; i < 3; ++i) {
        double velocity = dispersion.vel_error * dist(rng);
        velocities(i) = std::clamp(velocity, -1.0, 1.0);
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

// --------------------------------------------------------------------
// Samples
// --------------------------------------------------------------------

Sample run_sample(
    const DataParams &params,
    std::mt19937 &rng,
    double n,
    std::optional<Vec6> override_initial_state)
{
    Sample sample;
    Vec6 initial_state;

    if (override_initial_state.has_value()) {
        initial_state = *override_initial_state;
    } else {
        initial_state = dispersed_initial_state(
            params.success_range,
            params.max_range,
            params.dispersion,
            rng);
    }

    sample.initial_state = initial_state;

    ApproachResult approach_result = run_approach_guidance(
        initial_state, n, params,
        &params.nav, &rng, params.dispersion.thrust_mag_error,
        params.dispersion.thrust_pointing_error);

    sample.success = approach_result.success;
    sample.final_range = approach_result.final_range;
    sample.final_velocity = approach_result.final_velocity;
    sample.total_dv = approach_result.total_dv;
    sample.duration = approach_result.duration;
    sample.saturation_count = approach_result.saturation_count;
    sample.final_state = approach_result.trajectory.points[approach_result.num_points - 1].x;

    return sample;
}

DataPoint run_samples(
    const DataParams &params,
    double Q_pos,
    double Q_vel,
    int n_samples,
    int n_threads,
    unsigned int seed,
    double n)
{
    DataPoint pt;
    pt.Q_pos = Q_pos;
    pt.Q_vel = Q_vel;
    pt.n_samples = std::min(n_samples, MAX_SAMPLES);
    pt.max_saturation_count = 0;
    std::vector<Sample> samples(pt.n_samples);

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

        while (true) {
            int sample_idx;
            {
                std::lock_guard<std::mutex> lock(result_mutex);

                if (next_sample >= pt.n_samples)
                {
                    break;
                }

                sample_idx = next_sample++;
            }

            Sample sample = run_sample(
                params, rng, n);

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

    for (auto &t : threads) {
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

    auto percentile = [](const std::vector<double> &vals, double p) {
        size_t n = static_cast<size_t>(vals.size());
        double idx = p * (n - 1);
        int lo = static_cast<int>(std::floor(idx));
        int hi = std::min(lo + 1, static_cast<int>(n - 1));
        double frac = idx - lo;

        return vals[lo] * (1.0 - frac) + vals[hi] * frac;
    };

    pt.p95_dv = percentile(dv_vals, 0.95);
    pt.p99_dv = percentile(dv_vals, 0.99);
    pt.p95_duration = percentile(duration_vals, 0.95);

    pt.samples = std::move(samples);

    return pt;
}

EnvelopeData compute_envelope(
    const TestConfig &config,
    const DataParams &params,
    int n_threads,
    double n)
{
    auto start_time = std::chrono::steady_clock::now();

    EnvelopeData env;

    double Q_pos_log_min = std::log10(config.Q_pos_min);
    double Q_pos_log_max = std::log10(config.Q_pos_max);
    double Q_vel_log_min = std::log10(config.Q_vel_min);
    double Q_vel_log_max = std::log10(config.Q_vel_max);

    env.Q_pos_vals.resize(config.Q_pos_steps);
    env.Q_vel_vals.resize(config.Q_vel_steps);

    for (int i = 0; i < config.Q_pos_steps; ++i) {
        double Q_pos_frac = static_cast<double>(i) / std::max(config.Q_pos_steps - 1, 1);
        env.Q_pos_vals[i] = std::pow(10.0, Q_pos_log_min + Q_pos_frac * (Q_pos_log_max - Q_pos_log_min));
    }

    for (int i = 0; i < config.Q_vel_steps; ++i) {
        double Q_vel_frac = static_cast<double>(i) / std::max(config.Q_vel_steps - 1, 1);
        env.Q_vel_vals[i] = std::pow(10.0, Q_vel_log_min + Q_vel_frac * (Q_vel_log_max - Q_vel_log_min));
    }

    // Grid [Q_pos][Q_vel]
    env.grid.resize(config.Q_pos_steps);

    for (int i = 0; i < config.Q_pos_steps; ++i) {
        env.grid[i].resize(config.Q_vel_steps);
    }

    if (n_threads == 0) {
        n_threads = static_cast<int>(std::thread::hardware_concurrency());

        if (n_threads == 0) {
            n_threads = 4;
        }
    }

    int total_cells = config.Q_pos_steps * config.Q_vel_steps;
    std::mutex queue_mutex;
    int next_cell = 0;
    int completed_cells = 0;

    auto worker = [&]() {
        while (true) {
            int cell_idx;
            {
                std::lock_guard<std::mutex> lock(queue_mutex);

                if (next_cell >= total_cells) {
                    break;
                }

                cell_idx = next_cell++;
            }

            int i = cell_idx / config.Q_vel_steps;
            int j = cell_idx % config.Q_vel_steps;

            DataParams cell_params = params;
            cell_params.Q.block<3, 3>(0, 0) = Mat3::Identity() * env.Q_pos_vals[i];
            cell_params.Q.block<3, 3>(3, 3) = Mat3::Identity() * env.Q_vel_vals[j];

            unsigned int cell_seed = config.seed + static_cast<unsigned int>(cell_idx * 997);

            DataPoint pt = run_samples(
                cell_params,
                env.Q_pos_vals[i],
                env.Q_vel_vals[j],
                config.samples,
                1,
                cell_seed,
                n
            );

            {
                std::lock_guard<std::mutex> lock(queue_mutex);
                env.grid[i][j] = pt;
                completed_cells++;
                std::cout << "\r[" << completed_cells << "/" << total_cells << "]"
                          << (100.0 * completed_cells / total_cells) << "%" << std::flush;
            }
        }
    };

    std::vector<std::thread> threads;

    for (int t = 0; t < n_threads; ++t) {
        threads.emplace_back(worker);
    }

    for (auto &t : threads) {
        t.join();
        std::cout << std::endl;
    }

    auto end_time = std::chrono::steady_clock::now();
    env.total_compute_time = std::chrono::duration<double>(end_time - start_time).count();

    return env;
}

void export_envelope_csv(
    const EnvelopeData &env,
    const std::string &filepath)
{
    std::ofstream file(filepath);

    file << "Q_pos,Q_vel,"
         << "x0,y0,z0,vx0,vy0,vz0,"
         << "xf,yf,zf,vxf,vyf,vzf,"
         << "success,total_dv,duration,final_range,final_velocity,saturation_count\n";

    for (int i = 0; i < env.grid.size(); ++i) {
        for (int j = 0; j < env.grid[i].size(); ++j) {
            const DataPoint &cell = env.grid[i][j];

            for (const Sample &s : cell.samples) {
                file << cell.Q_pos << "," << cell.Q_vel << ","
                     << s.initial_state(0) << "," << s.initial_state(1) << "," << s.initial_state(2) << ","
                     << s.initial_state(3) << "," << s.initial_state(4) << "," << s.initial_state(5) << ","
                     << s.final_state(0) << "," << s.final_state(1) << "," << s.final_state(2) << ","
                     << s.final_state(3) << "," << s.final_state(4) << "," << s.final_state(5) << ","
                     << s.success << "," << s.total_dv << "," << s.duration << ","
                     << s.final_range << "," << s.final_velocity << "," << s.saturation_count << "\n";
            }
        }
    }

    file.close();
}
