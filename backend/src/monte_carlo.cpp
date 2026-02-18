/**
 * @file monte_carlo.cpp
 * @brief Monte Carlo Dispersion Analysis - Implementation
 */

#include <mutex>
#include <thread>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>

#include "monte_carlo.hpp"

namespace relnav {

// ----------------------------------------------------------------------------
// Sampling Functions
// ----------------------------------------------------------------------------

Vec6 sample_initial_state(
    const Vec6& x0_nominal,
    const UncertaintyModel& uncertainty,
    std::mt19937& rng) 
{
    std::normal_distribution<double> dist(0.0, 1.0);

    Vec6 x0 = x0_nominal;
    x0(0) += uncertainty.pos_error(0) * dist(rng);
    x0(1) += uncertainty.pos_error(1) * dist(rng);
    x0(2) += uncertainty.pos_error(2) * dist(rng);
    x0(3) += uncertainty.vel_error(0) * dist(rng);
    x0(4) += uncertainty.vel_error(1) * dist(rng);
    x0(5) += uncertainty.vel_error(2) * dist(rng);

    return x0;
}

Vec3 disperse_thrust(
    const Vec3& u_nominal,
    const UncertaintyModel& uncertainty,
    std::mt19937& rng)
{
    if (u_nominal.norm() < 1e-12) {
        return u_nominal;
    }

    std::normal_distribution<double> dist(0.0, 1.0);

    // Magnitude error
    double mag_scale = 1.0 + uncertainty.thrust_mag_error * dist(rng);

    // Pointing Error - rotate around perpendicular axes
    double angle1 = uncertainty.thrust_pointing_error * dist(rng);
    double angle2 = uncertainty.thrust_pointing_error * dist(rng);

    Vec3 u_dir = u_nominal.normalized();

    // Find perpendicular axis directional vectors
    Vec3 perp1 = u_dir.cross(Vec3::UnitX());

    if (perp1.norm() < 1e-16) {
        perp1 = u_dir.cross(Vec3::UnitZ());
    }

    perp1.normalize();
    Vec3 perp2 = u_dir.cross(perp1).normalized();

    // Apply small rotations
    Vec3 u_rotated = u_dir + angle1 * perp1 + angle2 * perp2;
    u_rotated.normalized();

    return mag_scale * u_nominal.norm() * u_rotated;
}

// ----------------------------------------------------------------------------
// Single Sample Execution
// ----------------------------------------------------------------------------

MonteCarloSampleResult run_sample(
    const Vec6& x0,
    double n,
    const ApproachParams& params,
    const UncertaintyModel& uncertainty,
    std::mt19937& rng,
    Vec6& final_state)
{
    MonteCarloSampleResult result;
    result.total_dv = 0.0;
    result.success = false;
    result.saturation_count = 0;
    

    Mat36 K = compute_lqr_gain(n, params.Q, params.R);

    Vec6 x = x0;
    double t = 0.0;
    bool entered_corridor = false;

    int max_steps = static_cast<int>(params.timeout / params.dt);

    double initial_range = x0.head<3>().norm();

    // Glideslope trap: track consecutive steps where range 
    // oscillates within band
    int stall_count = 0;
    double prev_range = initial_range;
    int STALL_THRESHOLD = static_cast<int>(0.25 * max_steps);    // [steps]
    constexpr double STALL_BAND = 0.5;      // [m]

    // Track whether reached success_range
    double min_range_seen = initial_range;

    for (int i = 0; i <= max_steps; ++i) {
        double range = x.head<3>().norm();
        double velocity = x.tail<3>().norm();

        min_range_seen = std::min(min_range_seen, range);

        if (std::isnan(range) || std::isnan(velocity)) {
            std::cout << "NaN detected at i = " << i << std::endl;
            std::cout << "x = " << x.transpose() << std::endl;
            break;
        }

        if (range < params.success_range && velocity < params.success_velocity) {
            result.success = true;
            result.failure_reason = FailureReason::NONE;
            result.final_range = range;
            result.final_velocity = velocity;
            result.duration = t;
            final_state = x;
            return result;
        }

        // Glideslope trap check
        if (i > 0) {
            double range_delta = std::abs(range - prev_range);

            if (range_delta < STALL_BAND && range > params.success_range * 2.0) {
                if (i % 50 == 0) {
                }

                stall_count++;
            } else {
                stall_count = 0;
            }

            if (stall_count > STALL_THRESHOLD) {
                std::cout << "STALLED AT : " << i << std::endl;
                result.success = false;
                result.failure_reason = FailureReason::GLIDESLOPE_TRAPPED;
                result.final_range = range;
                result.final_velocity = velocity;
                result.duration = t;
                final_state = x;
                return result;
            }
        }

        prev_range = range;

        if (i == max_steps) {
            break;
        }

        bool inside = is_inside_corridor(
            x.head<3>(),
            params.corridor_params.corridor_angle,
            params.corridor_params.approach_axis
        );

        if (inside) {
            entered_corridor = true;
        }

        Vec3 waypoint = entered_corridor ?
            Vec3::Zero() :
            compute_edge_waypoint(
                x.head<3>(),
                params.corridor_params.corridor_angle,
                params.corridor_params.approach_axis
            );
        
        Vec6 x_ref = Vec6::Zero();
        x_ref.head<3>() = waypoint;

        // LQR control
        Vec3 u = compute_lqr_control(x, K, x_ref);

        // Glideslope Constraint
        u = apply_glideslope_constraint(u, x, params.dt, params.corridor_params);

        // Saturate
        Vec3 u_sat = saturate_control(u, params.u_max);

        if ((u_sat - u).norm() > 1e-10) {
            result.saturation_count++;
        }

        u = u_sat;

        // Apply thrust dispersion
        Vec3 u_dispersed = disperse_thrust(u, uncertainty, rng);

        // Accumulate delta-v
        result.total_dv += u_dispersed.norm() * params.dt;

        // RK4 step
        x = rk4_step(x, t, params.dt, n, u_dispersed);
        t += params.dt;
    }

    // Timeout
    result.final_range = x.head<3>().norm();
    result.final_velocity = x.tail<3>().norm();
    result.duration = t;
    final_state = x;

    std::cout << "min_range_seen = " << min_range_seen << " | success_range = " << params.success_range << std::endl;

    if (min_range_seen < params.success_range && result.final_velocity > params.success_velocity) {
        result.failure_reason = FailureReason::EXCESS_VELOCITY;
    } else {
        result.failure_reason = FailureReason::POSITION_TIMEOUT;
    }

    return result;
}

// ----------------------------------------------------------------------------
// Percentile Helper
// ----------------------------------------------------------------------------

static PercentileStats compute_percentiles(std::vector<double>& values) {
    PercentileStats stats;

    if (values.empty()) {
        return stats;
    }

    std::sort(values.begin(), values.end());
    int n = static_cast<int>(values.size());

    auto percentile = [&](double p) -> double {
        double idx = p * (n - 1);
        int lo = static_cast<int>(std::floor(idx));
        int hi = std::min(lo + 1, n - 1);
        double frac = idx - lo;

        return values[lo] * (1.0 - frac) + values[hi] * frac;
    };

    stats.min = values.front();
    stats.max = values.back();
    stats.p50 = percentile(0.50);
    stats.p75 = percentile(0.75);
    stats.p90 = percentile(0.90);
    stats.p95 = percentile(0.95);
    stats.p99 = percentile(0.99);

    return stats;
}

// ----------------------------------------------------------------------------
// Statistics
// ----------------------------------------------------------------------------

void compute_statistics(MonteCarloResult& result) {
    if (result.n_samples == 0) {
        return ;
    }

    // Count successes
    result.n_success = 0;
    result.failures = {};

    for (int i = 0; i < result.n_samples; ++i) {
        if (result.samples[i].success) {
            result.n_success++;
        } else {
            switch (result.samples[i].failure_reason) {
                case FailureReason::POSITION_TIMEOUT:
                    result.failures.position_timeout++;
                    break;
                case FailureReason::EXCESS_VELOCITY:
                    result.failures.excess_velocity++;
                    break;
                case FailureReason::GLIDESLOPE_TRAPPED:
                    result.failures.glideslope_trapped++;
                    break;
                default:
                    break;
            }
        }
    }

    result.success_rate = static_cast<double>(result.n_success) / result.n_samples;

    // Mean values
    double sum_dv = 0.0;
    double sum_duration = 0.0;
    double sum_range = 0.0;
    double sum_saturation = 0.0;

    for (int i = 0; i < result.n_samples; ++i) {
        sum_dv += result.samples[i].total_dv;
        sum_duration += result.samples[i].duration;
        sum_range += result.samples[i].final_range;
        sum_saturation += result.samples[i].saturation_count; 
    }

    result.mean_dv = sum_dv / result.n_samples;
    result.mean_duration = sum_duration / result.n_samples;
    result.mean_final_range = sum_range / result.n_samples;
    result.mean_saturation_count = sum_saturation / result.n_samples;

    // Standard deviation
    double sum_sq_dv = 0.0;
    double sum_sq_duration = 0.0;

    for (int i = 0; i < result.n_samples; ++i) {
        sum_sq_dv += std::pow(result.samples[i].total_dv - result.mean_dv, 2);
        sum_sq_duration += std::pow(result.samples[i].duration - result.mean_duration, 2);
    }

    if (result.n_samples > 1) {
        result.std_dv = std::sqrt(sum_sq_dv / (result.n_samples - 1));
        result.std_duration = std::sqrt(sum_sq_duration / (result.n_samples - 1));
    }

    // Percentiles
    std::vector<double> dv_vals(result.n_samples);
    std::vector<double> duration_vals(result.n_samples);

    for (int i = 0; i < result.n_samples; ++i) {
        dv_vals[i] = result.samples[i].total_dv;
        duration_vals[i] = result.samples[i].duration;
    }

    result.dv_percentiles = compute_percentiles(dv_vals);
    result.duration_percentiles = compute_percentiles(duration_vals);
}

// ----------------------------------------------------------------------------
// Monte Carlo with Multi-threading
// ----------------------------------------------------------------------------

MonteCarloResult run_monte_carlo(
    const Vec6& x0_nominal,
    double n,
    const ApproachParams& params,
    const UncertaintyModel& uncertainty,
    int n_samples,
    int n_threads,
    unsigned int seed)
{
    MonteCarloResult result;
    result.n_samples = std::min(n_samples, MAX_MC_SAMPLES);
    result.samples.resize(result.n_samples);
    result.final_states.resize(result.n_samples);

    // Auto-detect thread count
    if (n_threads <= 0) {
        n_threads = std::thread::hardware_concurrency();

        if (n_threads == 0) {
            n_threads = 4;
        }
    }

    // Mutex for writing results
    std::mutex result_mutex;
    int next_sample = 0;

    // Worker
    auto worker = [&](unsigned int thread_seed) {
        std::mt19937 rng(thread_seed);

        while(true) {
            int sample_idx;

            // Get next sample index
            {
                std::lock_guard<std::mutex> lock(result_mutex);

                if (next_sample >= result.n_samples) {
                    break;
                }

                sample_idx = next_sample++;
            }

            // Sample initial state
            Vec6 x0 = sample_initial_state(x0_nominal, uncertainty, rng);

            // Run closed-loop guidance
            Vec6 final_state;
            MonteCarloSampleResult sample = run_sample(
                x0, n, params, uncertainty, rng, final_state
            );

            // Store result
            {
                std::lock_guard<std::mutex> lock(result_mutex);
                result.samples[sample_idx] = sample;
                result.final_states[sample_idx] = final_state;
            }
        }
    };

    // Launch threads
    std::vector<std::thread> threads;

    for (int i = 0; i < n_threads; ++i) {
        threads.emplace_back(worker, seed + i * 1000);
    }

    // Wait for completion
    for (auto& t : threads) {
        t.join();
    }

    // Compute stats
    compute_statistics(result);

    return result;
}

}