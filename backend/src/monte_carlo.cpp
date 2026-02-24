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

// ----------------------------------------------------------------------------
// Single Sample Execution
// ----------------------------------------------------------------------------

MonteCarloSampleResult run_sample(
    const Vec6 &x0,
    double n,
    const ApproachParams &params,
    const UncertaintyModel &uncertainty,
    std::mt19937 &rng)
{
    MonteCarloSampleResult result;

    NavConfig nav = default_nav_config();

    ApproachResult approach_result = run_approach_guidance(
        x0,
        n,
        params,
        &nav,
        &rng,
        uncertainty.thrust_mag_error,
        uncertainty.thrust_pointing_error);

    result.success = approach_result.success;
    result.final_range = approach_result.final_range;
    result.final_velocity = approach_result.final_velocity;
    result.total_dv = approach_result.total_dv;
    result.duration = approach_result.duration;
    result.saturation_count = approach_result.saturation_count;
    result.min_range_seen = approach_result.min_range_seen;
    result.final_state = approach_result.trajectory.points[approach_result.num_points - 1].x;
    // TODO maybe add dropout_count and measurement_count idk lol

    if (result.final_range <= params.success_range && result.final_velocity >= params.success_velocity) {
        result.failure_reason = FailureReason::EXCESS_VELOCITY;
    } else if (result.min_range_seen > params.success_range) {
        result.failure_reason = FailureReason::POSITION_TIMEOUT;
    } else if (result.final_range > params.success_range) {
        result.failure_reason = FailureReason::LIMIT_CYCLE;
    } else {
        result.failure_reason = FailureReason::NONE;
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
                case FailureReason::LIMIT_CYCLE:
                    result.failures.limit_cycle++;
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
            MonteCarloSampleResult sample = run_sample(
                x0, n, params, uncertainty, rng
            );

            // Store result
            {
                std::lock_guard<std::mutex> lock(result_mutex);
                result.samples[sample_idx] = sample;
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

// ----------------------------------------------------------------------------
// Performance Envelope
// ----------------------------------------------------------------------------

FailureReason dominant_failure_reason(const FailureBreakdown& failures) {
    if (failures.position_timeout == 0 && failures.excess_velocity == 0) {
        return FailureReason::NONE;
    }

    int max_count = failures.position_timeout;
    FailureReason dominant = FailureReason::POSITION_TIMEOUT;

    if (failures.excess_velocity > max_count) {
        max_count = failures.excess_velocity;
        dominant = FailureReason::EXCESS_VELOCITY;
    } else if (failures.limit_cycle > max_count) {
        max_count = failures.limit_cycle;
        dominant = FailureReason::LIMIT_CYCLE;
    }

    return dominant;
}

Vec6 state_from_range(double range, const ApproachParams& params) {
    Vec6 x0 = Vec6::Zero();
    Vec3 axis = params.corridor_params.approach_axis.normalized();

    x0.head<3>() = range * axis;

    return x0;
}

EnvelopePoint run_envelope_point(
    double range, 
    double u_max,
    double n,
    const ApproachParams& params,
    const UncertaintyModel& uncertainty,
    int n_samples,
    unsigned int seed) 
{
    ApproachParams new_params = params;
    new_params.u_max = u_max;
    Vec6 x0 = state_from_range(range, new_params);

    MonteCarloResult mc = run_monte_carlo(x0, n, params, uncertainty, n_samples, 1, seed);

    // Condense into EnvelopePoint
    EnvelopePoint pt;
    pt.range = range;
    pt.u_max = u_max;
    pt.success_rate = mc.success_rate;
    pt.n_success = mc.n_success;
    pt.n_samples = mc.n_samples;
    pt.mean_dv = mc.mean_dv;
    pt.p95_dv = mc.dv_percentiles.p95;
    pt.p99_dv = mc.dv_percentiles.p99;
    pt.mean_duration = mc.mean_duration;
    pt.p95_duration = mc.duration_percentiles.p95;
    pt.failures = mc.failures;
    pt.dominant_failure = dominant_failure_reason(mc.failures);

    return pt;
}

PerformanceEnvelope compute_performance_envelope(
    const ApproachParams& params,
    const UncertaintyModel& uncertainty,
    const EnvelopeConfig& config,
    double n,
    int n_threads)
{
    auto start_time = std::chrono::steady_clock::now();

    PerformanceEnvelope envelope;
    envelope.config = config;
    envelope.params = params;
    envelope.uncertainty = uncertainty;

    Vec3 axis = params.corridor_params.approach_axis.normalized();

    if (std::abs(axis[1]) > 0.9) {
        envelope.approach_axis = "vbar";
    } else if (std::abs(axis[0]) > 0.9) {
        envelope.approach_axis = "rbar";
    } else {
        envelope.approach_axis = "hbar";
    }

    // Build axes
    envelope.range_vals.resize(config.range_steps);
    envelope.u_max_vals.resize(config.u_max_steps);

    for (int i = 0; i < config.range_steps; ++i) {
        double range_frac = static_cast<double>(i) / std::max(config.range_steps - 1, 1);
        envelope.range_vals[i] = config.range_min + range_frac * (config.range_max - config.range_min);
    }

    // Max thrust log-spaced 
    double log_min = std::log10(config.u_max_min);
    double log_max = std::log10(config.u_max_max);

    for (int j = 0; j < config.u_max_steps; ++j) {
        double u_max_frac = static_cast<double>(j) / std::max(config.u_max_steps - 1, 1);
        envelope.u_max_vals[j] = std::pow(10.0, log_min + u_max_frac * (log_max - log_min));
    }

    // Allocate grid
    envelope.grid.resize(config.range_steps);

    for (int i = 0; i < config.range_steps; ++i) {
        envelope.grid[i].resize(config.u_max_steps);
    }

    // Thread count
    if (n_threads == 0) {
        n_threads = static_cast<int>(std::thread::hardware_concurrency());

        if (n_threads == 0) {
            n_threads = 4;
        }
    }

    // Flatten grid into work queue
    int total_cells = config.range_steps * config.u_max_steps;
    std::mutex queue_mutex;
    int next_cell = 0;

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

            int i = cell_idx / config.u_max_steps;
            int j = cell_idx % config.u_max_steps;

            unsigned int cell_seed = config.seed + static_cast<unsigned int>(cell_idx * 997);

            EnvelopePoint pt = run_envelope_point(
                envelope.range_vals[i],
                envelope.u_max_vals[j],
                n,
                params,
                uncertainty,
                config.samples_per_point,
                cell_seed
            );

            {
                std::lock_guard<std::mutex> lock(queue_mutex);
                envelope.grid[i][j] = pt;
            }
        }
    };

    // Launch threads
    std::vector<std::thread> threads;

    for (int t = 0; t < n_threads; ++t) {
        threads.emplace_back(worker);
    }

    for (auto& t : threads) {
        t.join();
    }

    auto end_time = std::chrono::steady_clock::now();
    envelope.total_compute_time = std::chrono::duration<double>(end_time - start_time).count();

    return envelope;
} 
}