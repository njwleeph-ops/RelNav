/**
 * @file monte_carlo.cpp
 * @brief Monte Carlo Dispersion Analysis - Implementation
 */

#include <algorithm>
#include <numeric>
#include <cmath>

#include "monte_carlo.hpp"

namespace relnav {

// ----------------------------------------------------------------------------
// UncertaintyModel Implementation
// ----------------------------------------------------------------------------

UncertaintyModel::UncertaintyModel(const Vec3& pos_err, const Vec3& vel_err)
    : pos_error(pos_err), vel_error(vel_err) {}

// ----------------------------------------------------------------------------
// ApproachCorridor Implementation
// ----------------------------------------------------------------------------

ApproachCorridor::ApproachCorridor(Type t, double rad, double cone_angle, const Vec3& box_half)
    : type(t), radius(rad), cone_half_angle(cone_angle), box_half_width(box_half) {}

bool ApproachCorridor::is_inside(const Vec3& position) const {
    switch (type) {
        case Type::Cylinder:{
            // Cylindrical corridor along v-bar (y-axis)
            double radial_dist = std::sqrt(position(0) * position(0) +
                                           position(2) * position(2));
            return radial_dist < radius;
        }
        case Type::Cone: {
            double range = position.norm();
            double radial_dist = std::sqrt(position(0) * position(0) +
                                           position(2) * position(2));
            double allowed_radius = range * std::tan(cone_half_angle);
            return radial_dist < allowed_radius;
        }
        case Type::Box: {
            return std::abs(position(0)) < box_half_width(0) &&
                   std::abs(position(1)) < box_half_width(1) &&
                   std::abs(position(2)) < box_half_width(2);
        }
    }

    return false;
}

// ----------------------------------------------------------------------------
// Sampling
// ----------------------------------------------------------------------------

std::vector<Vec6> sample_initial_states(
    const Vec6& x0_nominal,
    const UncertaintyModel& uncertainty,
    int n_samples,
    unsigned int seed)
{
    std::mt19937 rng(seed);
    std::vector<Vec6> samples(n_samples);

    // Create normal distribution for each component
    std::normal_distribution<double> pos_dist_x(0, uncertainty.pos_error(0));
    std::normal_distribution<double> pos_dist_y(0, uncertainty.pos_error(1));
    std::normal_distribution<double> pos_dist_z(0, uncertainty.pos_error(2));
    std::normal_distribution<double> vel_dist_x(0, uncertainty.vel_error(0));
    std::normal_distribution<double> vel_dist_y(0, uncertainty.vel_error(1));
    std::normal_distribution<double> vel_dist_z(0, uncertainty.vel_error(2));

    for (int i = 0; i < n_samples; ++i) {
        samples[i] = x0_nominal;
        samples[i](0) += pos_dist_x(rng);
        samples[i](1) += pos_dist_y(rng);
        samples[i](2) += pos_dist_z(rng);
        samples[i](3) += vel_dist_x(rng);
        samples[i](4) += vel_dist_y(rng);
        samples[i](5) += vel_dist_z(rng);
    }

    return samples;
}

// ----------------------------------------------------------------------------
// Covariance Propagation
// ----------------------------------------------------------------------------

Mat6 propagate_covariance_analytical(const Mat6& P0, double n, double t) {
    Mat6 Phi = cw_state_transition_matrix(n, t);
    return Phi * P0 * Phi.transpose();
}

// ----------------------------------------------------------------------------
// Monte Carlo Analysis
// ----------------------------------------------------------------------------

MonteCarloResult run_monte_carlo(
    const Vec6& x0_nominal,
    double duration,
    double n,
    const UncertaintyModel& uncertainty,
    const ApproachCorridor& corridor,
    int n_samples,
    int num_times,
    unsigned int seed)
{
    MonteCarloResult result;
    result.n_samples = n_samples;

    // Sample initial states
    auto x0_samples = sample_initial_states(x0_nominal, uncertainty, n_samples, seed);

    // Initialize storage
    double dt = duration / (num_times - 1);
    result.times.resize(num_times);
    result.ensemble.resize(n_samples);
    result.mean.resize(num_times, Vec6::Zero());
    result.covariance.resize(num_times, Mat6::Zero());
    result.sigma3_pos.resize(num_times);
    result.corridor_compliance.resize(num_times, 0);
    result.analytical_cov.resize(num_times);

    for (int i = 0; i < num_times; ++i) {
        result.times[i] = i * dt;
    }

    // Propagate each sample
    for (int s = 0; s < n_samples; ++s) {
        auto history = propagate_analytical(x0_samples[s], duration, n, num_times);
        result.ensemble[s].resize(num_times);

        for (int i = 0; i < num_times; ++i) {
            result.ensemble[s][i] = history[i].second;
        }
    }

    // Compute statistics at each time step
    for (int i = 0; i < num_times; ++i) {
        // Mean 
        for (int s = 0; s < n_samples; ++s) {
            result.mean[i] += result.ensemble[s][i];
        }
        
        result.mean[i] /= n_samples;

        // Covariance
        for (int s = 0; s < n_samples; ++s) {
            Vec6 diff = result.ensemble[s][i] - result.mean[i];
            result.covariance[i] += diff * diff.transpose();
        }
        result.covariance[i] /= (n_samples - 1);

        // 3 standard deviation position bounds
        result.sigma3_pos[i] = Vec3(
            3.0 * std::sqrt(result.covariance[i](0, 0)),
            3.0 * std::sqrt(result.covariance[i](1, 1)),
            3.0 * std::sqrt(result.covariance[i](2, 2))
        );

        // Corridor compliance
        int inside_count = 0;

        for (int s = 0; s < n_samples; ++s) {
            if (corridor.is_inside(result.ensemble[s][i].head<3>())) {
                inside_count++;
            }
        }

        result.corridor_compliance[i] = static_cast<double>(inside_count) / n_samples;
    }

    // Build initial covariance matrix from uncertainty model
    Mat6 P0 = Mat6::Zero();
    P0(0, 0) = uncertainty.pos_error(0) * uncertainty.pos_error(0);
    P0(1, 1) = uncertainty.pos_error(1) * uncertainty.pos_error(1);
    P0(2, 2) = uncertainty.pos_error(2) * uncertainty.pos_error(2);
    P0(3, 3) = uncertainty.vel_error(0) * uncertainty.vel_error(0);
    P0(4, 4) = uncertainty.vel_error(1) * uncertainty.vel_error(1);
    P0(5, 5) = uncertainty.vel_error(2) * uncertainty.vel_error(2);

    // Compute analytical covariance and compare to MC
    std::vector<double> cov_errors;

    for (int i = 0; i < num_times; ++i) {
        result.analytical_cov[i] = propagate_covariance_analytical(P0, n, result.times[i]);

        // Compare 3-sigma bounds for position only
        Vec3 mc3sig = result.sigma3_pos[i];
        Vec3 ana3sig(
            3.0 * std::sqrt(result.analytical_cov[i](0, 0)),
            3.0 * std::sqrt(result.analytical_cov[i](1, 1)),
            3.0 * std::sqrt(result.analytical_cov[i](2, 2))
        );

        for (int j = 0; j < 3; ++j) {
            if (ana3sig(j) > 1e-10) {
                double err = std::abs(mc3sig(j) - ana3sig(j)) / ana3sig(j) * 100;
                cov_errors.push_back(err);
            }
        }
    }

    // Compute error stats
    if (!cov_errors.empty()) {
        result.max_cov_error = *std::max_element(cov_errors.begin(), cov_errors.end());
        result.mean_cov_error = std::accumulate(cov_errors.begin(), cov_errors.end(), 0.0) / cov_errors.size();
    }

    return result;
}

TargetedMonteCarloResult run_targeted_monte_carlo(
    const Vec6& x0_nominal,
    const Vec3& target,
    double tof,
    double n,
    const UncertaintyModel& uncertainty,
    const ApproachCorridor& corridor,
    int n_samples,
    unsigned int seed)
{
    TargetedMonteCarloResult result;
    result.target = target;
    result.n_samples = n_samples;

    // 1. Compute nominal maneuver
    Vec3 r0 = x0_nominal.head<3>();
    Vec3 v0 = x0_nominal.tail<3>();
    result.nominal_maneuver = two_impulse_targeting(r0, v0, target, tof, n);

    // 2. Sample initial states with nav uncertainty
    auto x0_samples = sample_initial_states(x0_nominal, uncertainty, n_samples, seed);

    // 3. Set up thrust error distribution
    std::mt19937 rng(seed + 1);
    std::normal_distribution<double> thrust_mag_error(0, uncertainty.thrust_mag_error);
    std::normal_distribution<double> thrust_pointing(0, uncertainty.thrust_pointing_error);

    result.final_states.resize(n_samples);
    result.inside_corridor.resize(n_samples);

    // 4. Propagate each sample with dispersed maneuver
    for (int i = 0; i < n_samples; ++i) {
        Vec6 x = x0_samples[i];

        // Apply dv1 errors
        double mag_scale = 1.0 + thrust_mag_error(rng);
        double point_err_x = thrust_pointing(rng);
        double point_err_z = thrust_pointing(rng);

        Vec3 dv1_actual = result.nominal_maneuver.dv1 * mag_scale;
        dv1_actual(0) += point_err_x * dv1_actual.norm();
        dv1_actual(2) += point_err_z * dv1_actual.norm();

        x.tail<3>() += dv1_actual;

        // Propagate to arrival
        Mat6 Phi = cw_state_transition_matrix(n, tof);
        Vec6 x_final = Phi * x;

        result.final_states[i] = x_final;
        result.inside_corridor[i] = corridor.is_inside(x_final.head<3>());
    }

    // 5. Compute stats
    Vec3 sum_pos = Vec3::Zero();
    
    for (int i = 0; i < n_samples; ++i) {
        sum_pos += result.final_states[i].head<3>();
    }
    
    result.mean_arrival = sum_pos / n_samples;

    Vec3 sum_sq = Vec3::Zero();

    for (int i = 0; i < n_samples; ++i) {
        Vec3 diff = result.final_states[i].head<3>() - result.mean_arrival;
        sum_sq += Vec3(diff(0) * diff(0), diff(1) * diff(1), diff(2) * diff(2));
    }

    Vec3 variance = sum_sq / (n_samples - 1);
    result.sigma3_arrival = Vec3(
        3.0 * std::sqrt(variance(0)),
        3.0 * std::sqrt(variance(1)),
        3.0 * std::sqrt(variance(2))
    );
    
    int inside_count = std::count(result.inside_corridor.begin(),
                                  result.inside_corridor.end(), true);
    result.corridor_success_rate = static_cast<double>(inside_count) / n_samples;

    return result;
}

}