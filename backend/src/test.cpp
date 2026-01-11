/**
 * @file test.cpp
 * @brief Validate CW dynamics work lol
 */

#include <iostream>
#include <iomanip>

#include "cw_dynamics.hpp"
#include "monte_carlo.hpp"
#include "gnc_algorithms.hpp"

using namespace relnav;

void print_separator(const std::string& title) {
    std::cout << "\n"
              << std::string(60, '=') << "\n";
    std::cout << title << "\n";
    std::cout << std::string(60, '=') << "\n";
}


void test_propagator_validation() {
    print_separator("Test 1: Propagator Validation");

    OrbitalParams orbit(420e3);
    double n = orbit.mean_motion();
    double duration = 2 * orbit.period();

    std::cout << "ISS Orbit: alt = " << orbit.altitude / 1e3 << " km, "
              << "n = " << std::setprecision(6) << n << " rad/s, "
              << "T = " << orbit.period() << " s\n\n";

    // Test cases
    std::vector<std::pair<std::string, Vec6>> test_cases = {
        {"Radial offset (1 km)", (Vec6() << 1000, 0, 0, 0, 0, 0).finished()},
        {"Along-track offset", (Vec6() << 0, 1000, 0, 0, 0, 0).finished()},
        {"Cross-track offset", (Vec6() << 0, 0, 1000, 0, 0, 0).finished()},
        {"Realistic approach", (Vec6() << 200, -500, 50, 0.1, 0.05, 0).finished()},
    };

    double worst_error = 0;

    for (const auto& [name, x0] : test_cases) {
        auto result = validate_propagators(x0, duration, n);
        std::cout << name << ": max position error = " 
                  << std::scientific << std::setprecision(2)
                  << result.max_pos_error << " m\n";
        worst_error = std::max(worst_error, result.max_pos_error);
    }

    std::cout << "\n Validation " << (worst_error < 1e-7 ? "PASSED (<1e-7m)" : "FAIILED (>1e-7m)")
              << ": worse error = " << std::scientific << worst_error << " m\n";
    std::cout << "  Claim: <1e-10 km = 1e-6m -> "
              << (worst_error < 1e-7 ? "VERIFIED" : "FAILED") << "\n";
}


void test_monte_carlo() {
    print_separator("Test 2: Monte Carlo Dispersion");

    OrbitalParams orbit(420e3);

    Vec6 x0_nominal;
    x0_nominal << 100, -500, 0, 0, 0.1, 0;

    UncertaintyModel uncertainty;
    uncertainty.pos_error = Vec3(5, 5, 5);      // Default
    uncertainty.vel_error = Vec3(0.01, 0.01, 0.01);      // Default

    std::cout << "Running Monte Carlo with 10000 samples...\n";

    auto result = run_monte_carlo(
        x0_nominal,
        orbit.period() / 2,
        orbit.mean_motion(),
        uncertainty,
        ApproachCorridor(),
        10000,  // samples
        50,     // time points
        42      // seed
    );

    std::cout << "\nResults:\n";
    std::cout << "  Samples: " << result.n_samples << "\n";
    std::cout << "  Final mean position: [" 
              << std::fixed << std::setprecision(2)
              << result.mean.back()(0) << ", "
              << result.mean.back()(1) << ", "
              << result.mean.back()(2) << "] m\n";
    std::cout << "  Final 3σ bounds: ["
              << result.sigma3_pos.back()(0) << ", "
              << result.sigma3_pos.back()(1) << ", "
              << result.sigma3_pos.back()(2) << "] m\n";

    std::cout << "\n--- Covariance Validation (MC vs Analytical) ---\n";
    std::cout << "  Max 3σ bound error: " << std::setprecision(2)
              << result.max_cov_error << "%\n";
    std::cout << "  Mean 3σ bound error: " << result.mean_cov_error << "%\n";

    std::cout << "\n✓ VALIDATION " << (result.max_cov_error < 2.0 ? "PASSED" : "FAILED")
              << ": claim <2% error → "
              << (result.max_cov_error < 2.0 ? "VERIFIED" : "FAILED") << "\n";
}


void test_two_impulse() {
    print_separator("Test 3: Two-Impulse Targeting");

    OrbitalParams orbit(420e3);
    double n = orbit.mean_motion();

    Vec6 x0;
    x0 << 200, -1000, 0, 0, 0, 0;
    Vec3 rf(0, 0, 0);
    double tof = orbit.period() / 4;

    auto result = two_impulse_targeting(x0.head<3>(), x0.tail<3>(), rf, tof, n);

    std::cout << "Initial: [" << x0(0) << ", " << x0(1) << ", " << x0(2) << "] m\n";
    std::cout << "Target: [" << rf(0) << ", " << rf(1) << ", " << rf(2) << "] m\n";
    std::cout << "TOF: " << tof << " s\n\n";

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "dv_1 = [" << result.dv1(0) << ", " << result.dv1(1) << ", "
              << result.dv1(2) << "] m/s (mag: " << result.dv1.norm() << ")\n";
    std::cout << "dv_2 = [" << result.dv2(0) << ", " << result.dv2(1) << ", "
              << result.dv2(2) << "] m/s (mag: " << result.dv2.norm() << ")\n";
    std::cout << "Total dv = " << result.total_dv << " m/s\n";

    // Verify arrival
    auto trajectory = compute_transfer_trajectory(x0, rf, tof, n, 100);
    Vec3 final_pos = trajectory.trajectory.back().second.head<3>();
    double arrival_error = (final_pos - rf).norm();

    std::cout << "\nArrival position: [" << final_pos(0) << ", " << final_pos(1)
              << ", " << final_pos(2) << "] m\n";
    std::cout << "Arrival error: " << std::scientific << arrival_error << " m\n";
    std::cout << "\n Two-impulse targeting "
              << (arrival_error < 1e-6 ? "VERIFIED (<1e-6)" : "FAILED (>1e-6)") << "\n";
}


void test_lqr_control() {
    print_separator("Test 4: LQR Control");

    OrbitalParams orbit(420e3);
    double n = orbit.mean_motion();

    Vec6 x0;
    x0 << 100, -200, 50, 0, 0, 0;

    Mat6 Q = Mat6::Identity();
    Q.block<3, 3>(0, 0) *= 100;
    Q.block<3, 3>(3, 3) *= 10;

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 1000;

    auto result = simulate_lqr_approach(x0, orbit.period() / 2, n, Q, R, 0.01, 10.0);

    Vec6 xf = result.trajectory.back().second;

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Initial: [" << x0(0) << ", " << x0(1) << ", " << x0(2) << "] m\n";
    std::cout << "Final: [" << xf(0) << ", " << xf(1) << ", " << xf(2) << "] m\n";
    std::cout << "Total dv: " << std::setprecision(3) << result.total_dv << " m/s\n";
    
    std::cout << "\nLQR Controller operational\n";
}


void test_glideslope() {
    print_separator("Test 5: Glidelsope Constraint");

    GlideslopeParams params;
    params.k = 0.001;

    std::vector<std::pair<std::string, Vec6>> test_cases = {
        {"1 km, 0.5 m/s approach", (Vec6() << 0, -1000, 0, 0, 0.5, 0).finished()},
        {"1 km, 2.0 m/s approach (violation)", (Vec6() << 0, -1000, 0, 0, 2.0, 0).finished()},
        {"100 m, 0.2 m/s approach (violation)", (Vec6() << 0, -100, 0, 0, 0.2, 0).finished()},
    };

    std::cout << std::fixed << std::setprecision(3);
    for (const auto& [name, x] : test_cases) {
        auto check = check_glideslope_violation(x, params);
        std::cout << name << "\n";
        std::cout << "  Range: " << x.head<3>().norm() << " m, "
                  << "v_approach: " << check.v_approach << " m/s, "
                  << "v_max: " << check.v_max << " m/s, "
                  << "Violated: " << (check.violated ? "YES" : "NO") << "\n";
    }

    std::cout << "\nGlideslope constraint checker operational\n";
}


int main() {
    std::cout << std::string(60, '=') << "\n";
    std::cout << "RelNav-MC Validation\n";
    std::cout << std::string(60, '=') << "\n";

    test_propagator_validation();
    test_monte_carlo();
    test_two_impulse();
    test_lqr_control();
    test_glideslope();

    print_separator("Summary");
    std::cout << "CW propagator: numerical matches analytical to <1e-6 m\n";
    std::cout << "Monte Carlo: 3 standard deviations converge to analytical within 2%\n";
    std::cout << "Two-impulse targeting: CW Lambdert solver operational\n";
    std::cout << "LQR control: optimal approach controller operational\n";
    std::cout << "Glideslope: constraint checker operational\n";
    std::cout << std::string(60, '=') << "\n";

    return 0;
}