/**
 * @file gnc_algorithms.cpp
 * @brief GNC Algorithms for Proximity Operations - Implementation
 */

#include <cmath>
#include <algorithm>

#include "gnc_algorithms.hpp"

namespace relnav {

//StateHistory compute_transfer_trajectory(
  //  const Vec6& x0,
  //  const Vec3& rf,
  //  double tof,
  //  double n,
  //  int num_points)
//{
  //  StateHistory result;

   // Vec3 r0 = x0.head<3>();
   // Vec3 v0 = x0.tail<3>();

    // Apply first delta-v
   // Vec6 x0_post;
   // x0_post << r0, v0;

    //result.trajectory = propagate_analytical(x0_post, tof, n, num_points);

  //  return result;
//}

// ----------------------------------------------------------------------------
// Guideslope Guidance
// ----------------------------------------------------------------------------

double glideslope_velocity_limit(const Vec3& position, const GlideslopeParams& params) {
    double range = std::max(position.norm(), params.min_range);
    return params.k * range;
}

double glideslope_approach_angle(const Vec3& position, const Vec3& axis) {
    double r_magnitude = position.norm();
    Vec3 r_hat = position / r_magnitude;

    double arg = std::clamp(r_hat.dot(axis), -1.0, 1.0);

    return std::acos(arg);
}

GlideslopeCheck check_glideslope_violation(const Vec6& x, const GlideslopeParams& params) {
    GlideslopeCheck result;

    Vec3 r = x.head<3>();
    Vec3 v = x.tail<3>();

    double range = r.norm();
    Vec3 r_hat = -r / std::max(range, 1e-6);    // Unit vector towards target so -r (magnitude of 1)

    result.v_approach = v.dot(r_hat);
    result.v_max = glideslope_velocity_limit(r, params);
    result.margin = result.v_max - result.v_approach;
    result.violated = result.margin < 0;

    return result;
}

Vec3 glideslope_velocity_guidance(const Vec6& x, const GlideslopeParams& params) {
    Vec3 r = x.head<3>();
    Vec3 v = x.tail<3>();

    double range = r.norm();
    Vec3 r_hat = -r / std::max(range, 1e-6);

    double v_approach = v.dot(r_hat);
    double v_max = glideslope_velocity_limit(r, params);

    if (v_approach > v_max) {
        // Brake to glideslope limit
        double dv_mag = v_approach - v_max;
        return -dv_mag * r_hat;
    }

    return Vec3::Zero();
}

Vec3 glideslope_waypoint_guidance(const Vec3& x, const GlideslopeParams& params) {
    double angle = glideslope_approach_angle(x, params.approach_axis);
    double radial_dist = x.norm();

    if (std::abs(angle) > params.corridor_angle) {
        Vec3 waypoint;
        waypoint << 
            radial_dist * std::sin(params.corridor_angle), 
            radial_dist * std::cos(params.corridor_angle),
            0;
        
        return waypoint;
    }

    return Vec3::Zero();

}

// ----------------------------------------------------------------------------
// LQR Optimal Control
// ----------------------------------------------------------------------------

Mat6 solve_CARE(
    const Mat6& A,
    const Mat63& B,
    const Mat6& Q,
    const Eigen::Matrix3d& R,
    int max_iter,
    double tol) 
{
    Mat6 P = Q;
    Eigen::Matrix3d R_inv = R.inverse();

    for (int iter = 0; iter < max_iter; ++iter) {
        // Riccati iteration
        Mat6 PBRinvBtrP = P * B * R_inv * B.transpose() * P;
        Mat6 P_new = Q + A.transpose() * P + P * A - PBRinvBtrP;

        // Check convergence
        double diff = (P_new - P).norm();
        P = P_new;

        if (diff < tol) {
            break;
        }
    }

    return P;
}

Eigen::Matrix<double, 3, 6> compute_lqr_gain(
    double n,
    const Mat6& Q,
    const Eigen::Matrix3d& R)
{
    Mat6 A = cw_state_matrix(n);
    Mat63 B = cw_control_matrix();

    // Solve Riccati equation
    Mat6 P = solve_CARE(A, B, Q, R);

    // Optimal gain
    Eigen::Matrix<double, 3, 6> K = R.inverse() * B.transpose() * P;

    return K;
}

Vec3 lqr_control(
    const Vec6& x,
    const Eigen::Matrix<double, 3, 6>& K,
    const Vec6& x_ref,
    double u_max)
{
    Vec6 e = x - x_ref;
    Vec3 u = -K * e;

    // Saturation
    if (u_max > 0) {
        double u_norm = u.norm();

        if (u_norm > u_max) {
            u *= u_max / u_norm;
        }
    }

    return u;
}

LQRSimulationResult simulate_lqr_approach(
    const Vec6& x0,
    double duration,
    double n,
    const Mat6& Q,
    const Eigen::Matrix3d& R,
    double u_max,
    double dt)
{
    LQRSimulationResult result;

    // Compute LQR gain
    auto K = compute_lqr_gain(n, Q, R);

    // Control function wrapper
    auto control_func = [&](double t, const Vec6& x) -> Vec3 {
        return lqr_control(x, K, Vec6::Zero(), u_max);
    };

    // Number of points
    int num_points = static_cast<int>(duration / dt) + 1;

    // Propagate with control
    result.trajectory = propagate_numerical(x0, duration, n, num_points, control_func);

    // Compute control history and total delta-v
    result.total_dv = 0;
    result.control_history.resize(num_points);

    for (int i = 0; i < num_points; ++i) {
        result.control_history[i] = control_func(
            result.trajectory[i].first,
            result.trajectory[i].second
        );
        result.total_dv += result.control_history[i].norm() * dt;
    }

    return result;
}

}