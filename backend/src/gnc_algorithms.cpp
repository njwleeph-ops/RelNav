/**
 * @file gnc_algorithms.cpp
 * @brief GNC Algorithms for Proximity Operations - Implementation
 */

#include <cmath>
#include <algorithm>
#include <iostream>

#include "gnc_algorithms.hpp"

namespace relnav {

// ----------------------------------------------------------------------------
// Guideslope Guidance
// ----------------------------------------------------------------------------

double glideslope_velocity_limit(const Vec3& position, const double k, const double min_range) {
    double range = std::max(position.norm(), min_range);

    if (range > 100.0) {
        return 5 * k * range;
    } else {
        return 2 * k * range;
    }
}

double glideslope_approach_angle(const Vec3& position, const Vec3& axis) {
    double r_magnitude = position.norm();

    if (r_magnitude < 1e-10) {
        return 0.0;
    }

    Vec3 r_hat = position / r_magnitude;

    double arg = std::clamp(r_hat.dot(axis), -1.0, 1.0);

    return std::acos(arg);
}

bool is_inside_corridor(const Vec3& position, const double& corridor_angle, const Vec3& axis) {
    double angle = glideslope_approach_angle(position, axis);
    return angle < corridor_angle;
}

Vec3 compute_edge_waypoint(const Vec3& position, const double& corridor_angle, const Vec3& axis) {
    double angle = glideslope_approach_angle(position, axis);
    
    if (angle < corridor_angle + 0.0001) {
        return Vec3::Zero();
    }

    double range = position.norm();
    double rotation_angle = angle - corridor_angle;

    Vec3 rot_axis = position.cross(axis);
    double rot_axis_mag = rot_axis.norm();

    rot_axis /= rot_axis_mag;

    double c = std::cos(rotation_angle);
    double s = std::sin(rotation_angle);

    Vec3 waypoint = position * c + rot_axis.cross(position) * s + rot_axis * (rot_axis.dot(position)) * (1.0 - c);

    return waypoint;
}

GlideslopeCheck check_glideslope_violation(const Vec6& x, const GlideslopeParams& params) {
    GlideslopeCheck result;

    Vec3 r = x.head<3>();
    Vec3 v = x.tail<3>();

    double range = r.norm();
    
    if (range < 1e-10) {
        result.violated = false;
        result.margin = params.k * params.min_range;
        result.v_approach = 0.0;
        result.v_max = result.margin;
        return result;
    }

    Vec3 r_hat = -r / range;    // Unit vector towards target so -r (magnitude of 1)

    result.v_approach = v.dot(r_hat);
    result.v_max = glideslope_velocity_limit(r, params.k, params.min_range);
    result.margin = result.v_max - result.v_approach;
    result.violated = result.margin < 0;

    return result;
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

Vec3 apply_glideslope_constraint (
    const Vec3& u,
    const Vec6& x,
    double dt,
    const GlideslopeParams& params) 
{
    Vec3 r = x.head<3>();
    Vec3 v = x.tail<3>();

    double range = r.norm();

    if (range < params.min_range) {
        return u;   // Too close, no constraint needed
    }

    Vec3 r_hat = -r / range;
    double v_approach = v.dot(r_hat);
    double v_max = glideslope_velocity_limit(r, params.k, params.min_range);
    double u_approach = u.dot(r_hat);
    double v_approach_predicted = v_approach + u_approach * dt;

    if (v_approach_predicted <= v_max) {
        return u;   // Within bounds no needed constraint;
    }

    double u_approach_max = (v_max - v_approach) / dt;
    Vec3 u_approach_vec = u_approach * r_hat;
    Vec3 u_lateral = u - u_approach_vec;
    Vec3 u_approach_clamped = std::min(u_approach, u_approach_max) * r_hat;

    return u_lateral + u_approach_clamped;
}
// ----------------------------------------------------------------------------
// Navigation EKF Filtering
// ----------------------------------------------------------------------------

NavFilterState ekf_predict(
    const NavFilterState& state,
    const Vec3& u_applied,
    double n,
    double dt,
    const Mat6& Q_process)
{
    Mat6 Phi = cw_state_transition_matrix(n, dt);
    Mat63 B = cw_control_matrix();

    NavFilterState predicted;
    predicted.x_hat = Phi * state.x_hat + B * (u_applied * dt);
    predicted.P = Phi * state.P * Phi.transpose() + Q_process;

    return predicted;
}

NavFilterState ekf_update(
    const NavFilterState& predicted,
    const Measurement& meas,
    const Mat3& R_meas)
{
    if (!meas.valid) {
        return predicted;
    }

    Mat36 H = measurement_jacobian(predicted.x_hat);
    Vec3 z_predicted = measurement_function(predicted.x_hat);
    Vec3 residual = meas.z - z_predicted;

    // Wrap azimuthal and elevation residuals to [-pi, pi]
    residual(1) = std::atan2(std::sin(residual(1)), std::cos(residual(1)));
    residual(2) = std::atan2(std::sin(residual(2)), std::cos(residual(2)));

    Mat3 S = H * predicted.P * H.transpose() + R_meas;
    Mat63 K = predicted.P * H.transpose() * S.inverse();

    NavFilterState updated;
    updated.x_hat = predicted.x_hat + K * residual;
    updated.P = (Mat6::Identity() - K * H) * predicted.P;

    return updated;
}

Vec3 measurement_function(const Vec6& x) {
    double px = x(0);
    double py = x(1);
    double pz = x(2);

    double range = std::sqrt(px*px + py*py + pz*pz);
    double azimuth = std::atan2(py, px);
    double elevation = std::atan2(pz, std::sqrt(px*px + py*py));

    return Vec3{range, azimuth, elevation};
}

Mat36 measurement_jacobian(const Vec6& x) {
    double px = x(0);
    double py = x(1);
    double pz = x(2);

    double r = std::sqrt(px*px + py*py + pz*pz);
    double rxy = std::sqrt(px*px + py*py);

    Mat36 H = Mat36::Zero();

    // d(range)/d(pos)
    H(0, 0) = px / r;
    H(0, 1) = py / r;
    H(0, 2) = pz / r;

    // d(azimuth)/d(pos) - atan2(py, px)
    double rxy2 = px*px + py*py;
    H(1, 0) = -py / rxy2;
    H(1, 1) = px / rxy2;

    // d(elevation)/d(pos) - atan2(pz, rxy)
    double r2 = r * r;
    H(2, 0) = -px * pz / (r2 * rxy);
    H(2, 1) = -py * pz / (r2 * rxy);
    H(2, 2) = rxy / r2;

    // Columns 3-5 = 0

    return H;
}

Measurement generate_measurement(
    const Vec6& x_true,
    const SensorModel& sensor,
    std::mt19937& rng)
{
    double range = x_true.head<3>().norm();

    Measurement meas;
    meas.valid = (range <= sensor.max_range) && (range >= sensor.min_range);

    if (!meas.valid) {
        meas.z = Vec3::Zero();
        return meas;
    }

    Vec3 z_true = measurement_function(x_true);

    std::normal_distribution<double> range_dist(0.0, sensor.range_noise);
    std::normal_distribution<double> bearing_dist(0.0, sensor.bearing_noise);

    meas.z(0) = z_true(0) + range_dist(rng);
    meas.z(1) = z_true(1) + bearing_dist(rng);
    meas.z(2) = z_true(2) + bearing_dist(rng);

    return meas;
}

NavFilterState initialize_filter(
    const Vec6& x_true,
    const Mat6& P0,
    std::mt19937& rng)
{
    std::normal_distribution<double> dist(0.0, 1.0);
    Vec6 noise;

    for (int i = 0; i < 6; ++i) {
        noise(i) = dist(rng);
    }

    // Cholesky decomposition: P0 = L * L.transpose
    Eigen::LLT<Mat6> llt(P0);
    Mat6 L = llt.matrixL();

    NavFilterState state;
    state.x_hat = x_true + L * noise;
    state.P = P0;

    return state;
}

NavConfig default_nav_config() {
    NavConfig config;

    config.sensor.range_noise = 1.0;
    config.sensor.bearing_noise = 0.005;
    config.sensor.update_rate = 1.0;
    config.sensor.max_range = 2000.0;
    config.sensor.min_range = 2.0;

    config.filter.R_meas = Mat3::Zero();
    config.filter.R_meas(0, 0) = 1.0;
    config.filter.R_meas(1, 1) = 0.000025;
    config.filter.R_meas(2, 2) = 0.000025;

    config.filter.Q_process = Mat6::Zero();
    config.filter.Q_process(0, 0) = 0.001;
    config.filter.Q_process(1, 1) = 0.001;
    config.filter.Q_process(2, 2) = 0.001;
    config.filter.Q_process(3, 3) = 0.0001;
    config.filter.Q_process(4, 4) = 0.0001;
    config.filter.Q_process(5, 5) = 0.0001;

    config.filter.P0 = Mat6::Zero();
    config.filter.P0(0, 0) = 100.0;
    config.filter.P0(1, 1) = 100.0;
    config.filter.P0(2, 2) = 100.0;
    config.filter.P0(3, 3) = 0.01;
    config.filter.P0(4, 4) = 0.01;
    config.filter.P0(5, 5) = 0.01;

    return config;
}



// ----------------------------------------------------------------------------
// LQR Optimal Control
// ----------------------------------------------------------------------------

Mat6 solve_CARE(
    const Mat6& A,
    const Mat63& B,
    const Mat6& Q,
    const Mat3& R,
    int max_iter,
    double tol) 
{
    Mat3 R_inv = R.inverse();
    Mat6 BRinvBt = B * R_inv * B.transpose();

    Eigen::Matrix<double, 12, 12> H;
    H.setZero();
    H.block<6, 6>(0, 0) = A;
    H.block<6, 6>(0, 6) = -BRinvBt;
    H.block<6, 6>(6, 0) = -Q;
    H.block<6, 6>(6, 6) = -A.transpose();

    // Schur decomp
    Eigen::RealSchur<Eigen::Matrix<double, 12, 12>> schur(H);
    Eigen::Matrix<double, 12, 12> U = schur.matrixU();
    Eigen::Matrix<double, 12, 12> T = schur.matrixT();

    // Reorder so stable eigenvalues come first (negative and real)
    // For Hamiltonian, eigenvalues come in pos/neg pairs
    // Isolate 6 columns of U corresponding to stable eigenvalues
    Eigen::EigenSolver<Eigen::Matrix<double, 12, 12>> eig(H);

    Eigen::Matrix<std::complex<double>, 12, 6> U_stable;
    int col = 0;
    for (int i = 0; i < 12 && col < 6; ++i) {
        if (eig.eigenvalues()(i).real() < 1e-10) {
            U_stable.col(col) = eig.eigenvectors().col(i);
            col++;
        }
    }

    if (col < 6) {
        // Fallback: didn't find 6 stable eigenvalues
        return Q;
    }

    Eigen::Matrix<std::complex<double>, 6, 6> U11 = U_stable.topRows<6>();
    Eigen::Matrix<std::complex<double>, 6, 6> U21 = U_stable.bottomRows<6>();

    Mat6 P = (U21 * U11.inverse()).real();

    // Symmetrize
    P = 0.5 * (P + P.transpose());

    return P;
}

Mat36 compute_lqr_gain(double n, const Mat6& Q, const Mat3& R) {
    Mat6 A = cw_state_matrix(n);
    Mat63 B = cw_control_matrix();

    // Solve Riccati equation
    Mat6 P = solve_CARE(A, B, Q, R);

    // Optimal gain
    Mat36 K = R.inverse() * B.transpose() * P;

    return K;
}

Vec3 compute_lqr_control(const Vec6& x, const Mat36& K, const Vec6& x_ref) {
    Vec6 e = x - x_ref;
    return -K * e;
}

Vec3 saturate_control(const Vec3& u, double u_max) {
    double mag = u.norm();
    
    if (mag > u_max) {
        return u * (u_max / mag);
    }

    return u;
}

// ----------------------------------------------------------------------------
// Approach Guidance
// ----------------------------------------------------------------------------

ApproachResult run_approach_guidance(
    const Vec6& x0_true,
    double n,
    const ApproachParams& params,
    const NavConfig* nav,
    std::mt19937* rng)
{
    ApproachResult result;
    result.num_points = 0;
    result.total_dv = 0.0;
    result.success = false;
    result.saturation_count = 0;
    result.trajectory.count = 0;

    // Precompute LQR gain
    Mat36 K = compute_lqr_gain(n, params.Q, params.R);

    Vec6 x_true = x0_true;
    double t = 0.0;
    bool entered_corridor = false;

    // Initialize nav filter
    NavFilterState nav_state;
    double time_since_measurement = 0.0;
    int measurement_count = 0;
    int dropout_count = 0;

    if (nav && rng) {
        nav_state = initialize_filter(x0_true, nav->filter.P0, *rng);
    }

    int max_steps = static_cast<int>(params.timeout / params.dt);

    // Pre-allocate vectors
    result.trajectory.points.resize(max_steps + 1);
    result.control_history.resize(max_steps);
    result.waypoint_history.resize(max_steps);

    for (int i = 0; i <= max_steps; ++i) {
        result.trajectory.points[i].t = t;
        result.trajectory.points[i].x = x_true;

        double range = x_true.head<3>().norm();
        double velocity = x_true.tail<3>().norm();

        if (std::isnan(range) || std::isnan(velocity)) {
            break;
        }

        if (range < params.success_range && velocity < params.success_velocity) {
            result.success = true;
            result.num_points = i + 1;
            result.final_range = range;
            result.final_velocity = velocity;
            result.duration = t;
            result.trajectory.count = result.num_points;
            return result;
        }

        if (i == max_steps) {
            break;
        }

        Vec6 x_control = x_true;

        if (nav && rng) {
            double meas_interval = 1.0 / nav->sensor.update_rate;
            time_since_measruement += params.dt;

            if (time_since_measurement >= meas_interval){
                Measurement meas = generate_measurement(
                    x_true, nav->sensor, *rng
                );

                if (meas.valid) {
                    nav_state = ekf_update(
                        nav_state, meas, nav->filter.R_meas
                    );
                    measurement_count++;
                } else {
                    dropout_count++;
                }

                time_since_measurement = 0.0;
            }

            x_for_control = nav_state.x_hat;
        }

        bool inside = is_inside_corridor(
            x_for_control.head<3>(), 
            params.corridor_params.corridor_angle, 
            params.corridor_params.approach_axis
        );

        if (inside) {
            entered_corridor = true;
        }

        // Get waypoint based on corridor angle
        Vec3 waypoint = entered_corridor ? 
        Vec3::Zero() :
        compute_edge_waypoint(
            x_for_control.head<3>(), 
            params.corridor_params.corridor_angle, 
            params.corridor_params.approach_axis
        );
        result.waypoint_history[i] = waypoint;

        // Build reference state for LQR
        Vec6 x_ref = Vec6::Zero();
        x_ref.head<3>() = waypoint;

        // Compute LQR control thrust
        Vec3 u = compute_lqr_control(x_for_control, K, x_ref);

        // Apply glideslope constraints
        u = apply_glideslope_constraint(
            u, x_for_control, params.dt, params.corridor_params
        );

        // Saturate 
        Vec3 u_saturated = saturate_control(u, params.u_max);

        if ((u_saturated - u).norm() > 1e-10) {
            result.saturation_count++;
        }

        u = u_saturated;

        result.control_history[i] = u;

        // Accumulate delta-v
        result.total_dv += u.norm() * params.dt;

        // RK4 step
        x = rk4_step(x, t, params.dt, n , u);
        t += params.dt;
    }

    // Timeout
    result.num_points = max_steps + 1;
    result.final_range = x.head<3>().norm();
    result.final_velocity = x.tail<3>().norm();
    result.duration = t;
    result.trajectory.count = result.num_points;

    return result;
}
}