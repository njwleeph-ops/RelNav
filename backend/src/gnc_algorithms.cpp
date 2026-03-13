/**
 * @file gnc_algorithms.cpp
 * @brief GNC Algorithms for Proximity Operations - Implementation
 */

#include <cmath>
#include <algorithm>
#include <iostream>

#include "gnc_algorithms.hpp"

namespace relnav
{

    // ----------------------------------------------------------------------------
    // Guidance
    // ----------------------------------------------------------------------------

    Vec3 rodrigues_rotate(const Vec3 &v, const Vec3 &k, double theta)
    {
        double c = std::cos(theta);
        double s = std::sin(theta);

        return v * c + k.cross(v) * s + k * (k.dot(v)) * (1.0 - c);
    }

    Vec3 find_perpendicular(const Vec3 &v)
    {
        Vec3 candidate = (std::abs(v(0)) < 0.9) ? Vec3::UnitX() : Vec3::UnitZ();
        Vec3 perp = v.cross(candidate);

        return perp.normalized();
    }

    Vec3 compute_rotation_axis(const Vec3 &position, const Vec3 &approach_axis)
    {
        Vec3 rot_axis = position.cross(approach_axis);
        double mag = rot_axis.norm();

        if (mag < 1e-10)
        {
            return find_perpendicular(approach_axis);
        }

        return rot_axis / mag;
    }

    double compute_angle_to_corridor(
        const Vec3 &position,
        const GlideslopeParams &params)
    {
        double angle = glideslope_approach_angle(position, params.approach_axis);
        double offset = angle - params.corridor_angle;

        return std::max(offset, 0.0);
    }

    Vec3 compute_corridor_entry_point(
        const Vec3 &position,
        const GuidanceConfig &config)
    {
        double current_range = position.norm();

        double entry_range = std::max(
            current_range * config.entry_range_fraction,
            config.terminal_range + 10.0);
        entry_range = std::min(entry_range, current_range);

        Vec3 approach = config.approach_axis.normalized();
        Vec3 pos_hat = position.normalized();
        Vec3 rot_axis = approach.cross(pos_hat);
        double rot_mag = rot_axis.norm();

        if (rot_mag < 1e-10)
        {
            rot_axis = find_perpendicular(approach);
        }
        else
        {
            rot_axis /= rot_mag;
        }

        Vec3 entry_dir = rodrigues_rotate(approach, rot_axis, config.corridor_angle);

        return entry_dir.normalized() * entry_range;
    }

    std::vector<Vec6> generate_cw_transfer_arc(
        const Vec3 &position,
        const GuidanceConfig &config,
        double n)
    {
        std::vector<Vec6> waypoints;

        double angle_to_corridor = compute_angle_to_corridor(position, config);

        if (angle_to_corridor < 1e-6)
        {
            return waypoints;
        }

        Vec3 entry_point = compute_corridor_entry_point(position, config);

        double orbital_period = 2.0 * M_PI / n;
        double angle_fraction = angle_to_corridor / M_PI;

        double T_min = orbital_period * 0.125;
        double T_max = orbital_period * 0.5;
        double T = T_min + angle_fraction * (T_max - T_min);

        Vec3 v0_required = solve_cw_tpbvp(position, entry_point, T, n);

        if (v0_required.norm() < 1e-12)
        {
            return waypoints;
        }

        Vec6 x0;
        x0.head<3>() = position;
        x0.tail<3>() = v0_required;

        int num_waypoints = std::max(
            config.min_waypoints,
            static_cast<int>(std::ceil(T / (orbital_period * 0.05))));
        num_waypoints = std::min(num_waypoints, 20);

        double dt_wp = T / (num_waypoints + 1);

        for (int i = 1; i <= num_waypoints; ++i)
        {
            double t_sample = i * dt_wp;
            Vec6 x_wp = cw_propagate(x0, n, t_sample);
            waypoints.push_back(x_wp);
        }

        Vec6 x_final = cw_propagate(x0, n, T);

        waypoints.push_back(x_final);

        return waypoints;
    }

    std::vector<Vec6> generate_entry_arc(
        const Vec3 &position,
        const GuidanceConfig &config)
    {
        std::vector<Vec6> waypoints;

        double angle_to_corridor = compute_angle_to_corridor(
            position, config);

        if (angle_to_corridor < 1e-6)
        {
            return waypoints;
        }

        Vec3 entry_point = compute_corridor_entry_point(position, config);

        double current_range = position.norm();
        double entry_range = entry_point.norm();
        
        int num_waypoints = std::max(
            config.min_waypoints,
            static_cast<int>(std::ceil(angle_to_corridor / config.waypoint_angular_spacing))
        );

        Vec3 rot_axis = compute_rotation_axis(position, config.approach_axis);
        Vec3 p_hat = position.normalized();
        
        std::vector<Vec3> positions;
        positions.resize(num_waypoints);

        for (int i = 1; i <= num_waypoints; ++i) {
            double frac = static_cast<double>(i) / num_waypoints;
            double angle = frac * angle_to_corridor;
            double range = current_range + frac * (entry_range - current_range);

            Vec3 direction = rodrigues_rotate(p_hat, rot_axis, angle);
            positions.push_back(direction.normalized() * range);
        }

        for (int i = 0; i < static_cast<int>(positions.size()); ++i) {
            Vec6 wp = Vec6::Zero();
            wp.head<3>() = positions[i];

            Vec3 direction;
            
            if (i + 1 < static_cast<int>(positions.size())) {
                direction = positions[i + 1] - positions[i];
            } else {
                direction = -positions[i];
            }

            double segment_dist = direction.norm();

            if (segment_dist > 1e-10) {
                double v_max = glideslope_velocity_limit(
                    positions[i], config.k, config.min_range
                );
                double v_mag = std::min(segment_dist / 30.0, v_max);
                wp.tail<3>() = direction.normalized() * v_mag;
            }

            waypoints.push_back(wp);
        }

        return waypoints;
    }

    void determine_phase(
        const Vec6 &x,
        GuidanceState &state,
        const GuidanceConfig &config,
        double n)
    {
        Vec3 position = x.head<3>();
        Vec3 velocity = x.tail<3>();
        double range = position.norm();
        double speed = velocity.norm();
        double angle = glideslope_approach_angle(position, config.approach_axis);

        if (range < config.success_range && speed < config.success_velocity)
        {
            state.phase = GuidancePhase::COMPLETE;
            return;
        }

        if (state.elapsed_time >= config.timeout)
        {
            state.phase = GuidancePhase::ABORT;
            return;
        }

        switch (state.phase)
        {
        case GuidancePhase::CORRIDOR_ENTRY:
        {
            bool inside = is_inside_corridor(
                position, config.corridor_angle, config.approach_axis);

            bool waypoints_done = state.current_waypoint_idx >= static_cast<int>(state.waypoints.size());

            if (inside || waypoints_done)
            {
                if (range < config.terminal_range && angle <= config.corridor_angle)
                {
                    state.phase = GuidancePhase::TERMINAL;
                }
                else
                {
                    state.phase = GuidancePhase::CORRIDOR_TRAVERSE;
                }
            }

            return;
        }
        case GuidancePhase::CORRIDOR_TRAVERSE:
        {
            if (range < config.terminal_range && angle <= config.corridor_angle)
            {
                state.phase = GuidancePhase::TERMINAL;
                return;
            }

            bool inside = is_inside_corridor(
                position, config.corridor_angle, config.approach_axis);

            if (!inside)
            {
                state.phase = GuidancePhase::CORRIDOR_ENTRY;
                state.waypoints = generate_entry_arc(position, config);
                state.current_waypoint_idx = 0;
            }
            return;
        }
        case GuidancePhase::TERMINAL:
        case GuidancePhase::COMPLETE:
        case GuidancePhase::ABORT:
            return;
        }
    }

    bool try_advance_waypoint(
        const Vec3 &position,
        GuidanceState &state,
        const GuidanceConfig &config)
    {
        if (state.current_waypoint_idx >= static_cast<int>(state.waypoints.size()))
        {
            return false;
        }

        Vec3 target = state.waypoints[state.current_waypoint_idx].head<3>();
        double dist = (position - target).norm();

        if (dist < config.waypoint_advance_radius)
        {
            state.current_waypoint_idx++;
            return true;
        }

        return false;
    }

    GuidanceState initialize_guidance(
        const Vec3 &position,
        const GuidanceConfig &config,
        double n)
    {
        GuidanceState state;
        double range = position.norm();
        double angle = glideslope_approach_angle(position, config.approach_axis);

        if (range < config.terminal_range && angle <= config.corridor_angle)
        {
            state.phase = GuidancePhase::TERMINAL;
        }
        else if (is_inside_corridor(position, config.corridor_angle, config.approach_axis))
        {
            state.phase = GuidancePhase::CORRIDOR_TRAVERSE;
        }
        else
        {
            state.phase = GuidancePhase::CORRIDOR_ENTRY;
            state.waypoints = generate_entry_arc(position, config);
        }

        return state;
    }

    GuidanceOutput update_guidance(
        const Vec6 &x,
        GuidanceState &state,
        const GuidanceConfig &config,
        double dt)
    {
        Vec3 position = x.head<3>();

        state.elapsed_time += dt;
        determine_phase(x, state, config, config.n);

        if (state.phase == GuidancePhase::CORRIDOR_ENTRY)
        {
            try_advance_waypoint(position, state, config);
        }

        GuidanceOutput output;
        output.phase = state.phase;
        output.x_reference = Vec6::Zero();
        output.waypoint_idx = -1;
        output.total_waypoints = static_cast<int>(state.waypoints.size());

        switch (state.phase)
        {
        case GuidancePhase::CORRIDOR_ENTRY:
        {
            output.glideslope_active =
                state.current_waypoint_idx >=
                static_cast<int>(state.waypoints.size() - 1);
            output.corridor_active = true;

            if (state.current_waypoint_idx < static_cast<int>(state.waypoints.size()))
            {
                output.x_reference = state.waypoints[state.current_waypoint_idx];
                output.waypoint_idx = state.current_waypoint_idx;
            }

            break;
        }
        case GuidancePhase::CORRIDOR_TRAVERSE:
        {
            output.glideslope_active = true;
            output.corridor_active = true;
            break;
        }
        case GuidancePhase::TERMINAL:
        {
            output.glideslope_active = true;
            output.corridor_active = false;
            break;
        }
        case GuidancePhase::COMPLETE:
        case GuidancePhase::ABORT:
        {
            output.glideslope_active = false;
            output.corridor_active = false;
            break;
        }
        }

        return output;
    }

    /// Old
    double glideslope_velocity_limit(const Vec3 &position, const double k, const double min_range)
    {
        double range = std::max(position.norm(), min_range);

        if (range > 100.0)
        {
            return 5 * k * range;
        }
        else
        {
            return 2 * k * range;
        }
    }

    double glideslope_approach_angle(const Vec3 &position, const Vec3 &axis)
    {
        double r_magnitude = position.norm();

        if (r_magnitude < 1e-10)
        {
            return 0.0;
        }

        Vec3 r_hat = position / r_magnitude;

        double arg = std::clamp(r_hat.dot(axis), -1.0, 1.0);

        return std::acos(arg);
    }

    bool is_inside_corridor(const Vec3 &position, const double &corridor_angle, const Vec3 &axis)
    {
        double angle = glideslope_approach_angle(position, axis);
        return angle < corridor_angle;
    }

    Vec3 compute_edge_waypoint(
        const Vec3 &position,
        const double &corridor_angle,
        const Vec3 &axis,
        const double &target_range)
    {
        double angle = glideslope_approach_angle(position, axis);

        if (angle < corridor_angle + 0.0001)
        {
            return Vec3::Zero();
        }

        double range = position.norm();
        double rotation_angle = angle - corridor_angle;

        Vec3 rot_axis = position.cross(axis);
        double rot_axis_mag = rot_axis.norm();

        if (rot_axis_mag < 1e-10)
        {
            return axis * range;
        }

        rot_axis /= rot_axis_mag;

        double c = std::cos(rotation_angle);
        double s = std::sin(rotation_angle);

        Vec3 waypoint_direction = (position * c + rot_axis.cross(position) * s + rot_axis * (rot_axis.dot(position)) * (1.0 - c)).normalized();

        double waypoint_buffer = 1.0;

        return waypoint_direction * target_range * waypoint_buffer;
    }

    GlideslopeCheck check_glideslope_violation(const Vec6 &x, const GlideslopeParams &params)
    {
        GlideslopeCheck result;

        Vec3 r = x.head<3>();
        Vec3 v = x.tail<3>();

        double range = r.norm();

        if (range < 1e-10)
        {
            result.violated = false;
            result.margin = params.k * params.min_range;
            result.v_approach = 0.0;
            result.v_max = result.margin;
            return result;
        }

        Vec3 r_hat = -r / range; // Unit vector towards target so -r (magnitude of 1)

        result.v_approach = v.dot(r_hat);
        result.v_max = glideslope_velocity_limit(r, params.k, params.min_range);
        result.margin = result.v_max - result.v_approach;
        result.violated = result.margin < 0;

        return result;
    }

    Vec3 glideslope_waypoint_guidance(const Vec3 &x, const GlideslopeParams &params)
    {
        double angle = glideslope_approach_angle(x, params.approach_axis);
        double radial_dist = x.norm();

        if (std::abs(angle) > params.corridor_angle)
        {
            Vec3 waypoint;
            waypoint << radial_dist * std::sin(params.corridor_angle),
                radial_dist * std::cos(params.corridor_angle),
                0;

            return waypoint;
        }

        return Vec3::Zero();
    }

    Vec3 apply_glideslope_constraint(
        const Vec3 &u,
        const Vec6 &x,
        double dt,
        const GlideslopeParams &params,
        const Vec3 *x_reference)
    {
        Vec3 r = x.head<3>();
        Vec3 v = x.tail<3>();

        if (x_reference)
        {
            Vec3 r_to_ref = *x_reference - r;
            double range_to_ref = r_to_ref.norm();

            if (range_to_ref < params.min_range)
            {
                return u;
            }

            Vec3 r_hat = r_to_ref / range_to_ref;
            double range_to_origin = r.norm();
            double v_max = glideslope_velocity_limit(r, params.k, params.min_range);

            double v_approach = v.dot(r_hat);
            double u_approach = u.dot(r_hat);
            double v_approach_predicted = v_approach + u_approach * dt;

            if (v_approach_predicted <= v_max)
            {
                return u;
            }

            double u_approach_max = (v_max - v_approach) / dt;
            Vec3 u_approach_vec = u_approach * r_hat;
            Vec3 u_lateral = u - u_approach_vec;
            Vec3 u_approach_clamped = std::min(u_approach, u_approach_max) * r_hat;

            return u_lateral + u_approach_clamped;
        }
        else
        {

            double range = r.norm();

            if (range < params.min_range)
            {
                return u; // Too close, no constraint needed
            }

            Vec3 r_hat = -r / range;
            double v_approach = v.dot(r_hat);
            double v_max = glideslope_velocity_limit(r, params.k, params.min_range);
            double u_approach = u.dot(r_hat);
            double v_approach_predicted = v_approach + u_approach * dt;

            if (v_approach_predicted <= v_max)
            {
                return u; // Within bounds no needed constraint;
            }

            double u_approach_max = (v_max - v_approach) / dt;
            Vec3 u_approach_vec = u_approach * r_hat;
            Vec3 u_lateral = u - u_approach_vec;
            Vec3 u_approach_clamped = std::min(u_approach, u_approach_max) * r_hat;

            return u_lateral + u_approach_clamped;
        }
    }

    // ----------------------------------------------------------------------------
    // Navigation EKF Filtering
    // ----------------------------------------------------------------------------

    NavFilterState ekf_predict(
        const NavFilterState &state,
        const Vec3 &u_applied,
        double n,
        double dt,
        const Mat6 &Q_process)
    {
        Mat6 Phi = cw_state_transition_matrix(n, dt);
        Mat63 B = cw_control_matrix();

        NavFilterState predicted;
        predicted.x_hat = Phi * state.x_hat + B * (u_applied * dt);
        predicted.P = Phi * state.P * Phi.transpose() + Q_process;

        return predicted;
    }

    NavFilterState ekf_update(
        const NavFilterState &predicted,
        const Measurement &meas,
        const Mat3 &R_meas)
    {
        if (!meas.valid)
        {
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

    Vec3 measurement_function(const Vec6 &x)
    {
        double px = x(0);
        double py = x(1);
        double pz = x(2);

        double range = std::sqrt(px * px + py * py + pz * pz);
        double azimuth = std::atan2(py, px);
        double elevation = std::atan2(pz, std::sqrt(px * px + py * py));

        return Vec3{range, azimuth, elevation};
    }

    Mat36 measurement_jacobian(const Vec6 &x)
    {
        double px = x(0);
        double py = x(1);
        double pz = x(2);

        double r = std::sqrt(px * px + py * py + pz * pz);
        double rxy = std::sqrt(px * px + py * py);

        Mat36 H = Mat36::Zero();

        // d(range)/d(pos)
        H(0, 0) = px / r;
        H(0, 1) = py / r;
        H(0, 2) = pz / r;

        // d(azimuth)/d(pos) - atan2(py, px)
        double rxy2 = px * px + py * py;
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
        const Vec6 &x_true,
        const SensorModel &sensor,
        std::mt19937 &rng)
    {
        double range = x_true.head<3>().norm();

        Measurement meas;
        meas.valid = (range <= sensor.max_range) && (range >= sensor.min_range);

        if (!meas.valid)
        {
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
        const Vec6 &x_true,
        const Mat6 &P0,
        std::mt19937 &rng)
    {
        std::normal_distribution<double> dist(0.0, 1.0);
        Vec6 noise;

        for (int i = 0; i < 6; ++i)
        {
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

    NavConfig default_nav_config()
    {
        NavConfig config;

        config.sensor.range_noise = 1.0;
        config.sensor.bearing_noise = 0.005;
        config.sensor.update_rate = 1.0;
        config.sensor.max_range = 2000.0;
        config.sensor.min_range = 2.0;

        config.filter.R_meas = Mat3::Zero();
        config.filter.R_meas(0, 0) = 5.455595 * 1.0;
        config.filter.R_meas(1, 1) = 5.455595 * 0.000025;
        config.filter.R_meas(2, 2) = 5.455595 * 0.000025;

        config.filter.Q_process = Mat6::Zero();
        config.filter.Q_process(0, 0) = 2.07e-7;
        config.filter.Q_process(1, 1) = 2.07e-7;
        config.filter.Q_process(2, 2) = 2.07e-7;
        config.filter.Q_process(3, 3) = 2.07e-8;
        config.filter.Q_process(4, 4) = 2.07e-8;
        config.filter.Q_process(5, 5) = 2.07e-8;

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
        const Mat6 &A,
        const Mat63 &B,
        const Mat6 &Q,
        const Mat3 &R,
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
        for (int i = 0; i < 12 && col < 6; ++i)
        {
            if (eig.eigenvalues()(i).real() < 1e-10)
            {
                U_stable.col(col) = eig.eigenvectors().col(i);
                col++;
            }
        }

        if (col < 6)
        {
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

    Mat36 compute_lqr_gain(double n, const Mat6 &Q, const Mat3 &R)
    {
        Mat6 A = cw_state_matrix(n);
        Mat63 B = cw_control_matrix();

        // Solve Riccati equation
        Mat6 P = solve_CARE(A, B, Q, R);

        // Optimal gain
        Mat36 K = R.inverse() * B.transpose() * P;

        return K;
    }

    Vec3 compute_lqr_control(const Vec6 &x, const Mat36 &K, const Vec6 &x_ref)
    {
        Vec6 e = x - x_ref;
        return -K * e;
    }

    Vec3 saturate_control(const Vec3 &u, double u_max)
    {
        double mag = u.norm();

        if (mag > u_max)
        {
            return u * (u_max / mag);
        }

        return u;
    }

    // ----------------------------------------------------------------------------
    // Approach Guidance
    // ----------------------------------------------------------------------------
    ApproachResult run_approach_guidance(
        const Vec6 &x0,
        double n,
        const ApproachParams &base_params,
        const NavConfig *nav,
        std::mt19937 *rng,
        double thrust_mag_error,
        double thrust_pointing_error)
    {
        ApproachParams params = base_params;
        ApproachResult result;
        result.num_points = 0;
        result.total_dv = 0.0;
        result.saturation_count = 0;
        result.trajectory.count = 0;

        // Precompute LQR gain
        Mat36 K = compute_lqr_gain(n, params.Q, params.R);

        Vec6 x = x0;
        Vec6 x_true = x0;

        double t = 0.0;
        double initial_range = x0.head<3>().norm();
        double min_range_seen = initial_range;

        NavFilterState nav_state;
        double time_since_measurement = 0.0;
        int measurement_count = 0;
        int dropout_count = 0;

        if (nav && rng)
        {
            nav_state = initialize_filter(x0, nav->filter.P0, *rng);
            x = nav_state.x_hat;
        }

        GuidanceState guidance_state;
        params.guidance_config.n = n;

        if (params.use_phased_guidance)
        {
            guidance_state = initialize_guidance(x.head<3>(), params.guidance_config, params.guidance_config.n);
        }

        int max_steps = static_cast<int>(params.timeout / params.dt);

        result.trajectory.points.resize(max_steps + 1);
        result.control_history.resize(max_steps);
        result.waypoint_history.resize(max_steps);
        result.phase_history.resize(max_steps + 1);

        for (int i = 0; i <= max_steps; ++i)
        {
            result.trajectory.points[i].t = t;
            result.trajectory.points[i].x = x;

            double range = x.head<3>().norm();
            double velocity = x.tail<3>().norm();

            min_range_seen = std::min(min_range_seen, range);

            if (std::isnan(range) || std::isnan(velocity))
            {
                break;
            }

            if (range < params.success_range && velocity < params.success_velocity)
            {
                result.success = true;
                result.num_points = i + 1;
                result.final_range = range;
                result.final_velocity = velocity;
                result.duration = t;
                result.trajectory.count = result.num_points;
                result.measurement_count = measurement_count;
                result.dropout_count = dropout_count;
                result.min_range_seen = min_range_seen;

                return result;
            }

            if (i == max_steps)
            {
                break;
            }

            if (nav && rng)
            {
                double meas_interval = 1.0 / nav->sensor.update_rate;
                time_since_measurement += params.dt;

                if (time_since_measurement >= meas_interval)
                {
                    Measurement meas = generate_measurement(
                        x_true, nav->sensor, *rng);

                    if (meas.valid)
                    {
                        nav_state = ekf_update(
                            nav_state, meas, nav->filter.R_meas);
                        measurement_count++;
                    }
                    else
                    {
                        dropout_count++;
                    }

                    time_since_measurement = 0.0;
                }

                x = nav_state.x_hat;
            }

            Vec6 x_ref = Vec6::Zero();

            if (params.use_phased_guidance)
            {
                GuidanceOutput guidance = update_guidance(
                    x, guidance_state, params.guidance_config, params.dt);

                x_ref = guidance.x_reference;
                result.phase_history[i] = guidance.phase;
                result.waypoint_history[i] = x_ref;

                if (guidance.phase == GuidancePhase::COMPLETE || guidance.phase == GuidancePhase::ABORT)
                {
                    break;
                }

                Vec3 u = compute_lqr_control(x, K, x_ref);

                if (guidance.phase == GuidancePhase::TERMINAL)
                {
                    params.corridor_params.k = params.guidance_config.terminal_k;
                }

                if (guidance.glideslope_active)
                {
                    Vec3 glideslope_ref = x_ref.head<3>();
                    u = apply_glideslope_constraint(
                        u, x, params.dt, params.corridor_params, &glideslope_ref);
                }

                if (rng)
                {
                    std::normal_distribution<double> dist(0.0, 1.0);

                    double mag_scale = 1.0 + thrust_mag_error * dist(*rng);
                    double angle1 = thrust_pointing_error * dist(*rng);
                    double angle2 = thrust_pointing_error * dist(*rng);

                    Vec3 u_dir = u.normalized();
                    Vec3 perp1 = u_dir.cross(Vec3::UnitX());

                    if (perp1.norm() < 1e-16)
                    {
                        perp1 = u_dir.cross(Vec3::UnitZ());
                    }

                    perp1.normalize();
                    Vec3 perp2 = u_dir.cross(perp1).normalized();

                    Vec3 u_rotated = u_dir + angle1 * perp1 + angle2 * perp2;
                    u_rotated.normalize();

                    u = mag_scale * u.norm() * u_rotated;
                }

                Vec3 u_saturated = saturate_control(u, params.u_max);

                if ((u_saturated - u).norm() > 1e-10)
                {
                    result.saturation_count++;
                }

                u = u_saturated;
                result.control_history[i] = u;
                result.total_dv += u.norm() * params.dt;

                if (nav && rng)
                {
                    x_true = rk4_step(x_true, t, params.dt, n, u);
                    nav_state = ekf_predict(nav_state, u, n, params.dt, nav->filter.Q_process);
                    x = nav_state.x_hat;
                }
                else
                {
                    x = rk4_step(x, t, params.dt, n, u);
                }
            }
            else
            {
                bool inside = is_inside_corridor(
                    x.head<3>(),
                    params.corridor_params.corridor_angle,
                    params.corridor_params.approach_axis);

                Vec3 waypoint = inside ? Vec3::Zero() : compute_edge_waypoint(x.head<3>(), params.corridor_params.corridor_angle, params.corridor_params.approach_axis, params.success_range);
                Vec6 full_waypoint;
                full_waypoint.head<3>() = waypoint;
                full_waypoint.tail<3>() = Vec3::Zero();
                result.waypoint_history[i] = full_waypoint;
                x_ref.head<3>() = waypoint;

                Vec3 u = compute_lqr_control(x, K, x_ref);

                u = apply_glideslope_constraint(
                    u, x, params.dt, params.corridor_params);

                if (rng)
                {
                    std::normal_distribution<double> dist(0.0, 1.0);

                    double mag_scale = 1.0 + thrust_mag_error * dist(*rng);

                    double angle1 = thrust_pointing_error * dist(*rng);
                    double angle2 = thrust_pointing_error * dist(*rng);

                    Vec3 u_dir = u.normalized();

                    Vec3 perp1 = u_dir.cross(Vec3::UnitX());

                    if (perp1.norm() < 1e-16)
                    {
                        perp1 = u_dir.cross(Vec3::UnitZ());
                    }

                    perp1.normalize();
                    Vec3 perp2 = u_dir.cross(perp1).normalized();

                    Vec3 u_rotated = u_dir + angle1 * perp1 + angle2 * perp2;
                    u_rotated.normalize();

                    u = mag_scale * u.norm() * u_rotated;
                }

                Vec3 u_saturated = saturate_control(u, params.u_max);

                if ((u_saturated - u).norm() > 1e-10)
                {
                    result.saturation_count++;
                }

                u = u_saturated;
                result.control_history[i] = u;
                result.total_dv += u.norm() * params.dt;

                if (nav && rng)
                {
                    x_true = rk4_step(x_true, t, params.dt, n, u);
                    nav_state = ekf_predict(nav_state, u, n, params.dt, nav->filter.Q_process);
                    x = nav_state.x_hat;
                }
                else
                {
                    x = rk4_step(x, t, params.dt, n, u);
                }
            }
            t += params.dt;
        }

        result.num_points = max_steps + 1;
        result.final_range = x.head<3>().norm();
        result.final_velocity = x.tail<3>().norm();
        result.duration = t;
        result.trajectory.count = result.num_points;
        result.measurement_count = measurement_count;
        result.dropout_count = dropout_count;
        result.min_range_seen = min_range_seen;

        if (result.min_range_seen <= params.success_range && result.final_velocity >= params.success_velocity)
        {
            result.success = false;
        }
        else if (result.min_range_seen >= params.success_range)
        {
            result.success = false;
        }
        else if (result.final_range >= params.success_range)
        {
            result.success = false;
        }
        else
        {
            result.success = true;
        }

        return result;
    }
}