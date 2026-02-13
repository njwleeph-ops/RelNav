/**
 * @file test_glideslope_updates.cpp
 * @brief Unit testing for glideslope guidance functions
 */

#include <iostream>
#include <cmath>
#include <gmock/gmock.h>

#include <gtest/gtest.h>
#include "gnc_algorithms.hpp"

using namespace relnav;

// ----------------------------------------------------------------------------
// glideslope_approach_angle
// ----------------------------------------------------------------------------
TEST(CorridorAngle, OnAxisReturnsZero) {
    Vec3 r{0, -500, 0};
    Vec3 axis{0, -1, 0};
    double angle = glideslope_approach_angle(r, axis);
    EXPECT_NEAR(angle, 0.0, 1e-6);
}

TEST(CorridorAngle, FortyFiveDegrees) {
    Vec3 r{500, -500, 0};
    Vec3 axis{0, -1, 0};
    double angle = glideslope_approach_angle(r, axis);
    EXPECT_NEAR(angle, M_PI / 4, 1e-6);
}

TEST(CorridorAngle, NinetyDegrees) {
    Vec3 r{500, 0, 0};
    Vec3 axis{0, -1, 0};
    double angle = glideslope_approach_angle(r, axis);
    EXPECT_NEAR(angle, M_PI / 2, 1e-6);
}

TEST(CorridorAngle, HandlesZeroPosition) {
    Vec3 r{0, 0, 0};
    Vec3 axis{0, -1, 0};

    double angle = glideslope_approach_angle(r, axis);

    EXPECT_NEAR(angle, 0.0, 1e-10);
}

TEST(CorridorAngle, ThreeDimensional) {
    Vec3 r{100, -500, 200};
    Vec3 axis{0, -1, 0};

    double angle = glideslope_approach_angle(r, axis);
    double expected = std::acos(500.0 / r.norm());

    EXPECT_NEAR(angle, expected, 1e-10);
}

// ----------------------------------------------------------------------------
// is_inside_corridor
// ----------------------------------------------------------------------------

TEST(InsideCorridor, OnAxisInside) {
    Vec3 position{0, -500, 0};
    double half_angle = 0.175;
    Vec3 approach_axis{0, -1, 0};

    EXPECT_TRUE(is_inside_corridor(position, half_angle, approach_axis));
}

TEST(InsideCorridor, SmallOffsetIsInside) {
    double half_angle = 0.175;
    double range = 500.0;
    double offset = range * std::sin(half_angle * 0.5);

    Vec3 position{offset, -range * std::cos(half_angle * 0.5), 0};
    
    Vec3 approach_axis{0, -1, 0};

    EXPECT_TRUE(is_inside_corridor(position, half_angle, approach_axis));
}

TEST(InsideCorridor, LargeOffsetIsOutside) {
    Vec3 position{500, -500, 0};
    double half_angle = 0.175;
    Vec3 approach_axis{0, -1, 0};

    EXPECT_FALSE(is_inside_corridor(position, half_angle, approach_axis));
}

TEST(InsideCorridor, ExactlyOnBoundary) {
    double half_angle = 0.175;
    double range = 500.0;

    Vec3 position{
        range * std::sin(half_angle),
        -range * std::cos(half_angle),
        0
    };
    
    Vec3 approach_axis{0, -1, 0};
    
    // Boundary is not "inside" so should be false
    EXPECT_FALSE(is_inside_corridor(position, half_angle, approach_axis));
}

// ----------------------------------------------------------------------------
// compute_edge_waypoint
// ----------------------------------------------------------------------------

TEST(EdgeWaypoint, InsideCorridorReturnsOrigin) {
    Vec3 position{0, -500, 0};
    double angle = 0.175;
    Vec3 axis{0, -1, 0};

    Vec3 waypoint = compute_edge_waypoint(position, angle, axis);

    EXPECT_NEAR(waypoint.norm(), 0.0, 1e-10);
}

TEST(EdgeWaypoint, OutsideCorridorPreservesRange) {
    Vec3 position{400, -400, 0};
    double angle = 0.175;
    Vec3 axis{0, -1, 0};

    Vec3 waypoint = compute_edge_waypoint(position, angle, axis);

    EXPECT_NEAR(waypoint.norm(), position.norm(), 1e-6);
}

TEST(EdgeWaypoint, OutsideCorridorLandsOnEdge) {
    Vec3 position{400, -400, 0};
    double angle = 0.175;
    Vec3 axis{0, -1, 0};

    Vec3 waypoint = compute_edge_waypoint(position, angle, axis);

    double waypoint_angle = glideslope_approach_angle(waypoint, axis);

    EXPECT_NEAR(waypoint_angle, angle, 1e-6);
}

TEST(EdgeWaypoint, ThreeDimensionPreservesRange) {
    Vec3 position{300, -300, 300};
    double angle = 0.175;
    Vec3 axis{0, -1, 0};

    Vec3 waypoint = compute_edge_waypoint(position, angle, axis);

    EXPECT_NEAR(waypoint.norm(), position.norm(), 1e-6);
}

TEST(EdgeWaypoint, ThreeDimensionLandsOnEdge) {
    Vec3 position{300, -300, 300};
    double angle = 0.175;
    Vec3 axis{0, -1, 0};

    Vec3 waypoint = compute_edge_waypoint(position, angle, axis);
    
    double waypoint_angle = glideslope_approach_angle(waypoint, axis);

    EXPECT_NEAR(waypoint_angle, angle, 1e-6);
}

// ----------------------------------------------------------------------------
// check_glideslope_violation
// ----------------------------------------------------------------------------

TEST(CheckGlideslopeViolation, StationaryNotViolated) {
    Vec6 x;
    x << 0, -500, 0, 0, 0, 0;   
    GlideslopeParams params;

    GlideslopeCheck result = check_glideslope_violation(x, params);

    EXPECT_FALSE(result.violated);
    EXPECT_GT(result.margin, 0);
    EXPECT_NEAR(result.v_approach, 0.0, 1e-10);
}

TEST(CheckGlideslopeViolation, SlowApproachNotViolated) {
    Vec6 x;
    x << 0, -500, 0, 0, 0.1, 0;
    GlideslopeParams params;
    params.k = 0.001;

    GlideslopeCheck result = check_glideslope_violation(x, params);

    EXPECT_FALSE(result.violated);
    EXPECT_NEAR(result.v_approach, 0.1, 1e-10);
}

TEST(CheckGlideslopeViolation, FastApproachViolated) {
    Vec6 x;
    x << 0, -500, 0, 0, 10.0, 0;
    GlideslopeParams params;
    params.k = 0.001;

    GlideslopeCheck result = check_glideslope_violation(x, params);

    EXPECT_TRUE(result.violated);
    EXPECT_LT(result.margin, 0);
}

TEST(CheckGlideslopeViolation, MovingAwayNotViolated) {
    Vec6 x;
    x << 0, -500, 0, 0, -1.0, 0;
    GlideslopeParams params;
    params.k = 0.001;

    GlideslopeCheck result = check_glideslope_violation(x, params);

    EXPECT_FALSE(result.violated);
    EXPECT_LT(result.v_approach, 0);
}

// ----------------------------------------------------------------------------
// apply_glideslope_constraint
// ----------------------------------------------------------------------------

TEST(ApplyGlideslope, NoNanAtOrigin) {
    Vec6 x;
    x << 0, 0, 0, 0, 0, 0;
    Vec3 u{0, 0.01, 0};
    double dt = 1.0;
    GlideslopeParams params;
    params.k = 0.001;
    params.min_range = 10.0;

    Vec3 u_clamped = apply_glideslope_constraint(u, x, dt, params);

    EXPECT_THAT(u_clamped.norm(), Not(testing::IsNan()));
}

TEST(ApplyGlideslope, NoConstraintWhenBelowLimit) {
    Vec6 x;
    x << 0, -500, 0, 0, 0.1, 0;
    Vec3 u{0, 0.001, 0};
    double dt = 1.0;
    GlideslopeParams params;
    params.k = 0.001;
    params.min_range = 10.0;

    Vec3 u_clamped = apply_glideslope_constraint(u, x, dt, params);

    EXPECT_NEAR((u_clamped - u).norm(), 0.0, 1e-10);
}

TEST(ApplyGlideslope, ClampedWhenExceedsLimit) {
    Vec6 x;
    x << 0, -150, 0, 0, 1.0, 0;
    Vec3 u{0, 0.05, 0};
    double dt = 1.0;
    GlideslopeParams params;
    params.k = 0.001;
    params.min_range = 10.0;

    Vec3 u_clamped = apply_glideslope_constraint(u, x, dt, params);

    EXPECT_LT(u_clamped(1), u(1));
}

TEST(ApplyGlideslope, PreservesLateralComponent) {
    Vec6 x;
    x << 0, -200, 0, 0, 0.19, 0;
    Vec3 u{0.05, 0.1, 0.05};
    double dt;
    GlideslopeParams params;
    params.k = 0.001;
    params.min_range = 10.0;
    params.approach_axis = Vec3{0, -1, 0};

    Vec3 u_clamped = apply_glideslope_constraint(u, x, dt, params);

    EXPECT_NEAR(u_clamped(0), u(0), 1e-10);
    EXPECT_NEAR(u_clamped(2), u(2), 1e-10);
}

TEST(ApplyGlideslope, IgnoresWhenTooClose) {
    Vec6 x;
    x << 0, -5, 0, 0, 0.5, 0;
    Vec3 u{0, 0.1, 0};
    double dt = 1.0;
    GlideslopeParams params;
    params.k = 0.001;
    params.min_range = 10.0;
    
    Vec3 u_clamped = apply_glideslope_constraint(u, x, dt, params);

    EXPECT_NEAR((u_clamped - u).norm(), 0.0, 1e-10);
}

// ----------------------------------------------------------------------------
// saturate_control
// ----------------------------------------------------------------------------

TEST(SaturateControl, NoSaturationBelowLimit) {
    Vec3 u{0.005, 0.005, 0.005};
    double u_max = 0.01;

    Vec3 u_sat = saturate_control(u, u_max);

    EXPECT_NEAR((u_sat - u).norm(), 0.0, 1e-10);
}

TEST(SaturateControl, SaturatesAboveLimit) {
    Vec3 u{0.1, 0.0, 0.0};
    double u_max = 0.01;

    Vec3 u_sat = saturate_control(u, u_max);

    EXPECT_NEAR(u_sat.norm(), u_max, 1e-10);
}

TEST(SaturateControl, PreservesDirection) {
    Vec3 u{0.1, 0.2, 0.3};
    double u_max = 0.01;

    Vec3 u_sat = saturate_control(u, u_max);
    Vec3 u_dir = u.normalized();
    Vec3 u_sat_dir = u_sat.normalized();

    EXPECT_NEAR((u_dir - u_sat_dir).norm(), 0.0, 1e-10);
}

// ----------------------------------------------------------------------------
// run_approach_guidance
// ----------------------------------------------------------------------------

TEST(RunApproachGuidance, ReachesTargetFromVbar) {
    Vec6 x0;
    x0 << 0, -500, 0, 0, 0, 0;

    double n = OrbitalParams(420e3).mean_motion();
    ApproachParams params;
    params.Q = Mat6::Identity();
    params.R = Mat3::Identity();

    ApproachResult result = run_approach_guidance(x0, n, params);

    std::cout << "Success: " << result.success << std::endl;
    std::cout << "Final Range: " << result.final_range << " | Success Range: " << params.success_range << std::endl;
    std::cout << "Final Velocity: " << result.final_velocity << " | Success Velocity: " << params.success_velocity << std::endl;

    EXPECT_TRUE(result.success);
    EXPECT_LT(result.final_range, params.success_range);
    EXPECT_LT(result.final_velocity, params.success_velocity);
}

TEST(RunApproachGuidance, ReachesTargetWithCorridorRecovery) {
    Vec6 x0;
    x0 << 300, -300, 0, 0, 0, 0;

    double n = OrbitalParams(420e3).mean_motion();
    ApproachParams params;

    ApproachResult result = run_approach_guidance(x0, n, params);

    EXPECT_TRUE(result.success);
}

TEST(RunApproachGuidance, ReachesTargetThreeDimensional) {
    Vec6 x0;
    x0 << 200, -400, 150, 0, 0, 0;

    double n = OrbitalParams(420e3).mean_motion();
    ApproachParams params;

    ApproachResult result = run_approach_guidance(x0, n, params);

    EXPECT_TRUE(result.success);
}

TEST(RunApproachGuidance, TimeoutWhenUnreachable) {
    Vec6 x0;
    x0 << 0, -5000, 0, 0, 0, 0;

    double n = OrbitalParams(420e3).mean_motion();
    ApproachParams params;
    params.timeout = 100.0;
    params.u_max = 0.0001;

    ApproachResult result = run_approach_guidance(x0, n, params);

    EXPECT_FALSE(result.success);
    EXPECT_GE(result.duration, params.timeout - params.dt);
}

TEST(RunApproachGuidance, RecordsSaturation) {
    Vec6 x0;
    x0 << 0, -1000, 0, 0, 0, 0;

    double n = OrbitalParams(420e3).mean_motion();
    ApproachParams params;
    params.u_max = 0.001;
    params.Q = Mat6::Identity() * 100;

    ApproachResult result = run_approach_guidance(x0, n, params);

    EXPECT_GT(result.saturation_count, 0);
}

TEST(RunApproachGuidance, AccumulatesDeltaV) {
    Vec6 x0;
    x0 << 0, -500, 0, 0, 0, 0;

    double n = OrbitalParams(420e3).mean_motion();
    ApproachParams params;

    ApproachResult result = run_approach_guidance(x0, n, params);

    EXPECT_GT(result.total_dv, 0);
}

TEST(RunApproachGuidance, TrajectoryCountMatchesNumPoints) {
    Vec6 x0;
    x0 << 0, -500, 0, 0, 0, 0;

    double n = OrbitalParams(420e3).mean_motion();
    ApproachParams params;

    ApproachResult result = run_approach_guidance(x0, n, params);

    EXPECT_EQ(result.trajectory.count, result.num_points);
    EXPECT_GT(result.num_points, 0);
}