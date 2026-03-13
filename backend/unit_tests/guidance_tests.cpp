/**
 * @file guidance_tests.cpp
 * @brief Unit tests for guidance algorithm
 */

#include <gtest/gtest.h>
#include <cmath>

#include "gnc_algorithms.hpp"

using namespace relnav;

// -----------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------

static constexpr double TOL = 1e-6;

static GuidanceConfig default_test_config() {
    GuidanceConfig config;
    config.k = 0.001;
    config.approach_axis = Vec3{0, -1, 0};
    config.corridor_angle = 0.175;
    config.min_range = 10.0;
    config.terminal_range = 30.0;
    config.success_range = 5.0;
    config.success_velocity = 0.05;
    config.timeout = 6000.0;
    config.waypoint_advance_radius = 25.0;
    config.waypoint_angular_spacing = 10.0 * M_PI / 180.0;
    config.target_entry_range = 50.0;
    config.min_waypoints = 2;
    
    return config;
}

// -----------------------------------------------------------------
// rodrigues_rotate
// -----------------------------------------------------------------

TEST(RodriguesRotate, ZeroAngleReturnsOriginal) {
    Vec3 v{1, 0, 0};
    Vec3 k{0, 0, 1};
    Vec3 result = rodrigues_rotate(v, k, 0.0);

    EXPECT_NEAR(result(0), 1.0, TOL);
    EXPECT_NEAR(result(1), 0.0, TOL);
    EXPECT_NEAR(result(2), 0.0, TOL);
}

TEST(RodriguesRotate, Rotate90DegreesAroundZ) {
    Vec3 v{1, 0, 0};
    Vec3 k{0, 0, 1};
    Vec3 result = rodrigues_rotate(v, k, M_PI / 2.0);

    EXPECT_NEAR(result(0), 0.0, TOL);
    EXPECT_NEAR(result(1), 1.0, TOL);
    EXPECT_NEAR(result(2), 0.0, TOL);
}

TEST(RodriguesRotate, Rotate180Degrees) {
    Vec3 v{1, 0, 0};
    Vec3 k{0, 0, 1};
    Vec3 result = rodrigues_rotate(v, k, M_PI);

    EXPECT_NEAR(result(0), -1.0, TOL);
    EXPECT_NEAR(result(1), 0.0, TOL);
    EXPECT_NEAR(result(2), 0.0, TOL);
}

TEST(RodriguesRotate, PreservesNorm) {
    Vec3 v{3.0, 4.0, 5.0};
    Vec3 k = Vec3{1, 1, 1}.normalized();
    Vec3 result = rodrigues_rotate(v, k, 1.23);

    EXPECT_NEAR(result.norm(), v.norm(), TOL);
}

TEST(RodriguesRotate, RotateAroundParallelAxisNoChange) {
    Vec3 v{0, 0, 5.0};
    Vec3 k{0, 0, 1};
    Vec3 result = rodrigues_rotate(v, k, 1.5);

    EXPECT_NEAR(result(0), 0.0, TOL);
    EXPECT_NEAR(result(1), 0.0, TOL);
    EXPECT_NEAR(result(2), 5.0, TOL);
}

// -----------------------------------------------------------------
// find_perpendicular
// -----------------------------------------------------------------

TEST(FindPerpendicular, ResultIsOrthogonal) {
    Vec3 v{0, -1, 0};
    Vec3 perp = find_perpendicular(v);
    EXPECT_NEAR(v.dot(perp), 0.0, TOL);
    EXPECT_NEAR(perp.norm(), 1.0, TOL);
}

TEST(FindPerpendicular, WorksForXAxis) {
    Vec3 v{1, 0, 0};
    Vec3 perp = find_perpendicular(v);
    EXPECT_NEAR(v.dot(perp), 0.0, TOL);
    EXPECT_NEAR(perp.norm(), 1.0, TOL);
}

TEST(FindPerpendicular, WorksForArbitraryVector) {
    Vec3 v = Vec3{3, -2, 7}.normalized();
    Vec3 perp = find_perpendicular(v);
    EXPECT_NEAR(v.dot(perp), 0.0, TOL);
    EXPECT_NEAR(perp.norm(), 1.0, TOL);
}

// -----------------------------------------------------------------
// compute_rotation_axis
// -----------------------------------------------------------------

TEST(ComputeRotationAxis, ResultIsUnit) {
    Vec3 pos{500, 0, 0};
    Vec3 axis{0, -1, 0};
    Vec3 rot = compute_rotation_axis(pos, axis);

    EXPECT_NEAR(rot.norm(), 1.0, TOL);
}

TEST(ComputeRotationAxis, PerpendicularToBoth) {
    Vec3 pos{500, 0, 0};
    Vec3 axis{0, -1, 0};
    Vec3 rot = compute_rotation_axis(pos, axis);

    EXPECT_NEAR(rot.dot(pos.normalized()), 0.0, TOL);
    EXPECT_NEAR(rot.dot(axis), 0.0, TOL);
}

TEST(ComputeRotationAxis, DegenerateCaseAligned) {
    Vec3 pos{0, -500, 0};
    Vec3 axis{0, -1, 0};
    Vec3 rot = compute_rotation_axis(pos, axis);

    EXPECT_NEAR(rot.norm(), 1.0, TOL);
    EXPECT_FALSE(std::isnan(rot(0)));
}

TEST(ComputeRotationAxis, DegenerateCaseAntiAligned) {
    Vec3 pos{0, 500, 0};
    Vec3 axis{0, -1, 0};
    Vec3 rot = compute_rotation_axis(pos, axis);

    EXPECT_NEAR(rot.norm(), 1.0, TOL);
    EXPECT_FALSE(std::isnan(rot(0)));
}

// -----------------------------------------------------------------
// compute_angle_to_corridor
// -----------------------------------------------------------------

TEST(AngleToCorridor, InsideCorridorReturnsZero) {
    GuidanceConfig config = default_test_config();
    Vec3 pos{0, -1000, 0};

    EXPECT_NEAR(compute_angle_to_corridor(pos, config), 0.0, TOL);
}

TEST(AngleToCorridor, JustInsideReturnsZero) {
    GuidanceConfig config = default_test_config();
    double angle = 9.0 * M_PI / 180.0; 
    Vec3 pos{std::sin(angle) * 1000, -std::cos(angle) * 1000, 0};

    EXPECT_NEAR(compute_angle_to_corridor(pos, config), 0.0, TOL);
}

TEST(AngleToCorridor, OutsideCorridorReturnsOffset) {
    GuidanceConfig config = default_test_config();
    Vec3 pos{1000, 0, 0};
    double expected = M_PI / 2.0 - config.corridor_angle;

    EXPECT_NEAR(compute_angle_to_corridor(pos, config), expected, 1e-3);
}

TEST(AngleToCorridor, FullyOppositeReturnsLargeAngle) {
    GuidanceConfig config = default_test_config();
    Vec3 pos{0, 1000, 0};
    double expected = M_PI - config.corridor_angle;

    EXPECT_NEAR(compute_angle_to_corridor(pos, config), expected, 1e-3);
}

TEST(AngleToCorridor, ZeroPositionReturnsZero) {
    GuidanceConfig config = default_test_config();
    Vec3 pos{0, 0, 0};

    EXPECT_NEAR(compute_angle_to_corridor(pos, config), 0.0, TOL);
}

// -----------------------------------------------------------------
// generate_entry_arc
// -----------------------------------------------------------------

TEST(GenerateEntryArc, InsideCorridorReturnsEmpty) {
    GuidanceConfig config = default_test_config();
    Vec3 pos{0, -1000, 0};
    auto waypoints = generate_entry_arc(pos, config);

    EXPECT_TRUE(waypoints.empty());
}

TEST(GenerateEntryArc, OutsideCorridorProducesWaypoints) {
    GuidanceConfig config = default_test_config();
    Vec3 pos{1000, 0, 0}; 
    auto waypoints = generate_entry_arc(pos, config);

    EXPECT_GE(static_cast<int>(waypoints.size()), config.min_waypoints);
}

TEST(GenerateEntryArc, WaypointCountProportionalToAngle) {
    GuidanceConfig config = default_test_config();

    Vec3 pos_small{std::sin(0.3) * 1000, -std::cos(0.3) * 1000, 0};
    auto wp_small = generate_entry_arc(pos_small, config);
 
    Vec3 pos_large{1000, 0, 0};
    auto wp_large = generate_entry_arc(pos_large, config);

    EXPECT_GT(wp_large.size(), wp_small.size());
}

TEST(GenerateEntryArc, LastWaypointNearCorridorEdge) {
    GuidanceConfig config = default_test_config();
    Vec3 pos{1000, 0, 0};
    auto waypoints = generate_entry_arc(pos, config);
    ASSERT_FALSE(waypoints.empty());

    Vec3 last = waypoints.back();
    double angle = glideslope_approach_angle(last, config.approach_axis);

    EXPECT_NEAR(angle, config.corridor_angle, 0.05);
}

TEST(GenerateEntryArc, WaypointsDecreaseInRange) {
    GuidanceConfig config = default_test_config();
    Vec3 pos{1000, 0, 0};
    auto waypoints = generate_entry_arc(pos, config);
    ASSERT_GE(waypoints.size(), 2u);

    double prev_range = pos.norm();

    for (const auto &wp : waypoints) {
        double range = wp.norm();
        EXPECT_LE(range, prev_range + TOL);
        prev_range = range;
    }
}

TEST(GenerateEntryArc, LastWaypointRangeNearTarget) {
    GuidanceConfig config = default_test_config();
    Vec3 pos{1000, 0, 0};
    auto waypoints = generate_entry_arc(pos, config);

    ASSERT_FALSE(waypoints.empty());

    double last_range = waypoints.back().norm();
    double expected = std::max(config.target_entry_range, config.terminal_range + 10.0);
    
    EXPECT_NEAR(last_range, expected, 5.0);
}

TEST(GenerateEntryArc, WaypointsAreInPlane) {
    GuidanceConfig config = default_test_config();
    Vec3 pos{1000, 0, 0};
    Vec3 rot_axis = compute_rotation_axis(pos, config.approach_axis);
    auto waypoints = generate_entry_arc(pos, config);

    for (const auto &wp : waypoints) {
        EXPECT_NEAR(wp.normalized().dot(rot_axis), 0.0, 1e-3);
    }
}

TEST(GenerateEntryArc, MinWaypointsEnforced) {
    GuidanceConfig config = default_test_config();
    config.min_waypoints = 5;
    double small_angle = config.corridor_angle + 0.05;
    Vec3 pos{std::sin(small_angle) * 1000, -std::cos(small_angle) * 1000, 0};
    auto waypoints = generate_entry_arc(pos, config);

    EXPECT_GE(static_cast<int>(waypoints.size()), 5);
}

TEST(GenerateEntryArc, CloseRangeDoesNotIncreaseRange) {
    GuidanceConfig config = default_test_config();
    config.target_entry_range = 50.0;
    Vec3 pos{40, 0, 0};
    auto waypoints = generate_entry_arc(pos, config);

    for (const auto &wp : waypoints) {
        EXPECT_LE(wp.norm(), pos.norm() + TOL);
    }
}

TEST(GenerateEntryArc, ThreeDimensionalPosition) {
    GuidanceConfig config = default_test_config();
    Vec3 pos{500, 0, 500}; 
    auto waypoints = generate_entry_arc(pos, config);

    EXPECT_FALSE(waypoints.empty());

    Vec3 last = waypoints.back();
    double angle = glideslope_approach_angle(last, config.approach_axis);

    EXPECT_NEAR(angle, config.corridor_angle, 0.05);
}

// -----------------------------------------------------------------
// determine_phase
// -----------------------------------------------------------------

TEST(DeterminePhase, SuccessCondition) {
    GuidanceConfig config = default_test_config();
    GuidanceState state;
    state.phase = GuidancePhase::TERMINAL;
    state.elapsed_time = 100.0;

    Vec6 x;
    x << 2, 0, 0, 0.01, 0, 0; 

    determine_phase(x, state, config);
    EXPECT_EQ(state.phase, GuidancePhase::COMPLETE);
}

TEST(DeterminePhase, TimeoutFromAnyPhase) {
    GuidanceConfig config = default_test_config();
    GuidanceState state;
    state.phase = GuidancePhase::CORRIDOR_TRAVERSE;
    state.elapsed_time = 7000.0;

    Vec6 x;
    x << 500, 0, 0, 0, 0, 0;

    determine_phase(x, state, config);
    EXPECT_EQ(state.phase, GuidancePhase::ABORT);
}

TEST(DeterminePhase, EntryToTraverseWhenInsideCorridor) {
    GuidanceConfig config = default_test_config();
    GuidanceState state;
    state.phase = GuidancePhase::CORRIDOR_ENTRY;
    state.elapsed_time = 100.0;
    state.waypoints = {};
    state.current_waypoint_idx = 0;

    Vec6 x;
    x << 0, -500, 0, 0, 0, 0;

    determine_phase(x, state, config);
    EXPECT_EQ(state.phase, GuidancePhase::CORRIDOR_TRAVERSE);
}

TEST(DeterminePhase, EntryToTerminalWhenCloseAndInside) {
    GuidanceConfig config = default_test_config();
    GuidanceState state;
    state.phase = GuidancePhase::CORRIDOR_ENTRY;
    state.elapsed_time = 100.0;
    state.waypoints = {};
    state.current_waypoint_idx = 0;

    Vec6 x;
    x << 0, -20, 0, 0, 0, 0;

    determine_phase(x, state, config);
    EXPECT_EQ(state.phase, GuidancePhase::TERMINAL);
}

TEST(DeterminePhase, EntryStaysWhenOutsideWithWaypoints) {
    GuidanceConfig config = default_test_config();
    GuidanceState state;
    state.phase = GuidancePhase::CORRIDOR_ENTRY;
    state.elapsed_time = 100.0;
    state.waypoints = {Vec3{100, -100, 0}, Vec3{50, -200, 0}};
    state.current_waypoint_idx = 0;

    Vec6 x;
    x << 500, 0, 0, 0, 0, 0;

    determine_phase(x, state, config);
    EXPECT_EQ(state.phase, GuidancePhase::CORRIDOR_ENTRY);
}

TEST(DeterminePhase, TraverseToTerminal) {
    GuidanceConfig config = default_test_config();
    GuidanceState state;
    state.phase = GuidancePhase::CORRIDOR_TRAVERSE;
    state.elapsed_time = 100.0;

    Vec6 x;
    x << 0, -20, 0, 0, -0.1, 0; 

    determine_phase(x, state, config);
    EXPECT_EQ(state.phase, GuidancePhase::TERMINAL);
}

TEST(DeterminePhase, TraverseToEntryWhenDriftedOut) {
    GuidanceConfig config = default_test_config();
    GuidanceState state;
    state.phase = GuidancePhase::CORRIDOR_TRAVERSE;
    state.elapsed_time = 100.0;

    Vec6 x;
    x << 500, 0, 0, 0, 0, 0;

    determine_phase(x, state, config);
    EXPECT_EQ(state.phase, GuidancePhase::CORRIDOR_ENTRY);
    EXPECT_FALSE(state.waypoints.empty());
    EXPECT_EQ(state.current_waypoint_idx, 0);
}

TEST(DeterminePhase, TerminalStaysTerminal) {
    GuidanceConfig config = default_test_config();
    GuidanceState state;
    state.phase = GuidancePhase::TERMINAL;
    state.elapsed_time = 100.0;

    Vec6 x;
    x << 0, -15, 0, 0, -0.1, 0;

    determine_phase(x, state, config);
    EXPECT_EQ(state.phase, GuidancePhase::TERMINAL);
}

TEST(DeterminePhase, CompleteStaysComplete) {
    GuidanceConfig config = default_test_config();
    GuidanceState state;
    state.phase = GuidancePhase::COMPLETE;
    state.elapsed_time = 100.0;

    Vec6 x;
    x << 500, 0, 0, 1, 0, 0;

    determine_phase(x, state, config);
    EXPECT_EQ(state.phase, GuidancePhase::COMPLETE);
}

// -----------------------------------------------------------------
// try_advance_waypoint
// -----------------------------------------------------------------

TEST(TryAdvanceWaypoint, AdvancesWhenClose) {
    GuidanceConfig config = default_test_config();
    GuidanceState state;
    state.waypoints = {Vec3{100, 0, 0}, Vec3{50, -50, 0}};
    state.current_waypoint_idx = 0;

    Vec3 pos{105, 0, 0}; 

    bool advanced = try_advance_waypoint(pos, state, config);
    EXPECT_TRUE(advanced);
    EXPECT_EQ(state.current_waypoint_idx, 1);
}

TEST(TryAdvanceWaypoint, DoesNotAdvanceWhenFar) {
    GuidanceConfig config = default_test_config();
    GuidanceState state;
    state.waypoints = {Vec3{100, 0, 0}, Vec3{50, -50, 0}};
    state.current_waypoint_idx = 0;

    Vec3 pos{200, 0, 0}; 

    bool advanced = try_advance_waypoint(pos, state, config);
    EXPECT_FALSE(advanced);
    EXPECT_EQ(state.current_waypoint_idx, 0);
}

TEST(TryAdvanceWaypoint, DoesNotAdvancePastEnd) {
    GuidanceConfig config = default_test_config();
    GuidanceState state;
    state.waypoints = {Vec3{100, 0, 0}};
    state.current_waypoint_idx = 1; 

    Vec3 pos{100, 0, 0};

    bool advanced = try_advance_waypoint(pos, state, config);
    EXPECT_FALSE(advanced);
    EXPECT_EQ(state.current_waypoint_idx, 1);
}

TEST(TryAdvanceWaypoint, EmptyWaypoints) {
    GuidanceConfig config = default_test_config();
    GuidanceState state;
    state.current_waypoint_idx = 0;

    Vec3 pos{0, 0, 0};

    bool advanced = try_advance_waypoint(pos, state, config);
    EXPECT_FALSE(advanced);
}

// -----------------------------------------------------------------
// update_guidance
// -----------------------------------------------------------------

TEST(UpdateGuidance, EntryOutputHasWaypointReference) {
    GuidanceConfig config = default_test_config();
    Vec3 pos{1000, 0, 0};
    GuidanceState state = initialize_guidance(pos, config);

    Vec6 x;
    x << 1000, 0, 0, 0, 0, 0;

    GuidanceOutput output = update_guidance(x, state, config, 1.0);
    EXPECT_EQ(output.phase, GuidancePhase::CORRIDOR_ENTRY);
    EXPECT_TRUE(output.glideslope_active);
    EXPECT_TRUE(output.corridor_active);
    EXPECT_GE(output.waypoint_idx, 0);
    EXPECT_GT(output.x_reference.head<3>().norm(), 1.0);
}

TEST(UpdateGuidance, TraverseOutputReferencesOrigin) {
    GuidanceConfig config = default_test_config();

    GuidanceState state;
    state.phase = GuidancePhase::CORRIDOR_TRAVERSE;
    state.elapsed_time = 0.0;

    Vec6 x;
    x << 0, -500, 0, 0, 0, 0;

    GuidanceOutput output = update_guidance(x, state, config, 1.0);
    EXPECT_EQ(output.phase, GuidancePhase::CORRIDOR_TRAVERSE);
    EXPECT_TRUE(output.glideslope_active);
    EXPECT_NEAR(output.x_reference.norm(), 0.0, TOL);
}

TEST(UpdateGuidance, TerminalOutputNoCorridorConstraint) {
    GuidanceConfig config = default_test_config();

    GuidanceState state;
    state.phase = GuidancePhase::TERMINAL;
    state.elapsed_time = 0.0;

    Vec6 x;
    x << 0, -15, 0, 0, -0.1, 0;

    GuidanceOutput output = update_guidance(x, state, config, 1.0);
    EXPECT_EQ(output.phase, GuidancePhase::TERMINAL);
    EXPECT_TRUE(output.glideslope_active);
    EXPECT_FALSE(output.corridor_active);
}

TEST(UpdateGuidance, TimeAccumulates) {
    GuidanceConfig config = default_test_config();

    GuidanceState state;
    state.phase = GuidancePhase::CORRIDOR_TRAVERSE;
    state.elapsed_time = 0.0;

    Vec6 x;
    x << 0, -500, 0, 0, 0, 0;

    update_guidance(x, state, config, 1.0);
    update_guidance(x, state, config, 1.0);
    update_guidance(x, state, config, 1.0);

    EXPECT_NEAR(state.elapsed_time, 3.0, TOL);
}

TEST(UpdateGuidance, CompleteOutputNoConstraints) {
    GuidanceConfig config = default_test_config();

    GuidanceState state;
    state.phase = GuidancePhase::TERMINAL;
    state.elapsed_time = 0.0;

    Vec6 x;
    x << 0, -3, 0, 0, 0.01, 0;

    GuidanceOutput output = update_guidance(x, state, config, 1.0);
    EXPECT_EQ(output.phase, GuidancePhase::COMPLETE);
    EXPECT_FALSE(output.glideslope_active);
    EXPECT_FALSE(output.corridor_active);
}