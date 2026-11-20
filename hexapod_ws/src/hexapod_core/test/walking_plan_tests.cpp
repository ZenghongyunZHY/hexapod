#include "hexapod_core/walking_plan.hpp"
#include "hexapod_core/joint_mapper.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using namespace hexapod_core;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

double gap(const vector3& first, const vector3& second) {
    return std::hypot(first.x - second.x, first.y - second.y, first.z - second.z);
}

tripod_sequence_schedule_request preview_request(const robot_model& model) {
    const joint_mapper mapper(model);
    const leg_kinematics kinematics(model);
    const planar_body_pose pose{{0.025, -0.010, 0.0}, 0.2};
    std::array<leg_joint_angles, leg_count> angles{};
    std::array<vector3, leg_count> anchors{};
    std::array<stance_midpoint_reference, leg_count> standing_xy{};
    std::array<std::array<double, joints_per_leg>, leg_count> speed_limits{};
    for (const auto leg : leg_order) {
        const auto index = leg_index(leg);
        const auto& ids = model.leg(leg).servo_ids;
        const auto nominal = [&](std::size_t joint, double degrees) {
            return mapper.servo_position_to_joint_angle_rad(ids[joint], mapper.joint_angle_rad_to_servo_position(ids[joint], degrees_to_radians(degrees)));
        };
        angles[index] = {nominal(0, 0.0), nominal(1, 12.0), nominal(2, -72.0)};
        const auto foot = kinematics.forward_in_body_frame(leg, angles[index]);
        standing_xy[index] = {foot.x, foot.y};
        anchors[index] = body_to_planning_frame(pose, foot);
        speed_limits[index].fill(1.0);
    }
    return {
        {pose, gait_foot_state(anchors), angles, {}},
        {
            {{4.0, 0.625}, 10, {0.0005, 0.0, 0.0}, 0.003},
            {{3.0, 0.5}, 6, {0.001, 0.0002, 0.0}, 0.002},
            {{5.0, 0.75}, 4, {0.00025, 0.0, 0.0}, 0.004}
        },
        standing_xy, anchors.front().z,
        {knee_singularity_policy::reject, branch_continuity_policy::keep_reference_branch},
        speed_limits, 4, 1e-10, 0.001, foot_clearance_policy{0.002, 0.002, 0.002}
    };
}

stance_recovery_parameters recovery_for(const tripod_sequence_schedule_request& request) {
    return {{4.0, 0.625}, 10, 0.003, request.midpoint_references_in_body, 0.0001};
}

void verify_stance(const robot_model& model, const tripod_sequence_initial_state& state, const stance_recovery_parameters& recovery, double ground_height_m) {
    const leg_kinematics kinematics(model);
    const auto& pose = state.body_pose;
    const double standing_z = ground_height_m - pose.body_origin_in_planning_frame.z;
    for (const auto leg : leg_order) {
        const auto index = leg_index(leg);
        const auto& xy = recovery.standing_feet_in_body[index];
        const vector3 expected_body{xy.x_m, xy.y_m, standing_z};
        const auto expected_anchor = body_to_planning_frame(pose, expected_body);
        const auto nominal_foot = kinematics.forward_in_body_frame(leg, state.reference_angles[index]);
        require(!state.feet.swings()[index], "recovery must finish all swings");
        require(gap(state.feet.stance_points()[index], expected_anchor) < 1e-11, "recovery must land at the stance anchor in the final body pose");
        require(gap(nominal_foot, expected_body) <= recovery.foot_tolerance_m, "recovery must meet stance tolerance after quantization");
    }
}

void test_finite_walk_and_recovery(const robot_model& model) {
    const walking_planner planner(model);
    const auto request = preview_request(model);
    const auto recovery = recovery_for(request);
    const auto result = planner.plan(request, recovery);
    require(result.status == walking_plan_status::checks_passed && result.solution, "finite walk plus stance recovery must pass");
    const auto& plan = *result.solution;
    require(plan.full_cycle_count == 3 && plan.cycle_end_states.size() == 3 && plan.recovery_added, "only the three moving cycles are counted as completed travel");
    require(plan.timeline.size() == 108 && plan.timeline.front().start_time_s == 0.0
        && plan.timeline.back().end_time_s == 16.0, "timeline includes the 12 s walk and stationary 4 s recovery");
    for (std::size_t index = 1; index < plan.timeline.size(); ++index) {
        require(plan.timeline[index - 1].end_time_s == plan.timeline[index].start_time_s, "all timeline seams must be contiguous");
    }
    require(plan.timeline[67].cycle_index == 2 && plan.timeline[68].cycle_index == 3
        && plan.timeline[68].start_time_s == 12.0, "recovery gets a distinct cycle index at the final travel boundary");

    const joint_mapper mapper(model);
    const std::array<std::size_t, 3> boundary_segments{39, 51, 67};
    auto expected_pose = request.initial_state.body_pose;
    for (std::size_t index = 0; index < request.cycles.size(); ++index) {
        const auto& cycle = request.cycles[index];
        expected_pose = advance_body_pose(expected_pose, cycle.velocity, cycle.timing.period_s);
        const auto& saved = plan.cycle_end_states[index];
        require(gap(saved.body_pose.body_origin_in_planning_frame, expected_pose.body_origin_in_planning_frame) < 1e-12, "stored end state must match the corresponding cycle");
        for (const auto& swing : saved.feet.swings()) {
            require(!swing, "stored walking boundaries must be available for recovery");
        }
        const auto& positions = plan.timeline[boundary_segments[index]].nominal_positions;
        for (const auto leg : leg_order) {
            const auto& ids = model.leg(leg).servo_ids;
            const auto& angles = saved.reference_angles[leg_index(leg)];
            const std::array<double, joints_per_leg> saved_angles{angles.yaw_rad, angles.upper_leg_rad, angles.lower_leg_rad};
            for (std::size_t joint = 0; joint < joints_per_leg; ++joint) {
                const auto& position = positions[static_cast<std::size_t>(ids[joint].value() - 1)];
                require(position && mapper.servo_position_to_joint_angle_rad(ids[joint], *position) == saved_angles[joint],
                    "saved nominal reference must match the mapped end of the interrupted prefix");
            }
        }
    }
    require(gap(plan.final_state.body_pose.body_origin_in_planning_frame,
        plan.cycle_end_states.back().body_pose.body_origin_in_planning_frame) == 0.0
        && plan.final_state.body_pose.yaw_rad == plan.cycle_end_states.back().body_pose.yaw_rad,
        "stance recovery must preserve the final body's position and yaw");
    verify_stance(model, plan.final_state, recovery, request.ground_height_m);

    const auto travel_only = planner.plan(request);
    require(travel_only.solution && travel_only.solution->timeline.size() == 68 && !travel_only.solution->recovery_added, "continuous input can get a checked travel-only batch");
    auto stop_request = request;
    stop_request.cycles.clear();
    stop_request.initial_state = travel_only.solution->final_state;
    stop_request.initial_foot_tolerance_m = request.continuation_foot_tolerance_m;
    const auto stop_only = planner.plan(stop_request, recovery);
    require(stop_only.solution && stop_only.solution->full_cycle_count == 0
        && stop_only.solution->cycle_end_states.empty() && stop_only.solution->timeline.size() == 40,
        "known cycle boundary can be independently recovered without a walking cycle");
    verify_stance(model, stop_only.solution->final_state, recovery, request.ground_height_m);

    stop_request.initial_state = plan.cycle_end_states.front();
    const auto interrupted = planner.plan(stop_request, recovery);
    require(interrupted.solution && interrupted.solution->recovery_added && interrupted.solution->full_cycle_count == 0,
        "interruption at an earlier accepted boundary must use the completed prefix rather than the planned tail");
    require(gap(interrupted.solution->final_state.body_pose.body_origin_in_planning_frame,
        plan.cycle_end_states.front().body_pose.body_origin_in_planning_frame) == 0.0,
        "interruption recovery must keep the body pose reached by the completed prefix");
    verify_stance(model, interrupted.solution->final_state, recovery, request.ground_height_m);

    stop_request.initial_state = stop_only.solution->final_state;
    const auto already_standing = planner.plan(stop_request, recovery);
    require(already_standing.solution && already_standing.solution->timeline.empty()
        && !already_standing.solution->recovery_added, "settled stance must not schedule redundant swing cycles");
}

void test_failure_atomicity(const robot_model& model) {
    const walking_planner planner(model);
    auto request = preview_request(model);
    auto recovery = recovery_for(request);
    request.cycles[1].lift_height_m = 0.6;
    const auto walking_failure = planner.plan(request, recovery);
    require(walking_failure.status == walking_plan_status::walking_cycle_failed && !walking_failure.solution
        && walking_failure.walking_failure && walking_failure.walking_failure->failed_cycle_index == 1,
        "walking failure preserves its cycle diagnostic and returns no partial trajectory");

    request = preview_request(model);
    recovery.lift_height_m = 0.6;
    const auto recovery_failure = planner.plan(request, recovery);
    require(recovery_failure.status == walking_plan_status::recovery_cycle_failed && !recovery_failure.solution
        && recovery_failure.recovery_failure && recovery_failure.recovery_failure->failed_frame_index,
        "recovery planning failure discards the valid walking prefix and preserves its diagnostic");

    recovery = recovery_for(request);
    recovery.standing_feet_in_body.front().x_m += 0.00015;
    recovery.foot_tolerance_m = 1e-10;
    const auto terminal_failure = planner.plan(request, recovery);
    require(terminal_failure.status == walking_plan_status::recovery_terminal_mismatch && !terminal_failure.solution
        && terminal_failure.terminal_failed_leg == leg_id::right_front,
        "quantized nominal FK must independently pass the requested terminal tolerance");
}

}  // namespace

int main(int argc, char** argv) {
    try {
        require(argc == 2, "usage: walking_plan_tests robot_hardware.json");
        const auto model = robot_model::from_file(argv[1]);
        test_finite_walk_and_recovery(model);
        test_failure_atomicity(model);
        std::cout << "PASS: finite walking timeline, cycle handoff, recovery, and atomic failures.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
