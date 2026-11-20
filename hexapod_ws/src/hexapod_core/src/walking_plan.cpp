#include "hexapod_core/walking_plan.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace hexapod_core {
namespace {

tripod_sequence_initial_state end_state_of(const tripod_cycle_solution& cycle) {
    if (cycle.segments.empty()) {
        throw std::logic_error("Successful walking cycle is missing its joint segments");
    }
    const auto& last = cycle.segments.back();
    return {cycle.foot_cycle.final_pose, cycle.foot_cycle.final_feet, last.nominal_angles, last.branches};
}

void require_recovery_parameters(const stance_recovery_parameters& recovery) {
    if (recovery.intervals_per_stage == 0 || !std::isfinite(recovery.lift_height_m) || recovery.lift_height_m <= 0.0 ||
        !std::isfinite(recovery.foot_tolerance_m) || recovery.foot_tolerance_m < 0.0) {
        throw std::invalid_argument("Invalid stance recovery intervals, lift, or foot tolerance");
    }
    (void)make_tripod_cycle_boundaries(recovery.timing);
}

double distance(const vector3& lhs, const vector3& rhs) {
    return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}

std::optional<leg_id> first_stance_mismatch(const leg_kinematics& kinematics, const tripod_sequence_initial_state& state, const stance_recovery_parameters& recovery, double ground_height_m) {
    if (!std::isfinite(ground_height_m)) {
        throw std::invalid_argument("Planning ground height must be finite");
    }
    const double standing_z_in_body = ground_height_m - state.body_pose.body_origin_in_planning_frame.z;
    for (const auto leg : leg_order) {
        const auto index = leg_index(leg);
        const auto& xy = recovery.standing_feet_in_body[index];
        const vector3 target_in_body{xy.x_m, xy.y_m, standing_z_in_body};
        // 采用与落脚预测相同的水平变换和精确地面高度。
        auto target_in_planning = body_to_planning_frame(state.body_pose, {xy.x_m, xy.y_m, 0.0});
        target_in_planning.z = ground_height_m;

        const auto nominal_in_body = kinematics.forward_in_body_frame(leg, state.reference_angles[index]);
        const double nominal_error = distance(nominal_in_body, target_in_body);
        const double anchor_error = distance(state.feet.stance_points()[index], target_in_planning);
        if (!std::isfinite(nominal_error) || !std::isfinite(anchor_error)) {
            throw std::overflow_error("Stance foot error is not finite");
        }
        if (state.feet.swings()[index] || nominal_error > recovery.foot_tolerance_m || anchor_error > recovery.foot_tolerance_m) {
            return leg;
        }
    }
    return std::nullopt;
}

}  // namespace

walking_planner::walking_planner(robot_model model) : sequence_planner_(model), cycle_planner_(model), kinematics_(std::move(model)) {}

walking_plan_result walking_planner::plan(const tripod_sequence_schedule_request& request, const std::optional<stance_recovery_parameters>& recovery) const {
    if (request.cycles.empty() && !recovery) {
        throw std::invalid_argument("Walking plan requires at least one cycle or stance recovery");
    }
    if (recovery) {
        require_recovery_parameters(*recovery);
    }

    tripod_sequence_solution sequence;
    auto tail = request.initial_state;
    std::vector<tripod_sequence_initial_state> cycle_end_states;
    if (!request.cycles.empty()) {
        auto walked = sequence_planner_.plan(request);
        if (walked.status != tripod_sequence_status::checks_passed) {
            return {walking_plan_status::walking_cycle_failed, std::move(walked), {}, {}, {}};
        }
        if (!walked.solution || walked.solution->cycles.size() != request.cycles.size()) {
            throw std::logic_error("Successful walking sequence has the wrong cycle count");
        }
        sequence = std::move(*walked.solution);
        cycle_end_states.reserve(sequence.cycles.size());
        for (const auto& cycle : sequence.cycles) {
            cycle_end_states.push_back(end_state_of(cycle));
        }
        tail = cycle_end_states.back();
    }

    bool recovery_added = false;
    if (recovery && first_stance_mismatch(kinematics_, tail, *recovery, request.ground_height_m)) {
        const tripod_foot_cycle_request feet{
            recovery->timing, recovery->intervals_per_stage, tail.body_pose, {0.0, 0.0, 0.0},
            tail.feet, recovery->standing_feet_in_body, request.ground_height_m, recovery->lift_height_m
        };
        const double reference_tolerance_m = request.cycles.empty() ? request.initial_foot_tolerance_m : request.continuation_foot_tolerance_m;
        auto recovered = cycle_planner_.plan({
            feet, tail.reference_angles, tail.remembered_branches, request.selection_options,
            request.max_joint_speed_rad_s, request.path_interval_count, reference_tolerance_m,
            request.clearance_policy
        });
        if (recovered.status != tripod_cycle_status::checks_passed) {
            return {walking_plan_status::recovery_cycle_failed, {}, std::move(recovered), {}, {}};
        }
        if (!recovered.solution) {
            throw std::logic_error("Successful stance recovery has no joint solution");
        }
        tail = end_state_of(*recovered.solution);
        if (const auto leg = first_stance_mismatch(kinematics_, tail, *recovery, request.ground_height_m)) {
            return {walking_plan_status::recovery_terminal_mismatch, {}, {}, leg, {}};
        }
        sequence.cycles.push_back(std::move(*recovered.solution));
        recovery_added = true;
    }

    auto timeline = sequence.cycles.empty() ? std::vector<tripod_timed_segment>{} : make_tripod_sequence_timeline(sequence);
    return {walking_plan_status::checks_passed, {}, {}, {}, walking_plan_solution{
        std::move(timeline), std::move(tail), std::move(cycle_end_states), request.cycles.size(), recovery_added
    }};
}

}  // namespace hexapod_core
