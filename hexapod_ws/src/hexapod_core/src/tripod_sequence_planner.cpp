#include "hexapod_core/tripod_sequence_planner.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace hexapod_core {

namespace {

void require_sequence_settings(std::size_t cycle_count, double continuation_foot_tolerance_m) {
    if (cycle_count == 0) {
        throw std::invalid_argument("Tripod sequence requires at least one full cycle");
    }

    if (!std::isfinite(continuation_foot_tolerance_m) || continuation_foot_tolerance_m < 0.0) {
        throw std::invalid_argument("Continuation foot tolerance must be finite and nonnegative");
    }

    if (cycle_count > std::vector<tripod_cycle_solution>{}.max_size()) {
        throw std::length_error("Tripod sequence cycle count is too large");
    }
}

}  // namespace

tripod_sequence_planner::tripod_sequence_planner(robot_model model)
    : cycle_planner_(std::move(model)) {
}

tripod_sequence_result tripod_sequence_planner::plan(const tripod_sequence_request& request) const {
    require_sequence_settings(request.full_cycle_count, request.continuation_foot_tolerance_m);

    const auto& first = request.first_cycle;
    const auto& feet = first.foot_request;
    const tripod_sequence_cycle_parameters parameters{
        feet.timing,
        feet.intervals_per_stage,
        feet.velocity,
        feet.lift_height_m
    };

    return plan(tripod_sequence_schedule_request{
        {feet.initial_pose, feet.initial_feet, first.reference_angles, first.remembered_branches},
        std::vector<tripod_sequence_cycle_parameters>(request.full_cycle_count, parameters),
        feet.midpoint_references_in_body,
        feet.ground_height_m,
        first.selection_options,
        first.max_joint_speed_rad_s,
        first.path_interval_count,
        first.initial_foot_tolerance_m,
        request.continuation_foot_tolerance_m,
        first.clearance_policy
    });
}

tripod_sequence_result tripod_sequence_planner::plan(const tripod_sequence_schedule_request& request) const {
    require_sequence_settings(request.cycles.size(), request.continuation_foot_tolerance_m);

    std::vector<tripod_cycle_solution> cycles;
    cycles.reserve(request.cycles.size());
    auto state = request.initial_state;

    for (std::size_t index = 0; index < request.cycles.size(); ++index) {
        const auto& parameters = request.cycles[index];
        const double initial_tolerance_m = index == 0
            ? request.initial_foot_tolerance_m
            : request.continuation_foot_tolerance_m;

        const tripod_foot_cycle_request feet{
            parameters.timing,
            parameters.intervals_per_stage,
            state.body_pose,
            parameters.velocity,
            state.feet,
            request.midpoint_references_in_body,
            request.ground_height_m,
            parameters.lift_height_m
        };
        const tripod_cycle_request cycle_request{
            feet,
            state.reference_angles,
            state.remembered_branches,
            request.selection_options,
            request.max_joint_speed_rad_s,
            request.path_interval_count,
            initial_tolerance_m,
            request.clearance_policy
        };

        auto cycle = cycle_planner_.plan(cycle_request);
        if (cycle.status != tripod_cycle_status::checks_passed) {
            return {tripod_sequence_status::cycle_failed, index, std::move(cycle), std::nullopt};
        }

        if (!cycle.solution || cycle.solution->segments.empty()) {
            throw std::logic_error("Successful tripod cycle is missing its joint segments");
        }

        auto& accepted = *cycle.solution;
        const auto& last_segment = accepted.segments.back();
        state = {
            accepted.foot_cycle.final_pose,
            accepted.foot_cycle.final_feet,
            last_segment.nominal_angles,
            last_segment.branches
        };
        cycles.push_back(std::move(accepted));
    }

    return {
        tripod_sequence_status::checks_passed,
        std::nullopt,
        std::nullopt,
        tripod_sequence_solution{std::move(cycles)}
    };
}

}  // namespace hexapod_core
