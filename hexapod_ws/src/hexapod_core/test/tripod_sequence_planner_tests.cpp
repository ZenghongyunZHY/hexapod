#include "hexapod_core/tripod_sequence_planner.hpp"
#include "hexapod_core/tripod_sequence_timeline.hpp"
#include "hexapod_core/joint_mapper.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using namespace hexapod_core;

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

template <typename exception_type, typename function_type>
void require_throws(function_type function, const std::string& message) {
    try {
        function();
    } catch (const exception_type&) {
        return;
    }
    throw std::runtime_error("missing expected exception: " + message);
}

bool near(double actual, double expected) {
    return std::abs(actual - expected) <= 1e-11;
}

bool same_point(const vector3& left, const vector3& right) {
    return near(left.x, right.x) && near(left.y, right.y) && near(left.z, right.z);
}

bool same_pose(const planar_body_pose& left, const planar_body_pose& right) {
    return same_point(left.body_origin_in_planning_frame, right.body_origin_in_planning_frame)
        && near(left.yaw_rad, right.yaw_rad);
}

tripod_sequence_schedule_request example_schedule(const robot_model& model) {
    const joint_mapper mapper(model);
    const leg_kinematics kinematics(model);
    const planar_body_pose pose{{0.025, -0.010, 0.0}, 0.0};
    std::array<leg_joint_angles, leg_count> references{};
    std::array<leg_branch_state, leg_count> branches{};
    std::array<vector3, leg_count> anchors{};
    std::array<stance_midpoint_reference, leg_count> midpoint_references{};
    std::array<std::array<double, joints_per_leg>, leg_count> speeds{};
    const std::array<double, joints_per_leg> candidate{0.0, degrees_to_radians(12.0), degrees_to_radians(-72.0)};
    for (const auto leg : leg_order) {
        const auto index = leg_index(leg);
        std::array<double, joints_per_leg> angles{};
        for (std::size_t axis = 0; axis < joints_per_leg; ++axis) {
            const auto id = model.joint(leg, joint_order[axis]).binding.id;
            angles[axis] = mapper.servo_position_to_joint_angle_rad(id,
                mapper.joint_angle_rad_to_servo_position(id, candidate[axis]));
        }
        references[index] = {angles[0], angles[1], angles[2]};
        const auto point = kinematics.forward_in_body_frame(leg, references[index]);
        anchors[index] = body_to_planning_frame(pose, point);
        midpoint_references[index] = {point.x, point.y};
        speeds[index].fill(1.0);
    }

    // 离线候选与容差均为本测试输入。各周期的时长、采样数、占空比和抬脚高度各异。
    return {{pose, gait_foot_state(anchors), references, branches},
        {{{4.0, 0.625}, 10, {0.0005, 0.0, 0.0}, 0.003},
            {{3.0, 0.5}, 6, {0.001, 0.0002, 0.0}, 0.002},
            {{5.0, 0.75}, 4, {0.00025, 0.0, 0.0}, 0.004},
            {{2.5, 0.5625}, 8, {0.0, 0.0, 0.0}, 0.0}},
        midpoint_references, anchors[0].z,
        {knee_singularity_policy::reject, branch_continuity_policy::keep_reference_branch},
        speeds, 4, 1e-10, 0.001, foot_clearance_policy{0.002, 0.002, 0.002}};
}

tripod_cycle_request single_cycle_request(
    const tripod_sequence_schedule_request& schedule, std::size_t index
) {
    const auto& parameters = schedule.cycles[index];
    const auto& initial = schedule.initial_state;
    return {{parameters.timing, parameters.intervals_per_stage, initial.body_pose, parameters.velocity,
        initial.feet, schedule.midpoint_references_in_body, schedule.ground_height_m, parameters.lift_height_m},
        initial.reference_angles, initial.remembered_branches, schedule.selection_options,
        schedule.max_joint_speed_rad_s, schedule.path_interval_count,
        schedule.initial_foot_tolerance_m, schedule.clearance_policy};
}

void require_same_joint_solution(const body_ik_solution& actual, const body_ik_solution& expected) {
    for (std::size_t index = 0; index < leg_count; ++index) {
        const auto& left = actual.nominal_angles[index];
        const auto& right = expected.nominal_angles[index];
        require(near(left.yaw_rad, right.yaw_rad) && near(left.upper_leg_rad, right.upper_leg_rad)
            && near(left.lower_leg_rad, right.lower_leg_rad), "nominal angles match direct planning");
        require(actual.branches[index].radial == expected.branches[index].radial
            && actual.branches[index].knee == expected.branches[index].knee, "branch memory matches direct planning");
    }
    for (std::size_t joint = 0; joint < joint_count; ++joint) {
        const auto left = actual.targets.nominal_positions[joint];
        const auto right = expected.targets.nominal_positions[joint];
        require(left && right && left->value() == right->value(), "all nominal integers match direct planning");
        require(!actual.targets.commanded_positions[joint], "offline planning leaves command targets empty");
    }
}

void test_sequence_timeline(const tripod_sequence_solution& solution) {
    const auto timeline = make_tripod_sequence_timeline(solution);
    const std::array<std::size_t, 5> cycle_offsets{0, 40, 52, 68, 100};
    const std::array<double, 5> boundary_times{0.0, 4.0, 7.0, 12.0, 14.5};
    require(timeline.size() == cycle_offsets.back(), "one timeline row per joint segment");
    require(timeline.back().end_time_s == boundary_times.back(), "the fourth cycle ends at 14.5 s");
    for (std::size_t cycle = 0; cycle < solution.cycles.size(); ++cycle) {
        require(timeline[cycle_offsets[cycle]].start_time_s == boundary_times[cycle],
            "cycle starts at the sum of preceding cycle durations");
        for (std::size_t index = cycle_offsets[cycle]; index < cycle_offsets[cycle + 1]; ++index) {
            const auto& row = timeline[index];
            const std::size_t segment = index - cycle_offsets[cycle];
            const auto& expected = solution.cycles[cycle];
            require(row.cycle_index == cycle && row.end_frame_index == segment + 1,
                "each joint segment identifies its target frame");
            require(row.end_time_s == boundary_times[cycle] + expected.foot_cycle.frames[segment + 1].sample.time_s,
                "global end time comes from the corresponding target frame");
            require(row.end_time_s > row.start_time_s
                && (index == 0 || row.start_time_s == timeline[index - 1].end_time_s),
                "adjacent rows share a boundary and have positive duration");
            const auto& positions = expected.segments[segment].targets.nominal_positions;
            for (std::size_t joint = 0; joint < joint_count; ++joint) {
                require(row.nominal_positions[joint] && positions[joint]
                    && row.nominal_positions[joint]->value() == positions[joint]->value(),
                    "each row keeps the target positions in physical ID order");
            }
        }
    }

    require_throws<std::invalid_argument>([] {
        (void)make_tripod_sequence_timeline(tripod_sequence_solution{});
    }, "empty sequence");
    auto invalid = solution;
    invalid.cycles[0].segments.pop_back();
    require_throws<std::invalid_argument>([&] { (void)make_tripod_sequence_timeline(invalid); },
        "mismatched frame and segment counts");
    invalid = solution;
    invalid.cycles[0].segments[0].targets.nominal_positions[0].reset();
    require_throws<std::invalid_argument>([&] { (void)make_tripod_sequence_timeline(invalid); },
        "incomplete nominal servo positions");
    invalid = solution;
    for (auto& frame : invalid.cycles[0].foot_cycle.frames) {
        frame.sample.time_s *= 1e200;
    }
    require_throws<std::range_error>([&] { (void)make_tripod_sequence_timeline(invalid); },
        "global offset collapses distinguishable local times");
    std::cout << "SEQUENCE TIMELINE: segments=" << timeline.size()
        << ", end_time_s=" << timeline.back().end_time_s << '\n';
}

void test_variable_schedule(const robot_model& model) {
    const auto request = example_schedule(model);
    const tripod_sequence_planner planner(model);
    const body_ik_solver solver(model);
    const auto result = planner.plan(request);
    require(result.status == tripod_sequence_status::checks_passed && result.solution
        && !result.failed_cycle_index && !result.cycle_failure, "variable schedule succeeds with a complete solution");
    const auto& cycles = result.solution->cycles;
    require(cycles.size() == 4, "zero-speed final cycle remains part of the requested sequence");
    const std::array<std::size_t, 4> segment_counts{40, 12, 16, 32};
    const std::array<double, 4> end_x{0.027, 0.030, 0.03125, 0.03125};
    const std::array<double, 4> end_y{-0.010, -0.0094, -0.0094, -0.0094};

    for (std::size_t index = 0; index < cycles.size(); ++index) {
        const auto& cycle = cycles[index];
        const auto& parameters = request.cycles[index];
        require(cycle.segments.size() == segment_counts[index]
            && cycle.foot_cycle.frames.size() == cycle.segments.size() + 1,
            "each cycle uses its own stance ratio and frame count");
        const auto& first = cycle.foot_cycle.frames.front();
        const auto& last = cycle.foot_cycle.frames.back();
        const auto& start_pose = index == 0 ? request.initial_state.body_pose : cycles[index - 1].foot_cycle.final_pose;
        const auto& start_feet = index == 0 ? request.initial_state.feet : cycles[index - 1].foot_cycle.final_feet;
        require(first.sample.time_s == 0.0 && near(last.sample.time_s, parameters.timing.period_s),
            "each cycle uses its own local duration");
        require(last.sample.boundary && last.sample.boundary->cycle_complete,
            "each cycle reaches its complete boundary");
        require(same_pose(first.body_pose, start_pose)
            && same_pose(cycle.foot_cycle.final_pose, {{end_x[index], end_y[index], 0.0}, 0.0}),
            "body starts at the previous terminal pose and integrates this cycle's velocity");
        for (const auto leg : leg_order) {
            const auto leg_offset = leg_index(leg);
            require(same_point(first.targets.points_in_planning_frame[leg_offset], start_feet.stance_points()[leg_offset]),
                "cycle begins on the continuous planned anchors");
            require(!cycle.foot_cycle.final_feet.swings()[leg_offset]
                && same_point(last.targets.points_in_planning_frame[leg_offset],
                    cycle.foot_cycle.final_feet.stance_points()[leg_offset]), "cycle ends with all feet planted");
            double peak_z = request.ground_height_m;
            for (const auto& frame : cycle.foot_cycle.frames) {
                peak_z = std::max(peak_z, frame.targets.points_in_planning_frame[leg_offset].z);
            }
            require(near(peak_z, request.ground_height_m + parameters.lift_height_m),
                "swing peak follows this cycle's lift height");
        }
        const auto& next_frame = cycle.foot_cycle.frames[1];
        const auto& reference = index == 0 ? request.initial_state.reference_angles : cycles[index - 1].segments.back().nominal_angles;
        const auto& branches = index == 0 ? request.initial_state.remembered_branches : cycles[index - 1].segments.back().branches;
        const auto direct = solver.solve({next_frame.targets.points_in_body, reference, branches, request.selection_options,
            body_connection_request{next_frame.sample.time_s, request.max_joint_speed_rad_s}});
        require(direct.status == body_ik_status::success && direct.solution, "first segment reachable from the handed-off state");
        require_same_joint_solution(cycle.segments.front(), *direct.solution);

        std::cout << "SEQUENCE CYCLE: index=" << index << ", duration_s=" << last.sample.time_s
            << ", segments=" << cycle.segments.size() << ", end_x_m=" << end_x[index]
            << ", end_y_m=" << end_y[index] << ", lift_m=" << parameters.lift_height_m << '\n';
    }
    test_sequence_timeline(*result.solution);
}

const tripod_cycle_result& require_failed_cycle(const tripod_sequence_result& result, std::size_t index) {
    require(result.status == tripod_sequence_status::cycle_failed && result.failed_cycle_index == index
        && result.cycle_failure && !result.solution, "sequence failure identifies the cycle and releases no partial sequence");
    require(!result.cycle_failure->solution, "failed cycle has no partial motion solution");
    return *result.cycle_failure;
}

void test_first_and_continuation_tolerance(const robot_model& model) {
    const tripod_sequence_planner planner(model);
    auto request = example_schedule(model);
    request.initial_state.reference_angles[0].yaw_rad += 0.002;
    const auto mismatched_start = planner.plan(request);
    const auto& initial = require_failed_cycle(mismatched_start, 0);
    require(initial.status == tripod_cycle_status::initial_reference_mismatch
        && initial.initial_failed_leg == leg_id::right_front && initial.initial_foot_error_m
        && *initial.initial_foot_error_m > request.initial_foot_tolerance_m
        && *initial.initial_foot_error_m < request.continuation_foot_tolerance_m
        && !initial.failed_frame_index && !initial.segment_failure,
        "first cycle uses the tighter initial tolerance and retains its diagnostic");

    request = example_schedule(model);
    request.continuation_foot_tolerance_m = 1e-10;
    const auto mismatched_handoff = planner.plan(request);
    const auto& handoff = require_failed_cycle(mismatched_handoff, 1);
    require(handoff.status == tripod_cycle_status::initial_reference_mismatch
        && handoff.initial_failed_leg && handoff.initial_foot_error_m
        && *handoff.initial_foot_error_m > request.continuation_foot_tolerance_m
        && !handoff.failed_frame_index && !handoff.segment_failure,
        "second cycle fails independently under an insufficient continuation tolerance");
}

void test_later_segment_failure(const robot_model& model) {
    const tripod_sequence_planner planner(model);
    const tripod_cycle_planner single_planner(model);
    auto request = example_schedule(model);
    const auto feasible_second = request.cycles[1];
    request.cycles[1].lift_height_m = 0.6;
    const auto first = single_planner.plan(single_cycle_request(request, 0));
    require(first.status == tripod_cycle_status::checks_passed && first.solution, "single first cycle is feasible");
    auto second_request = single_cycle_request(request, 1);
    second_request.foot_request.initial_pose = first.solution->foot_cycle.final_pose;
    second_request.foot_request.initial_feet = first.solution->foot_cycle.final_feet;
    second_request.reference_angles = first.solution->segments.back().nominal_angles;
    second_request.remembered_branches = first.solution->segments.back().branches;
    second_request.initial_foot_tolerance_m = request.continuation_foot_tolerance_m;
    const auto direct = single_planner.plan(second_request);
    require(direct.status == tripod_cycle_status::segment_failed && direct.failed_frame_index
        && direct.segment_failure && direct.segment_failure->failed_leg, "infeasible lift produces a single-cycle segment diagnostic");
    const auto result = planner.plan(request);
    const auto& failure = require_failed_cycle(result, 1);
    require(failure.status == direct.status && failure.failed_frame_index == direct.failed_frame_index
        && failure.segment_failure && failure.segment_failure->status == direct.segment_failure->status
        && failure.segment_failure->failed_leg == direct.segment_failure->failed_leg
        && failure.segment_failure->selection_failure == direct.segment_failure->selection_failure
        && !failure.segment_failure->solution, "later failure preserves the frame, leg, and original reason");
    const auto& expected_path = direct.segment_failure->path_failure;
    const auto& path = failure.segment_failure->path_failure;
    require(path.has_value() == expected_path.has_value(), "path diagnostic availability is preserved");
    if (path) {
        require(path->status == expected_path->status && path->failed_sample_index == expected_path->failed_sample_index
            && path->failed_clearance_m == expected_path->failed_clearance_m, "path failure preserves the sample and clearance");
    }

    request.cycles[1] = feasible_second;
    const auto retry = planner.plan(request);
    require(retry.status == tripod_sequence_status::checks_passed && retry.solution
        && same_pose(retry.solution->cycles.front().foot_cycle.frames.front().body_pose, request.initial_state.body_pose),
        "retry after a rejected prefix starts from the caller's initial state");
}

void test_repeat_entry(const robot_model& model) {
    const tripod_sequence_planner planner(model);
    auto request = example_schedule(model);
    const auto first = single_cycle_request(request, 0);
    const auto first_parameters = request.cycles.front();
    request.cycles.assign(2, first_parameters);
    const auto scheduled = planner.plan(request);
    const auto repeated = planner.plan({first, 2, request.continuation_foot_tolerance_m});
    require(scheduled.status == tripod_sequence_status::checks_passed && scheduled.solution
        && repeated.status == tripod_sequence_status::checks_passed && repeated.solution
        && scheduled.solution->cycles.size() == 2 && repeated.solution->cycles.size() == 2,
        "uniform convenience entry preserves the established two-cycle result");
    for (std::size_t index = 0; index < 2; ++index) {
        const auto& expected = scheduled.solution->cycles[index];
        const auto& actual = repeated.solution->cycles[index];
        require(same_pose(actual.foot_cycle.final_pose, expected.foot_cycle.final_pose)
            && actual.segments.size() == expected.segments.size(), "repeat matches schedule geometry and segment count");
        for (std::size_t segment = 0; segment < expected.segments.size(); ++segment) {
            require_same_joint_solution(actual.segments[segment], expected.segments[segment]);
        }
    }
}

void test_invalid_inputs(const robot_model& model) {
    const tripod_sequence_planner planner(model);
    const auto original = example_schedule(model);
    const auto first = single_cycle_request(original, 0);
    auto request = original;
    request.cycles.clear();
    require_throws<std::invalid_argument>([&] { (void)planner.plan(request); }, "empty schedule");
    require_throws<std::invalid_argument>([&] {
        (void)planner.plan(tripod_sequence_request{first, 0, 0.001});
    }, "zero repetitions");
    require_throws<std::length_error>([&] {
        (void)planner.plan(tripod_sequence_request{first, std::numeric_limits<std::size_t>::max(), 0.001});
    }, "unrepresentable repetition count");
    for (const double bad : {-0.01, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        request = original;
        request.continuation_foot_tolerance_m = bad;
        require_throws<std::invalid_argument>([&] { (void)planner.plan(request); }, "invalid continuation tolerance");
    }
    request = original;
    request.initial_foot_tolerance_m = -0.001;
    require_throws<std::invalid_argument>([&] { (void)planner.plan(request); }, "invalid initial tolerance");
    request = original;
    request.cycles.back().timing.period_s = 0.0;
    require_throws<std::invalid_argument>([&] { (void)planner.plan(request); }, "invalid period in a later cycle");
}

}  // namespace

int main(int argc, char** argv) {
    try {
        require(argc == 2, "usage: tripod_sequence_planner_tests robot_hardware.json");
        const auto model = robot_model::from_file(argv[1]);
        test_variable_schedule(model);
        test_first_and_continuation_tolerance(model);
        test_later_segment_failure(model);
        test_repeat_entry(model);
        test_invalid_inputs(model);
        std::cout << "PASS: variable cycles, state handoff, whole-sequence failure, repeat compatibility and validation.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
