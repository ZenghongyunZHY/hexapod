#include "hexapod_core/tripod_cycle_planner.hpp"
#include "hexapod_core/joint_mapper.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using namespace hexapod_core;

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

template <typename exception_type, typename function_type>
void require_throws(function_type function, const std::string& message) {
    try { function(); } catch (const exception_type&) { return; }
    throw std::runtime_error("missing expected exception: " + message);
}

tripod_cycle_request example_request(const robot_model& model) {
    const joint_mapper mapper(model);
    const leg_kinematics kinematics(model);
    std::array<leg_joint_angles, leg_count> references{};
    std::array<leg_branch_state, leg_count> branches{};
    std::array<vector3, leg_count> anchors{};
    std::array<stance_midpoint_reference, leg_count> midpoint_references{};
    std::array<std::array<double, joints_per_leg>, leg_count> speeds{};
    const std::array<double, 3> candidate{0.0, degrees_to_radians(12.0), degrees_to_radians(-72.0)};
    for (const auto leg : leg_order) {
        const auto index = leg_index(leg);
        std::array<double, 3> angles{};
        for (std::size_t axis = 0; axis < joints_per_leg; ++axis) {
            const auto id = model.joint(leg, joint_order[axis]).binding.id;
            angles[axis] = mapper.servo_position_to_joint_angle_rad(id,
                mapper.joint_angle_rad_to_servo_position(id, candidate[axis]));
        }
        references[index] = {angles[0], angles[1], angles[2]};
        anchors[index] = kinematics.forward_in_body_frame(leg, references[index]);
        midpoint_references[index] = {anchors[index].x, anchors[index].y};
        speeds[index].fill(1.0);
    }
    const tripod_foot_cycle_request feet{{4.0, 0.625}, 10, {{0.0, 0.0, 0.0}, 0.0},
        {0.0, 0.0, 0.0}, gait_foot_state(anchors), midpoint_references, anchors[0].z, 0.0};
    return {feet, references, branches,
        {knee_singularity_policy::reject, branch_continuity_policy::keep_reference_branch}, speeds, 4, 1e-10};
}

void require_failure_has_no_solution(const tripod_cycle_result& result) {
    require(!result.solution, "failed cycle must not expose a partial solution");
}

void test_success(const robot_model& model) {
    const tripod_cycle_planner planner(model);
    const joint_mapper mapper(model);
    for (const double duty : {0.5, 0.625}) {
        auto request = example_request(model);
        request.foot_request.timing.stance_ratio = duty;
        const auto initial_references = request.reference_angles;
        const auto result = planner.plan(request);
        require(result.status == tripod_cycle_status::checks_passed && result.solution,
            "stationary zero-lift cycle passes exact ground checks");
        require(!result.initial_failed_leg && !result.initial_foot_error_m
            && !result.failed_frame_index && !result.segment_failure, "success has no stale diagnostics");
        const auto& solution = *result.solution;
        require(solution.segments.size() + 1 == solution.foot_cycle.frames.size(), "joint/frame indexing contract");
        require(solution.segments.size() == (duty == 0.5 ? 20U : 40U), "full cycle segment count");
        for (const auto& segment : solution.segments) {
            for (const auto leg : leg_order) {
                const auto index = leg_index(leg);
                const auto& angles = request.reference_angles[index];
                const std::array<double, 3> expected_angles{angles.yaw_rad, angles.upper_leg_rad, angles.lower_leg_rad};
                for (std::size_t axis = 0; axis < joints_per_leg; ++axis) {
                    const auto id = model.joint(leg, joint_order[axis]).binding.id;
                    require(segment.targets.nominal_positions[id.index()]
                        && segment.targets.nominal_positions[id.index()]->value()
                            == mapper.joint_angle_rad_to_servo_position(id, expected_angles[axis]).value()
                        && !segment.targets.commanded_positions[id.index()], "all nominal positions remain fixed and commands empty");
                }
                require(!solution.foot_cycle.final_feet.swings()[index], "final feet are in planned support");
                require(!request.foot_request.initial_feet.swings()[index], "input foot state unchanged");
            }
        }
        for (std::size_t leg = 0; leg < leg_count; ++leg) {
            require(request.reference_angles[leg].yaw_rad == initial_references[leg].yaw_rad
                && request.reference_angles[leg].upper_leg_rad == initial_references[leg].upper_leg_rad
                && request.reference_angles[leg].lower_leg_rad == initial_references[leg].lower_leg_rad,
                "input joint reference unchanged");
        }
    }
}

void test_initial_reference(const robot_model& model) {
    const tripod_cycle_planner planner(model);
    for (const auto leg : leg_order) {
        auto request = example_request(model);
        request.reference_angles[leg_index(leg)].yaw_rad += 0.01;
        const auto result = planner.plan(request);
        require(result.status == tripod_cycle_status::initial_reference_mismatch
            && result.initial_failed_leg == leg && result.initial_foot_error_m
            && *result.initial_foot_error_m > request.initial_foot_tolerance_m,
            "each leg reports initial mismatch");
        require(!result.failed_frame_index && !result.segment_failure, "initial mismatch has no segment diagnostics");
        require_failure_has_no_solution(result);
    }
    auto request = example_request(model);
    auto anchors = request.foot_request.initial_feet.stance_points();
    anchors[0].x += 0.001;
    request.foot_request.initial_feet = gait_foot_state(anchors);
    request.initial_foot_tolerance_m = 0.0;
    const auto mismatch = planner.plan(request);
    require(mismatch.initial_foot_error_m.has_value(), "known displaced initial point produces an error");
    request.initial_foot_tolerance_m = *mismatch.initial_foot_error_m;
    require(planner.plan(request).status != tripod_cycle_status::initial_reference_mismatch,
        "initial tolerance equality is accepted");
    request.initial_foot_tolerance_m = std::nextafter(*mismatch.initial_foot_error_m, 0.0);
    require(planner.plan(request).status == tripod_cycle_status::initial_reference_mismatch,
        "initial tolerance immediately below error is rejected");
}

void test_segment_failures(const robot_model& model) {
    const tripod_cycle_planner planner(model);
    auto unreachable = example_request(model);
    unreachable.foot_request.intervals_per_stage = 1;
    unreachable.foot_request.midpoint_references_in_body[leg_index(leg_id::left_rear)].x_m += 10.0;
    const auto selection = planner.plan(unreachable);
    require(selection.status == tripod_cycle_status::segment_failed && selection.failed_frame_index == 2
        && selection.segment_failure
        && selection.segment_failure->status == body_segment_status::target_selection_failed
        && selection.segment_failure->failed_leg == leg_id::left_rear
        && selection.segment_failure->selection_failure == leg_selection_status::geometric_unreachable,
        "later selection failure preserves exact frame, leg and reason");
    require_failure_has_no_solution(selection);

    auto too_fast = example_request(model);
    too_fast.foot_request.midpoint_references_in_body[5].x_m += 0.01;
    for (auto& limits : too_fast.max_joint_speed_rad_s) limits.fill(1e-12);
    const auto speed = planner.plan(too_fast);
    require(speed.status == tripod_cycle_status::segment_failed && speed.failed_frame_index
        && *speed.failed_frame_index > 1 && speed.segment_failure
        && speed.segment_failure->selection_failure == leg_selection_status::no_candidate_within_speed_limits,
        "later speed failure rejects entire cycle");
    require_failure_has_no_solution(speed);

    auto below_ground = example_request(model);
    below_ground.foot_request.ground_height_m += 0.001;
    below_ground.initial_foot_tolerance_m = 1.0;
    const auto path = planner.plan(below_ground);
    require(path.status == tripod_cycle_status::segment_failed && path.failed_frame_index == 1
        && path.segment_failure && path.segment_failure->status == body_segment_status::path_check_failed
        && path.segment_failure->path_failure
        && path.segment_failure->path_failure->status == leg_path_check_status::foot_clearance_below_minimum,
        "initial tolerance never relaxes actual ground constraint");
    require_failure_has_no_solution(path);
    require(below_ground.foot_request.initial_feet.stance_points()[0].z
        < below_ground.foot_request.ground_height_m, "failed request retains original anchors");
}

void test_invalid_inputs(const robot_model& model) {
    const tripod_cycle_planner planner(model);
    const auto valid = example_request(model);
    for (const double bad : {-0.01, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        auto request = valid;
        request.initial_foot_tolerance_m = bad;
        require_throws<std::invalid_argument>([&] { (void)planner.plan(request); }, "invalid initial tolerance");
    }
    for (const std::size_t bad : {std::size_t{0}, std::numeric_limits<std::size_t>::max()}) {
        auto request = valid;
        request.path_interval_count = bad;
        if (bad == 0) require_throws<std::invalid_argument>([&] { (void)planner.plan(request); }, "zero path intervals");
        else require_throws<std::length_error>([&] { (void)planner.plan(request); }, "path capacity rejection");
    }
    for (std::size_t leg = 0; leg < leg_count; ++leg) {
        for (std::size_t axis = 0; axis < joints_per_leg; ++axis) {
            for (const double bad : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
                auto request = valid;
                request.reference_angles[0].yaw_rad += 0.01;
                request.max_joint_speed_rad_s[leg][axis] = bad;
                require_throws<std::invalid_argument>([&] { (void)planner.plan(request); }, "all speed entries checked before mismatch return");
            }
        }
    }
    for (const auto component : {&leg_joint_angles::yaw_rad, &leg_joint_angles::upper_leg_rad, &leg_joint_angles::lower_leg_rad}) {
        auto request = valid;
        request.reference_angles[5].*component = std::numeric_limits<double>::quiet_NaN();
        require_throws<std::invalid_argument>([&] { (void)planner.plan(request); }, "nonfinite reference checked");
    }
    auto request = valid;
    request.selection_options.knee_singularity = static_cast<knee_singularity_policy>(99);
    require_throws<std::invalid_argument>([&] { (void)planner.plan(request); }, "invalid singularity policy");
    request = valid;
    request.selection_options.branch_continuity = static_cast<branch_continuity_policy>(99);
    require_throws<std::invalid_argument>([&] { (void)planner.plan(request); }, "invalid branch policy");
}

void report_moving_candidate(const robot_model& model) {
    auto request = example_request(model);
    request.foot_request.velocity.vx_m_s = 0.0005;
    request.foot_request.lift_height_m = 0.003;
    const auto result = tripod_cycle_planner(model).plan(request);
    require(result.status != tripod_cycle_status::initial_reference_mismatch, "moving probe starts from matched reference");
    std::cout << "OFFLINE MOVING CANDIDATE: status=" << static_cast<int>(result.status);
    if (result.failed_frame_index) std::cout << ", failed_frame=" << *result.failed_frame_index;
    if (result.segment_failure) {
        require_failure_has_no_solution(result);
        const auto& failure = *result.segment_failure;
        if (failure.failed_leg) std::cout << ", failed_leg=" << leg_index(*failure.failed_leg);
        if (failure.path_failure && failure.path_failure->failed_clearance_m) {
            require(*failure.path_failure->failed_clearance_m < 0.0, "reported strict clearance violation is negative");
            std::cout << ", clearance_mm=" << *failure.path_failure->failed_clearance_m * 1000.0;
            if (failure.path_failure->failed_sample) {
                std::cout << ", path_ratio=" << failure.path_failure->failed_sample->ratio;
            }
            if (result.failed_frame_index && failure.failed_leg) {
                const auto geometry = plan_tripod_foot_cycle(request.foot_request);
                const auto& target = geometry.frames[*result.failed_frame_index].targets.points_in_body[leg_index(*failure.failed_leg)];
                const double ground_offset = request.foot_request.initial_pose.body_origin_in_planning_frame.z
                    - request.foot_request.ground_height_m;
                std::cout << ", ideal_clearance_mm=" << (target.z + ground_offset) * 1000.0;
                auto references = request.reference_angles;
                auto branches = request.remembered_branches;
                const body_ik_solver solver(model);
                const leg_kinematics kinematics(model);
                for (std::size_t frame = 1; frame <= *result.failed_frame_index; ++frame) {
                    const double duration = geometry.frames[frame].sample.time_s - geometry.frames[frame - 1].sample.time_s;
                    const auto selected = solver.solve({geometry.frames[frame].targets.points_in_body,
                        references, branches, request.selection_options,
                        body_connection_request{duration, request.max_joint_speed_rad_s}});
                    require(selected.status == body_ik_status::success && selected.solution,
                        "moving probe can independently recover selected nominal endpoint");
                    if (frame == *result.failed_frame_index) {
                        const auto leg = *failure.failed_leg;
                        const auto start = kinematics.forward_in_body_frame(leg, references[leg_index(leg)]);
                        const auto end = kinematics.forward_in_body_frame(leg, selected.solution->nominal_angles[leg_index(leg)]);
                        std::cout << ", start_clearance_mm=" << (start.z + ground_offset) * 1000.0
                            << ", end_clearance_mm=" << (end.z + ground_offset) * 1000.0;
                    }
                    references = selected.solution->nominal_angles;
                    branches = selected.solution->branches;
                }
            }
        }
    } else {
        require(result.solution.has_value(), "successful moving probe has full solution");
    }
    std::cout << '\n';
}

tripod_cycle_request moving_request(const robot_model& model, double duty) {
    auto request = example_request(model);
    request.foot_request.timing.stance_ratio = duty;
    request.foot_request.velocity.vx_m_s = 0.0005;
    request.foot_request.lift_height_m = 0.003;
    // 离线回归预算，单位 m。
    request.clearance_policy.emplace(0.0007, 0.0007, 0.0007);
    return request;
}

void require_clearance_failure(const tripod_cycle_result& result, leg_path_check_status reason) {
    require(result.status == tripod_cycle_status::segment_failed && result.failed_frame_index
        && result.segment_failure && result.segment_failure->failed_leg
        && result.segment_failure->status == body_segment_status::path_check_failed
        && result.segment_failure->path_failure, "clearance rejection carries nested diagnostic");
    const auto& path = *result.segment_failure->path_failure;
    require(path.status == reason && path.failed_sample_index && path.failed_sample
        && path.failed_clearance_m, "clearance rejection identifies bound and sample");
    require_failure_has_no_solution(result);
    require(!result.segment_failure->solution, "rejected segment cannot expose joint targets");
}

void test_cycle_clearance_policy(const robot_model& model) {
    const tripod_cycle_planner planner(model);
    for (const double duty : {0.5, 0.625}) {
        const auto request = moving_request(model, duty);
        const auto accepted = planner.plan(request);
        require(accepted.status == tripod_cycle_status::checks_passed && accepted.solution
            && !accepted.failed_frame_index && !accepted.segment_failure,
            "moving cycle passes with explicit sampled clearance policy");
        const auto& solution = *accepted.solution;
        require(solution.segments.size() == (duty == 0.5 ? 20U : 40U)
            && solution.segments.size() + 1 == solution.foot_cycle.frames.size(),
            "two tripod swings yield a complete joint cycle");
        require(std::abs(solution.foot_cycle.final_pose.body_origin_in_planning_frame.x - 0.002) < 1e-12,
            "planned pose advances through the full 4 s cycle");
        for (const auto leg : leg_order) {
            require(!solution.foot_cycle.final_feet.swings()[leg_index(leg)], "cycle ends in planned support");
        }
        for (const auto& segment : solution.segments) {
            for (std::size_t joint = 0; joint < joint_count; ++joint) {
                require(segment.targets.nominal_positions[joint] && !segment.targets.commanded_positions[joint],
                    "complete nominal targets without commanded positions");
            }
        }
        std::cout << "SAMPLED MOVING CYCLE: duty=" << duty
            << ", segments=" << solution.segments.size() << ", x_m="
            << solution.foot_cycle.final_pose.body_origin_in_planning_frame.x << '\n';
    }

    auto request = moving_request(model, 0.625);
    request.clearance_policy.reset();
    const auto strict = planner.plan(request);
    require_clearance_failure(strict, leg_path_check_status::foot_clearance_below_minimum);
    require(strict.failed_frame_index == 8 && strict.segment_failure->failed_leg == leg_id::right_front
        && strict.segment_failure->path_failure->failed_sample_index == 1,
        "legacy strict stance preserves the failure location");

    request.clearance_policy.emplace(0.0, 0.0007, 0.0007);
    const auto tight_below = planner.plan(request);
    require_clearance_failure(tight_below, leg_path_check_status::foot_clearance_below_minimum);
    require(tight_below.failed_frame_index == strict.failed_frame_index
        && tight_below.segment_failure->failed_leg == strict.segment_failure->failed_leg
        && tight_below.segment_failure->path_failure->failed_clearance_m
            == strict.segment_failure->path_failure->failed_clearance_m,
        "tight contact lower bound recovers the strict stance rejection");

    request.clearance_policy.emplace(0.0007, 0.0, 0.0007);
    const auto tight_above = planner.plan(request);
    require_clearance_failure(tight_above, leg_path_check_status::foot_clearance_above_maximum);
    require(*tight_above.segment_failure->path_failure->failed_clearance_m > 0.0,
        "zero contact upper bound identifies a foot above the plane");

    request.clearance_policy.emplace(0.0007, 0.0007, 0.0);
    const auto tight_swing = planner.plan(request);
    require_clearance_failure(tight_swing, leg_path_check_status::foot_clearance_below_minimum);
    const auto geometry = plan_tripod_foot_cycle(request.foot_request);
    const auto& start = geometry.frames[*tight_swing.failed_frame_index - 1];
    const auto& end = geometry.frames[*tight_swing.failed_frame_index];
    const auto leg = leg_index(*tight_swing.segment_failure->failed_leg);
    require(start.outgoing_swings[leg].has_value(), "tight swing rejection occurs on a swing segment");
    const auto& path = *tight_swing.segment_failure->path_failure;
    const double r = path.failed_sample->ratio;
    const double s = (1.0 - r) * start.phases[leg].phase_ratio + r * end.phases[leg].phase_ratio;
    require(s > 0.0 && s < 1.0, "swing budget applies strictly inside swing");
    const double ideal_height = 0.003 * 64.0 * std::pow(s * (1.0 - s), 3);
    require(*path.failed_clearance_m < ideal_height - 1e-12,
        "nominal height misses the independently computed swing requirement");
    std::cout << "TIGHT CLEARANCE: lower_frame=" << *tight_below.failed_frame_index
        << ", upper_frame=" << *tight_above.failed_frame_index
        << ", swing_frame=" << *tight_swing.failed_frame_index
        << ", swing_ratio=" << s << ", nominal_mm=" << *path.failed_clearance_m * 1000.0
        << ", ideal_mm=" << ideal_height * 1000.0 << '\n';
}

void test_policy_validation_and_initial_reference(const robot_model& model) {
    const tripod_cycle_planner planner(model);
    auto request = moving_request(model, 0.625);
    request.reference_angles[0].yaw_rad += 0.01;
    const auto mismatch = planner.plan(request);
    require(mismatch.status == tripod_cycle_status::initial_reference_mismatch
        && mismatch.initial_failed_leg == leg_id::right_front && mismatch.initial_foot_error_m
        && *mismatch.initial_foot_error_m > request.initial_foot_tolerance_m,
        "clearance policy does not relax the initial 3D reference check");
    require_failure_has_no_solution(mismatch);
    for (const double bad : {-0.01, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        for (std::size_t field = 0; field < 3; ++field) {
            std::array<double, 3> values{0.0007, 0.0007, 0.0007};
            values[field] = bad;
            request.clearance_policy.emplace(values[0], values[1], values[2]);
            require_throws<std::invalid_argument>([&] { (void)planner.plan(request); },
                "invalid supplied clearance budget detected before initial mismatch");
        }
    }
}

void test_cycle_handoff(const robot_model& model) {
    const tripod_cycle_planner planner(model);
    const leg_kinematics kinematics(model);
    for (const double duty : {0.5, 0.625}) {
        auto request = moving_request(model, duty);
        const auto first = planner.plan(request);
        require(first.status == tripod_cycle_status::checks_passed && first.solution,
            "first cycle provides candidate state for the handoff");
        const auto& first_solution = *first.solution;
        const auto& terminal = first_solution.segments.back();
        request.foot_request.initial_pose = first_solution.foot_cycle.final_pose;
        request.foot_request.initial_feet = first_solution.foot_cycle.final_feet;
        request.reference_angles = terminal.nominal_angles;
        request.remembered_branches = terminal.branches;

        const auto strict_start = planner.plan(request);
        require(strict_start.status == tripod_cycle_status::initial_reference_mismatch
            && strict_start.initial_failed_leg && strict_start.initial_foot_error_m,
            "exact first-cycle tolerance detects quantization at the seam");
        require_failure_has_no_solution(strict_start);

        // Explicit seam budget for this offline trajectory.
        request.initial_foot_tolerance_m = 0.001;
        const auto second = planner.plan(request);
        require(second.status == tripod_cycle_status::checks_passed && second.solution,
            "second cycle accepts the planned anchors and nominal joint handoff");
        const auto& before = first_solution.foot_cycle.frames.back();
        const auto& after = second.solution->foot_cycle.frames.front();
        double max_error = 0.0;
        for (const auto leg : leg_order) {
            const auto index = leg_index(leg);
            const auto& end_anchor = before.targets.points_in_planning_frame[index];
            const auto& next_anchor = after.targets.points_in_planning_frame[index];
            require(end_anchor.x == next_anchor.x && end_anchor.y == next_anchor.y && end_anchor.z == next_anchor.z,
                "handoff preserves the planned anchor in the shared frame");
            const auto& target = after.targets.points_in_body[index];
            const auto nominal = kinematics.forward_in_body_frame(leg, terminal.nominal_angles[index]);
            const double error = std::hypot(nominal.x - target.x, nominal.y - target.y, nominal.z - target.z);
            if (error > max_error) max_error = error;
        }
        require(max_error > 1e-10 && max_error < request.initial_foot_tolerance_m,
            "three-dimensional seam error fits the explicit continuation tolerance");
        require(std::abs(second.solution->foot_cycle.final_pose.body_origin_in_planning_frame.x - 0.004) < 1e-12,
            "two accepted cycles advance the body by 4 mm");
        std::cout << "CYCLE HANDOFF: duty=" << duty << ", max_initial_foot_error_mm="
            << max_error * 1000.0 << ", x_m="
            << second.solution->foot_cycle.final_pose.body_origin_in_planning_frame.x << '\n';
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        require(argc == 2, "usage: tripod_cycle_planner_tests robot_hardware.json");
        const auto model = robot_model::from_file(argv[1]);
        test_success(model);
        test_initial_reference(model);
        test_segment_failures(model);
        test_invalid_inputs(model);
        report_moving_candidate(model);
        test_cycle_clearance_policy(model);
        test_policy_validation_and_initial_reference(model);
        test_cycle_handoff(model);
        std::cout << "PASS: joint cycle success, initial-reference boundaries, atomic rejection and sampled clearance.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
