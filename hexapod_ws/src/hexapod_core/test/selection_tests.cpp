#include "hexapod_core/leg_candidate_selector.hpp"

#include <algorithm>
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

void require_near(double actual, double expected, const std::string& message) {
    require(std::isfinite(actual) && std::isfinite(expected)
        && std::abs(actual - expected) <= 1e-10, message);
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

leg_joint_angles from_degrees(double yaw, double upper, double lower) {
    return {degrees_to_radians(yaw), degrees_to_radians(upper),
        degrees_to_radians(lower)};
}

void require_positions(
    const std::array<servo_position, joints_per_leg>& actual,
    const std::array<int, joints_per_leg>& expected,
    const std::string& message) {
    for (std::size_t index = 0; index < joints_per_leg; ++index) {
        require(actual[index].value() == expected[index],
            message + " joint " + std::to_string(index));
    }
}

leg_mapped_candidate require_selected(
    const leg_selection_result& result, const std::string& message) {
    require(result.status == leg_selection_status::success, message + " status");
    require(result.selected.has_value() && result.cost.has_value(),
        message + " candidate and cost");
    require(std::isfinite(*result.cost) && *result.cost >= 0.0,
        message + " finite nonnegative cost");
    return *result.selected;
}

void require_failure(const leg_selection_result& result,
    leg_selection_status expected, const std::string& message) {
    require(result.status == expected, message + " status");
    require(!result.selected && !result.cost, message + " no candidate or cost");
}

const leg_selection_options allow_nearest{
    knee_singularity_policy::allow, branch_continuity_policy::nearest};
const leg_selection_options allow_keep{
    knee_singularity_policy::allow,
    branch_continuity_policy::keep_reference_branch};
const leg_selection_options reject_keep{
    knee_singularity_policy::reject,
    branch_continuity_policy::keep_reference_branch};

// 独立预期检查六腿绑定和左右符号；不读取反馈或驱动硬件。
void test_mapping(const leg_candidate_selector& selector) {
    const leg_ik_candidate candidate{from_degrees(0.0, 12.1, -72.0), false};
    for (const auto leg : leg_order) {
        const auto mapped = selector.map_candidate(leg, candidate);
        require(mapped.has_value(), "six-leg mapping succeeds");
        const bool right = leg == leg_id::right_front
            || leg == leg_id::right_middle || leg == leg_id::right_rear;
        const std::array<int, joints_per_leg> expected = right
            ? std::array<int, joints_per_leg>{500, 550, 200}
            : std::array<int, joints_per_leg>{500, 450, 800};
        require_positions(mapped->positions, expected, "six-leg integer mapping");
        require_near(mapped->ik_candidate.angles.upper_leg_rad,
            degrees_to_radians(12.1), "ideal angle preserved");
        require_near(mapped->nominal_angles.upper_leg_rad,
            degrees_to_radians(12.0), "quantized nominal angle");
        require_near(mapped->nominal_angles.lower_leg_rad,
            degrees_to_radians(-72.0), "mathematical knee sign");
    }
    require(!selector.map_candidate(leg_id::right_middle,
        {from_degrees(90.0, 0.0, 0.0), false}), "installed limit rejection");
}

void test_selection_policies(const leg_kinematics& kinematics,
    const leg_candidate_selector& selector) {
    const auto target = kinematics.forward_in_leg_frame(from_degrees(0, 12, -72));
    auto ik_result = kinematics.inverse_candidates_in_leg_frame(target);
    const auto reference = from_degrees(0, 10, 1);
    const auto nearest = selector.select_candidate(leg_id::right_middle,
        ik_result, reference, {}, allow_nearest);
    const auto kept = selector.select_candidate(leg_id::right_middle,
        ik_result, reference, {}, allow_keep);
    require_positions(require_selected(nearest, "nearest").positions,
        {500, 550, 200}, "nearest negative knee");
    require_positions(require_selected(kept, "keep branch").positions,
        {500, 179, 800}, "kept positive knee");
    require(nearest.branch.knee == ik_branch_sign::negative,
        "nearest updates branch");
    require(kept.branch.knee == ik_branch_sign::positive,
        "keep preserves branch");
    require(*nearest.cost < *kept.cost, "policy can override distance");
    const double expected_cost = degrees_to_radians(2.0) * degrees_to_radians(2.0)
        + degrees_to_radians(-73.0) * degrees_to_radians(-73.0);
    require_near(*nearest.cost, expected_cost, "ordinary unwrapped angle cost");
    std::reverse(ik_result.solution.begin(), ik_result.solution.end());
    const auto reversed = selector.select_candidate(leg_id::right_middle,
        ik_result, reference, {}, allow_nearest);
    require_positions(require_selected(reversed, "reversed order").positions,
        {500, 550, 200}, "unique best independent of candidate order");
}

void test_failure_statuses(const leg_kinematics& kinematics,
    const leg_candidate_selector& selector) {
    const auto reference = from_degrees(0, 12, -72);
    const auto unreachable = kinematics.inverse_candidates_in_leg_frame({0, -1, 0});
    require_failure(selector.select_candidate(leg_id::right_middle,
        unreachable, reference, {}, allow_keep),
        leg_selection_status::geometric_unreachable, "unreachable propagation");
    const auto free_yaw = kinematics.inverse_candidates_in_leg_frame({0, 0, 0});
    require_failure(selector.select_candidate(leg_id::right_middle,
        free_yaw, reference, {}, allow_keep),
        leg_selection_status::geometric_singular, "free angle propagation");
    // 正径向需要反向偏航；负径向超出几何范围，故全部超安装限位。
    const auto blocked = kinematics.inverse_candidates_in_leg_frame({0, 0.2, 0});
    require(blocked.status == leg_ik_status::success, "blocked point has geometry");
    require_failure(selector.select_candidate(leg_id::right_middle,
        blocked, reference, {}, allow_keep),
        leg_selection_status::no_candidate_within_limits, "all candidates blocked");
}

void test_straight_and_memory(const leg_kinematics& kinematics,
    const leg_candidate_selector& selector) {
    const auto straight = kinematics.inverse_candidates_in_leg_frame({0, -0.22873, 0});
    const auto bent_reference = from_degrees(0, 12, -72);
    const auto allowed = selector.select_candidate(leg_id::right_middle,
        straight, bent_reference, {}, allow_keep);
    const auto& candidate = require_selected(allowed, "straight allowed");
    require_positions(candidate.positions, {500, 500, 500}, "straight positions");
    require(candidate.ik_candidate.knee_near_singular, "straight flag preserved");
    require(allowed.branch.knee == ik_branch_sign::negative,
        "negative knee memory retained");
    require_failure(selector.select_candidate(leg_id::right_middle,
        straight, bent_reference, {}, reject_keep),
        leg_selection_status::no_candidate_under_policy, "straight policy rejection");
    const auto zero_reference = from_degrees(0, 0, 0);
    require_failure(selector.select_candidate(leg_id::right_middle,
        straight, zero_reference, {}, allow_keep),
        leg_selection_status::reference_branch_undetermined, "unknown initial branch");
    const leg_branch_state remembered{
        ik_branch_sign::positive, ik_branch_sign::negative};
    const auto restored = selector.select_candidate(leg_id::right_middle,
        straight, zero_reference, remembered, allow_keep);
    (void)require_selected(restored, "straight explicit memory");
    require(restored.branch.knee == ik_branch_sign::negative,
        "explicit memory preserved");
    const auto nearest = selector.select_candidate(leg_id::right_middle,
        straight, zero_reference, {}, allow_nearest);
    (void)require_selected(nearest, "nearest from straight");
    require(!nearest.branch.knee, "unknown branch not invented");
}

// 几何膝角不在奇异带内，整数命令却变成零；随后检查跨帧记忆。
void test_quantized_knee(const leg_kinematics& kinematics,
    const leg_candidate_selector& selector) {
    const auto target = kinematics.forward_in_leg_frame(from_degrees(10, 20, 0.1));
    const auto ik_result = kinematics.inverse_candidates_in_leg_frame(target);
    const auto reference = from_degrees(10, 20, 0.96);
    const auto allowed = selector.select_candidate(leg_id::right_middle,
        ik_result, reference, {}, allow_keep);
    const auto& candidate = require_selected(allowed, "quantized knee allowed");
    require(!candidate.ik_candidate.knee_near_singular,
        "geometric knee outside numerical singular band");
    require(candidate.ik_candidate.angles.lower_leg_rad > 0,
        "positive geometric knee retained");
    require_near(candidate.nominal_angles.lower_leg_rad, 0,
        "integer knee becomes zero");
    require(allowed.branch.knee == ik_branch_sign::positive,
        "geometric branch survives quantization");
    require_failure(selector.select_candidate(leg_id::right_middle,
        ik_result, reference, {}, reject_keep),
        leg_selection_status::no_candidate_under_policy,
        "nominal singularity included in rejection");
    const auto next_reference = candidate.nominal_angles;
    const auto next_ik = kinematics.inverse_candidates_in_leg_frame(
        kinematics.forward_in_leg_frame(next_reference));
    const auto continued = selector.select_candidate(leg_id::right_middle,
        next_ik, next_reference, allowed.branch, allow_keep);
    (void)require_selected(continued, "next frame with memory");
    require(continued.branch.knee == ik_branch_sign::positive,
        "memory survives next frame");
    require_failure(selector.select_candidate(leg_id::right_middle,
        next_ik, next_reference, {}, allow_keep),
        leg_selection_status::reference_branch_undetermined,
        "discarding memory loses knee branch");
}

void test_negative_radial(const leg_kinematics& kinematics,
    const leg_candidate_selector& selector) {
    const auto reference = from_degrees(0, 80, 80);
    const auto target = kinematics.forward_in_leg_frame(reference);
    require(target.y > 0, "negative radial fixture");
    const auto ik_result = kinematics.inverse_candidates_in_leg_frame(target);
    const auto result = selector.select_candidate(leg_id::right_middle,
        ik_result, reference, {}, allow_keep);
    require_positions(require_selected(result, "negative radial").positions,
        {500, 833, 833}, "negative radial integer target");
    require(result.branch.radial == ik_branch_sign::negative,
        "signed radial preserved");
    require(result.branch.knee == ik_branch_sign::positive,
        "positive knee preserved");
}

void test_invalid_input(const leg_kinematics& kinematics,
    const leg_candidate_selector& selector) {
    const auto reference = from_degrees(0, 12, -72);
    const auto ik_result = kinematics.inverse_candidates_in_leg_frame(
        kinematics.forward_in_leg_frame(reference));
    const std::array<double leg_joint_angles::*, joints_per_leg> members{
        &leg_joint_angles::yaw_rad, &leg_joint_angles::upper_leg_rad,
        &leg_joint_angles::lower_leg_rad};
    for (const auto member : members) {
        for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity()}) {
            auto bad_reference = reference;
            bad_reference.*member = invalid;
            require_throws<std::invalid_argument>([&] {
                (void)selector.select_candidate(leg_id::right_middle,
                    ik_result, bad_reference, {}, allow_keep);
            }, "nonfinite reference component");
            const leg_ik_candidate bad_candidate{bad_reference, false};
            require_throws<std::invalid_argument>([&] {
                (void)selector.map_candidate(leg_id::right_middle, bad_candidate);
            }, "nonfinite candidate component");
        }
    }
    require_throws<std::out_of_range>([&] {
        (void)selector.select_candidate(leg_id::right_middle, ik_result,
            from_degrees(0, 90, -72), {}, allow_keep);
    }, "reference outside limits");
    const leg_branch_state conflict{
        ik_branch_sign::positive, ik_branch_sign::positive};
    require_throws<std::invalid_argument>([&] {
        (void)selector.select_candidate(leg_id::right_middle, ik_result,
            reference, conflict, allow_keep);
    }, "memory conflicts with reference");
    const leg_branch_state bad_memory{
        ik_branch_sign::positive, static_cast<ik_branch_sign>(999)};
    require_throws<std::invalid_argument>([&] {
        (void)selector.select_candidate(leg_id::right_middle, ik_result,
            reference, bad_memory, allow_keep);
    }, "invalid memory enum");
    const leg_ik_result empty_success{leg_ik_status::success, {}};
    require_throws<std::invalid_argument>([&] {
        (void)selector.filter_candidates(leg_id::right_middle, empty_success);
    }, "success without candidates");
    for (const auto status : {leg_ik_status::unreachable, leg_ik_status::singular}) {
        auto inconsistent = ik_result;
        inconsistent.status = status;
        require_throws<std::invalid_argument>([&] {
            (void)selector.filter_candidates(leg_id::right_middle, inconsistent);
        }, "failed status with candidates");
    }
    require_throws<std::out_of_range>([&] {
        (void)selector.filter_candidates(static_cast<leg_id>(leg_count), ik_result);
    }, "invalid leg");
    auto bad_options = allow_keep;
    bad_options.branch_continuity = static_cast<branch_continuity_policy>(999);
    require_throws<std::invalid_argument>([&] {
        (void)selector.select_candidate(leg_id::right_middle, ik_result,
            reference, {}, bad_options);
    }, "invalid branch policy");
    bad_options = allow_keep;
    bad_options.knee_singularity = static_cast<knee_singularity_policy>(999);
    require_throws<std::invalid_argument>([&] {
        (void)selector.select_candidate(leg_id::right_middle, ik_result,
            reference, {}, bad_options);
    }, "invalid knee policy");
}

}  // namespace

// 仅执行离线断言，测试姿态不作为默认运行姿态或实物测量结果。
int main(int argc, char** argv) {
    try {
        require(argc == 2, "usage: selection_tests robot_hardware.json");
        const auto model = robot_model::from_file(argv[1]);
        const leg_kinematics kinematics(model);
        const leg_candidate_selector selector(model);
        test_mapping(selector);
        test_selection_policies(kinematics, selector);
        test_failure_statuses(kinematics, selector);
        test_straight_and_memory(kinematics, selector);
        test_quantized_knee(kinematics, selector);
        test_negative_radial(kinematics, selector);
        test_invalid_input(kinematics, selector);
        std::cout << "PASS: six-leg mapping, selection policies, failure states, "
            "straight memory, quantized knee, negative radial and invalid input.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
