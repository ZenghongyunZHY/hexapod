#include "hexapod_core/body_ik_solver.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using namespace hexapod_core;
using json = nlohmann::json;

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void near(double actual, double expected, const std::string& message) {
    require(std::isfinite(actual) && std::abs(actual - expected) < 1e-10, message);
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

leg_joint_angles degrees(double yaw, double upper, double lower) {
    return {degrees_to_radians(yaw), degrees_to_radians(upper),
        degrees_to_radians(lower)};
}

const leg_selection_options keep{
    knee_singularity_policy::reject,
    branch_continuity_policy::keep_reference_branch};

const std::array<std::array<double, 3>, 6> desired_degrees{{
    {0, 12.1, -72}, {2.4, 14.5, -67.2}, {-4.8, 9.7, -76.8},
    {7.2, 16.9, -62.4}, {-9.6, 19.3, -57.6}, {12, 21.7, -52.8}
}};

// 独立列出位置预期，覆盖每条腿不同目标、左右符号及整数取整。
const std::array<std::array<int, 3>, 6> expected_positions{{
    {500, 550, 200}, {510, 560, 220}, {480, 540, 180},
    {530, 430, 760}, {460, 420, 740}, {550, 410, 720}
}};

body_ik_request fixture(const leg_kinematics& kinematics) {
    std::array<vector3, leg_count> points{};
    std::array<leg_joint_angles, leg_count> references{};
    for (const auto leg : leg_order) {
        const auto index = leg_index(leg);
        const auto& angle = desired_degrees[index];
        references[index] = degrees(angle[0], angle[1], angle[2]);
        points[index] = kinematics.forward_in_body_frame(leg, references[index]);
    }
    return {points, references, {}, keep};
}

void require_failure(const body_ik_result& result, leg_id leg,
    leg_selection_status reason) {
    require(result.status == body_ik_status::leg_selection_failed,
        "whole-body failure status");
    require(result.failed_leg == leg && result.failure_reason == reason,
        "first failed leg and reason");
    require(!result.solution, "failure exposes no partial solution");
}

void test_body_wrapper(const leg_kinematics& kinematics) {
    // 由独立 CAD 根部坐标与零位外伸方向构成机身脚点。
    const double diagonal = 0.22873 / std::sqrt(2.0);
    const std::array<vector3, 6> zero_points{{
        {0.10217 + diagonal, -0.063 - diagonal, 0.021},
        {-0.00004, -0.30773, 0.021},
        {-0.10223 - diagonal, -0.06294 - diagonal, 0.021},
        {0.10223 + diagonal, 0.06294 + diagonal, 0.021},
        {0.00004, 0.30773, 0.021},
        {-0.10217 - diagonal, 0.063 + diagonal, 0.021}
    }};
    for (const auto leg : leg_order) {
        const auto result = kinematics.inverse_candidates_in_body_frame(
            leg, zero_points[leg_index(leg)]);
        require(result.status == leg_ik_status::success && result.solution.size() == 1,
            "independent body zero point returns one straight candidate");
        near(result.solution[0].angles.yaw_rad, 0, "zero yaw");
        near(result.solution[0].angles.upper_leg_rad, 0, "zero upper");
        near(result.solution[0].angles.lower_leg_rad, 0, "zero knee");
        for (const auto& local : std::array<vector3, 2>{{{0, -1, 0}, {0, 0, 0}}}) {
            const auto body = kinematics.leg_to_body_frame(leg, local);
            const auto failed = kinematics.inverse_candidates_in_body_frame(leg, body);
            const auto expected = local.y == -1
                ? leg_ik_status::unreachable : leg_ik_status::singular;
            require(failed.status == expected && failed.solution.empty(),
                "wrapper preserves geometry failure status");
        }
    }
    require_throws<std::out_of_range>([&] {
        (void)kinematics.inverse_candidates_in_body_frame(
            static_cast<leg_id>(99), {0, 0, 0});
    }, "wrapper rejects invalid leg");
}

void test_complete_solution(const robot_model& model) {
    const leg_kinematics kinematics(model);
    const body_ik_solver solver(model);
    const auto request = fixture(kinematics);
    const auto result = solver.solve(request);
    require(result.status == body_ik_status::success && result.solution,
        "six distinct leg targets solve");
    require(!result.failed_leg && !result.failure_reason,
        "success has no failure metadata");
    const auto& solution = *result.solution;
    for (std::size_t leg = 0; leg < leg_count; ++leg) {
        const auto& nominal = solution.nominal_angles[leg];
        const std::array<double, 3> nominal_components{
            nominal.yaw_rad, nominal.upper_leg_rad, nominal.lower_leg_rad};
        for (std::size_t axis = 0; axis < joints_per_leg; ++axis) {
            // 预期绑定为连续 1..18，独立于求解器内部查询过程。
            const auto id_index = leg * 3 + axis;
            require(solution.targets.desired_angles_rad[id_index]
                && solution.targets.nominal_positions[id_index],
                "all eighteen ideal angles and nominal positions are filled");
            near(*solution.targets.desired_angles_rad[id_index],
                degrees_to_radians(desired_degrees[leg][axis]),
                "ideal angle is preserved at physical ID");
            require(solution.targets.nominal_positions[id_index]->value()
                == expected_positions[leg][axis], "independent physical integer position");
            const int sign = (leg >= 3 && axis != 0) ? -1 : 1;
            near(nominal_components[axis], degrees_to_radians(
                sign * (expected_positions[leg][axis] - 500) * 0.24),
                "nominal reference reflects integer position");
            require(!solution.targets.commanded_positions[id_index],
                "IK does not create an execution command");
        }
        require(solution.branches[leg].radial == ik_branch_sign::positive
            && solution.branches[leg].knee == ik_branch_sign::negative,
            "six branch memories are returned");
    }
    auto next = request;
    next.reference_angles = solution.nominal_angles;
    next.remembered_branches = solution.branches;
    require(solver.solve(next).status == body_ik_status::success,
        "accepted nominal reference can be used on next frame");
    for (std::size_t leg = 0; leg < leg_count; ++leg) {
        near(request.reference_angles[leg].upper_leg_rad,
            degrees_to_radians(desired_degrees[leg][1]), "input reference remains unchanged");
        require(!request.remembered_branches[leg].knee,
            "input branch memory remains unchanged");
    }
}

void test_atomic_failure(const robot_model& model) {
    const leg_kinematics kinematics(model);
    const body_ik_solver solver(model);
    const std::array<vector3, 3> rejected_points{{{0, -1, 0}, {0, 0, 0}, {0, 0.2, 0}}};
    const std::array<leg_selection_status, 3> reasons{
        leg_selection_status::geometric_unreachable,
        leg_selection_status::geometric_singular,
        leg_selection_status::no_candidate_within_limits};
    for (const auto leg : leg_order) {
        for (std::size_t reason = 0; reason < reasons.size(); ++reason) {
            auto request = fixture(kinematics);
            request.points_in_body[leg_index(leg)] = kinematics.leg_to_body_frame(
                leg, rejected_points[reason]);
            require_failure(solver.solve(request), leg, reasons[reason]);
        }
    }
    auto request = fixture(kinematics);
    request.points_in_body[2] = kinematics.leg_to_body_frame(
        leg_id::right_rear, {0, -1, 0});
    request.points_in_body[5] = kinematics.leg_to_body_frame(
        leg_id::left_rear, {0, -1, 0});
    require_failure(solver.solve(request), leg_id::right_rear,
        leg_selection_status::geometric_unreachable);
    require(solver.solve(fixture(kinematics)).status == body_ik_status::success,
        "failure does not mutate solver state");

    request = fixture(kinematics);
    request.points_in_body[5] = kinematics.forward_in_body_frame(
        leg_id::left_rear, {0, 0, 0});
    require_failure(solver.solve(request), leg_id::left_rear,
        leg_selection_status::no_candidate_under_policy);
    request = fixture(kinematics);
    request.reference_angles[5] = {0, 0, 0};
    require_failure(solver.solve(request), leg_id::left_rear,
        leg_selection_status::reference_branch_undetermined);
}

void test_body_speed(const robot_model& model) {
    const leg_kinematics kinematics(model);
    const body_ik_solver solver(model);
    const auto solved = solver.solve(fixture(kinematics));
    require(solved.solution.has_value(), "speed fixture solves");
    const std::array<double leg_joint_angles::*, 3> members{
        &leg_joint_angles::yaw_rad, &leg_joint_angles::upper_leg_rad,
        &leg_joint_angles::lower_leg_rad};
    for (const auto leg : leg_order) {
        const auto index = leg_index(leg);
        for (std::size_t axis = 0; axis < joints_per_leg; ++axis) {
            auto request = fixture(kinematics);
            request.reference_angles = solved.solution->nominal_angles;
            request.reference_angles[index].*members[axis] -= degrees_to_radians(0.24);
            std::array<std::array<double, 3>, 6> speeds{};
            for (auto& speed : speeds) speed.fill(1);
            speeds[index][axis] = degrees_to_radians(0.2);
            request.connection.emplace(1, speeds);
            require_failure(solver.solve(request), leg,
                leg_selection_status::no_candidate_within_speed_limits);
            request.connection->duration_s = 2;
            require(solver.solve(request).status == body_ik_status::success,
                "longer common duration passes each of eighteen axis budgets");
        }
    }
    const std::array<double, 5> invalid{0, -1,
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()};
    std::array<std::array<double, 3>, 6> speeds{};
    for (auto& speed : speeds) speed.fill(1);
    for (const double value : invalid) {
        auto request = fixture(kinematics);
        request.connection.emplace(value, speeds);
        require_throws<std::invalid_argument>([&] { (void)solver.solve(request); },
            "invalid whole-body duration");
        for (std::size_t leg = 0; leg < leg_count; ++leg) {
            for (std::size_t axis = 0; axis < joints_per_leg; ++axis) {
                request.connection.emplace(1, speeds);
                request.connection->max_joint_speed_rad_s[leg][axis] = value;
                // 首腿不可达仍须先拒绝最后一腿等位置的非法速度。
                request.points_in_body[0] = kinematics.leg_to_body_frame(
                    leg_id::right_front, {0, -1, 0});
                require_throws<std::invalid_argument>([&] { (void)solver.solve(request); },
                    "complete speed validation precedes geometric early return");
            }
        }
    }
}

void test_errors(const robot_model& model, const json& root) {
    const leg_kinematics kinematics(model);
    const body_ik_solver solver(model);
    auto request = fixture(kinematics);
    request.points_in_body[5].x = std::numeric_limits<double>::quiet_NaN();
    require_throws<std::invalid_argument>([&] { (void)solver.solve(request); },
        "nonfinite body point propagates exception");
    request = fixture(kinematics);
    request.reference_angles[5].lower_leg_rad = degrees_to_radians(90);
    require_throws<std::out_of_range>([&] { (void)solver.solve(request); },
        "out-of-limit reference propagates exception");
    request = fixture(kinematics);
    request.remembered_branches[5].knee = ik_branch_sign::positive;
    require_throws<std::invalid_argument>([&] { (void)solver.solve(request); },
        "conflicting branch memory propagates exception");
    auto missing = root;
    missing["mounting"]["fixed_body_to_first_axis_transforms"]["left_rear"] = nullptr;
    const auto incomplete = robot_model::from_json_text(missing.dump());
    const body_ik_solver incomplete_solver(incomplete);
    require_throws<std::logic_error>([&] {
        (void)incomplete_solver.solve(fixture(kinematics));
    }, "missing leg installation is not treated as unreachable");
    auto reordered = root;
    std::reverse(reordered["legs"].begin(), reordered["legs"].end());
    test_complete_solution(robot_model::from_json_text(reordered.dump()));
}

}  // namespace

int main(int argc, char** argv) {
    try {
        require(argc == 2, "usage: body_ik_tests robot_hardware.json");
        std::ifstream input(argv[1]);
        require(input.good(), "hardware config opens");
        const auto root = json::parse(input);
        const auto model = robot_model::from_file(argv[1]);
        test_body_wrapper(leg_kinematics(model));
        test_complete_solution(model);
        test_atomic_failure(model);
        test_body_speed(model);
        test_errors(model, root);
        std::cout << "PASS: body IK wrapper, eighteen-axis assembly, atomic failure, "
            "speed propagation and invalid input.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
