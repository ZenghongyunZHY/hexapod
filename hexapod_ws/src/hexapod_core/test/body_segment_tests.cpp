#include "hexapod_core/body_segment_planner.hpp"

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

const std::array<std::array<double, 3>, 6> desired_degrees{{
    {0, 12.1, -72}, {2.4, 14.5, -67.2}, {-4.8, 9.7, -76.8},
    {7.2, 16.9, -62.4}, {-9.6, 19.3, -57.6}, {12, 21.7, -52.8}
}};
const std::array<int, 18> expected_positions{
    500, 550, 200, 510, 560, 220, 480, 540, 180,
    530, 430, 760, 460, 420, 740, 550, 410, 720};

// 全部姿态、平面及速度均为离线测试输入，不作为运行配置。
body_segment_request fixture(const leg_kinematics& kinematics) {
    std::array<vector3, leg_count> points{};
    std::array<leg_joint_angles, leg_count> references{};
    for (const auto leg : leg_order) {
        const auto index = leg_index(leg);
        const auto& angle = desired_degrees[index];
        references[index] = degrees(angle[0], angle[1], angle[2]);
        points[index] = kinematics.forward_in_body_frame(leg, references[index]);
    }
    const leg_selection_options options{
        knee_singularity_policy::reject,
        branch_continuity_policy::keep_reference_branch};
    return {body_ik_request{points, references, {}, options}, {{0, 0, 1}, 1}, {}, 4};
}

void require_pass(const body_segment_result& result) {
    require(result.status == body_segment_status::checks_passed && result.solution,
        "whole segment passes and returns complete solution");
    require(!result.failed_leg && !result.selection_failure && !result.path_failure,
        "successful result has no failure metadata");
}

void require_selection_failure(const body_segment_result& result, leg_id leg,
    leg_selection_status reason) {
    require(result.status == body_segment_status::target_selection_failed
        && result.failed_leg == leg && result.selection_failure == reason,
        "selection failure preserves leg and reason");
    require(!result.solution && !result.path_failure,
        "selection failure returns no solution or path failure");
}

void require_path_failure(const body_segment_result& result, leg_id leg) {
    require(result.status == body_segment_status::path_check_failed
        && result.failed_leg == leg && result.path_failure,
        "path failure preserves leg and detail");
    require(!result.solution && !result.selection_failure,
        "path failure returns no partially accepted solution");
    require(result.path_failure->status
        == leg_path_check_status::foot_clearance_below_minimum,
        "plane failure status survives composition");
    require(result.path_failure->failed_sample_index
        && result.path_failure->failed_sample
        && result.path_failure->failed_clearance_m
        && !result.path_failure->failed_joint,
        "nested path diagnostics remain complete and consistent");
}

void test_success(const robot_model& model) {
    const leg_kinematics kinematics(model);
    const body_segment_planner planner(model);
    const auto request = fixture(kinematics);
    const auto result = planner.plan(request);
    require_pass(result);
    for (std::size_t id = 0; id < joint_count; ++id) {
        require(result.solution->targets.nominal_positions[id]
            && result.solution->targets.desired_angles_rad[id],
            "all eighteen target fields survive composition");
        require(result.solution->targets.nominal_positions[id]->value()
            == expected_positions[id], "independent eighteen-axis integer expectations");
        require(!result.solution->targets.commanded_positions[id],
            "planning still does not create execution commands");
    }
    for (std::size_t index = 0; index < leg_count; ++index) {
        near(result.solution->nominal_angles[index].upper_leg_rad,
            degrees_to_radians(desired_degrees[index][1] - 0.1),
            "nominal reference survives whole-segment result");
        require(result.solution->branches[index].knee == ik_branch_sign::negative,
            "branch memory survives whole-segment result");
        near(request.target_request.reference_angles[index].upper_leg_rad,
            degrees_to_radians(desired_degrees[index][1]), "caller reference is unchanged");
        require(!request.target_request.remembered_branches[index].knee,
            "caller branch memory is unchanged");
    }
    auto next = request;
    next.target_request.reference_angles = result.solution->nominal_angles;
    next.target_request.remembered_branches = result.solution->branches;
    require_pass(planner.plan(next));
}

void test_selection_failures(const robot_model& model) {
    const leg_kinematics kinematics(model);
    const body_segment_planner planner(model);
    const std::array<vector3, 3> points{{{0, -1, 0}, {0, 0, 0}, {0, 0.2, 0}}};
    const std::array<leg_selection_status, 3> reasons{
        leg_selection_status::geometric_unreachable,
        leg_selection_status::geometric_singular,
        leg_selection_status::no_candidate_within_limits};
    for (const auto leg : leg_order) {
        for (std::size_t reason = 0; reason < reasons.size(); ++reason) {
            auto request = fixture(kinematics);
            request.target_request.points_in_body[leg_index(leg)] =
                kinematics.leg_to_body_frame(leg, points[reason]);
            require_selection_failure(planner.plan(request), leg, reasons[reason]);
        }
    }
    auto request = fixture(kinematics);
    request.target_request.reference_angles[5].yaw_rad -= degrees_to_radians(0.24);
    std::array<std::array<double, 3>, 6> speeds{};
    for (auto& row : speeds) row.fill(1);
    speeds[5][0] = degrees_to_radians(0.2);
    request.target_request.connection.emplace(1, speeds);
    require_selection_failure(planner.plan(request), leg_id::left_rear,
        leg_selection_status::no_candidate_within_speed_limits);
    request.target_request.connection->duration_s = 2;
    require_pass(planner.plan(request));
}

void test_path_failures(const robot_model& model) {
    const leg_kinematics kinematics(model);
    const body_segment_planner planner(model);
    for (const auto leg : leg_order) {
        auto request = fixture(kinematics);
        request.minimum_clearance_m[leg_index(leg)] = 2;
        const auto result = planner.plan(request);
        require_path_failure(result, leg);
        require(result.path_failure->failed_sample_index == 0
            && result.path_failure->checked_sample_count == 1,
            "each leg's own clearance budget is used");
    }
    auto request = fixture(kinematics);
    request.minimum_clearance_m[2] = 2;
    request.minimum_clearance_m[5] = 2;
    require_path_failure(planner.plan(request), leg_id::right_rear);

    // 只改变末腿：端点允许，中间下探；前五腿通过也不能返回部分目标。
    request = fixture(kinematics);
    request.ground.offset_m = 0.12;
    request.target_request.reference_angles[5] = degrees(0, -30, -30);
    request.target_request.points_in_body[5] = kinematics.forward_in_body_frame(
        leg_id::left_rear, degrees(0, -30, -86.4));
    request.interval_count = 1;
    require_pass(planner.plan(request));
    request.interval_count = 2;
    const auto dip = planner.plan(request);
    require_path_failure(dip, leg_id::left_rear);
    require(dip.path_failure->failed_sample_index == 1
        && dip.path_failure->checked_sample_count == 2, "midpoint diagnostics are retained");
    near(dip.path_failure->failed_sample->ratio, 0.5, "midpoint ratio");
    near(dip.path_failure->failed_sample->angles.lower_leg_rad,
        degrees_to_radians(-58.2), "midpoint joint angle");
    require_pass(planner.plan(fixture(kinematics)));
}

void test_nominal_endpoint(const robot_model& model) {
    const leg_kinematics kinematics(model);
    const body_segment_planner planner(model);
    auto request = fixture(kinematics);
    const auto initial = planner.plan(request);
    require_pass(initial);
    const auto ideal = request.target_request.reference_angles[5];
    const auto nominal = initial.solution->nominal_angles[5];
    const auto ideal_point = kinematics.forward_in_body_frame(leg_id::left_rear, ideal);
    const auto nominal_point = kinematics.forward_in_body_frame(leg_id::left_rear, nominal);
    require(nominal_point.z < ideal_point.z, "fixture quantization lowers foot slightly");
    request.minimum_clearance_m[5] = request.ground.offset_m
        + (ideal_point.z + nominal_point.z) / 2;
    const auto result = planner.plan(request);
    require_path_failure(result, leg_id::left_rear);
    require(result.path_failure->failed_sample->ratio > 0,
        "ideal reference passes before interpolating toward nominal target");
    const double ratio = result.path_failure->failed_sample->ratio;
    near(result.path_failure->failed_sample->angles.upper_leg_rad,
        degrees_to_radians(21.7 - 0.1 * ratio),
        "path interpolates toward decoded nominal angle rather than ideal angle");
}

void test_invalid_input(const robot_model& model) {
    const leg_kinematics kinematics(model);
    const body_segment_planner planner(model);
    auto request = fixture(kinematics);
    request.interval_count = 0;
    require_throws<std::invalid_argument>([&] { (void)planner.plan(request); },
        "zero intervals");
    request.interval_count = std::numeric_limits<std::size_t>::max();
    require_throws<std::length_error>([&] { (void)planner.plan(request); },
        "impossible sample count");
    for (std::size_t leg = 0; leg < leg_count; ++leg) {
        for (const double value : {-1.0, std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
            request = fixture(kinematics);
            request.minimum_clearance_m[leg] = value;
            request.target_request.points_in_body[0] = kinematics.leg_to_body_frame(
                leg_id::right_front, {0, -1, 0});
            require_throws<std::invalid_argument>([&] { (void)planner.plan(request); },
                "every clearance validated before early selection failure");
        }
    }
    request = fixture(kinematics);
    request.ground.unit_normal = {0, 0, 2};
    require_throws<std::invalid_argument>([&] { (void)planner.plan(request); },
        "shared plane validation is reused");
    request = fixture(kinematics);
    request.ground.offset_m = std::numeric_limits<double>::quiet_NaN();
    require_throws<std::invalid_argument>([&] { (void)planner.plan(request); },
        "invalid shared offset");
    request = fixture(kinematics);
    request.target_request.reference_angles[5].lower_leg_rad = degrees_to_radians(90);
    require_throws<std::out_of_range>([&] { (void)planner.plan(request); },
        "invalid reference is not converted to a normal selection failure");
}

}  // namespace

int main(int argc, char** argv) {
    try {
        require(argc == 2, "usage: body_segment_tests robot_hardware.json");
        const auto model = robot_model::from_file(argv[1]);
        test_success(model);
        test_selection_failures(model);
        test_path_failures(model);
        test_nominal_endpoint(model);
        test_invalid_input(model);
        std::cout << "PASS: complete body segment, selection/path failure separation, "
            "atomic rejection, nominal endpoint and request validation.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
