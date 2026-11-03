#include "hexapod_core/leg_path_checker.hpp"

#include <nlohmann/json.hpp>

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
    require(std::isfinite(actual) && std::abs(actual - expected) < 1e-12, message);
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

// 所有平面均为显式离线测试输入，不写入实际机器人配置。
const leg_path_constraints permissive{{{0, 0, 1}, 1}, 0};

void require_pass(const leg_path_check_result& result, std::size_t count) {
    require(result.status == leg_path_check_status::samples_passed
        && result.checked_sample_count == count, "passed status and checked count");
    require(!result.failed_sample_index && !result.failed_sample
        && !result.failed_joint && !result.failed_clearance_m,
        "passed result has no failure metadata");
}

void require_failure(const leg_path_check_result& result,
    leg_path_check_status status, std::size_t index, double ratio) {
    require(result.status == status, "failure status");
    require(result.checked_sample_count == index + 1
        && result.failed_sample_index == index && result.failed_sample,
        "failure index and count include failing point");
    near(result.failed_sample->ratio, ratio, "failure ratio");
    require(result.failed_joint.has_value()
        == (status == leg_path_check_status::joint_limit_exceeded),
        "joint metadata belongs only to joint failure");
    require(result.failed_clearance_m.has_value()
        == (status == leg_path_check_status::foot_clearance_below_minimum),
        "clearance metadata belongs only to plane failure");
}

void test_clearance(const robot_model& model) {
    const leg_path_checker checker(model);
    const leg_joint_angles zero{0, 0, 0};
    for (const auto leg : leg_order) {
        require_pass(checker.check_linear_segment(leg,
            degrees(0, 12, -72), degrees(6, 20, -60), 4, permissive), 5);
        // 六腿零位脚点的机身 Z 都是 CAD 根部高度 21 mm。
        const leg_path_constraints equality{{{0, 0, 1}, 0}, 0.021};
        require_pass(checker.check_linear_segment(leg, zero, zero, 1, equality), 2);
        auto stricter = equality;
        stricter.minimum_clearance_m = std::nextafter(0.021, 1.0);
        const auto failed = checker.check_linear_segment(leg, zero, zero, 1, stricter);
        require_failure(failed, leg_path_check_status::foot_clearance_below_minimum, 0, 0);
        near(*failed.failed_clearance_m, 0.021, "actual distance is reported, not margin");
        require_pass(checker.check_linear_segment(leg, zero, zero, 1,
            {{{0, 0, 1}, -0.021}, 0}), 2);
    }
    const auto start = degrees(0, -30, -30);
    const auto end = degrees(0, -30, -86.4);
    const leg_path_constraints plane{{{0, 0, 1}, 0.12}, 0};
    // 端点采样会漏过内部下探；明确保留这一离散检查的局限。
    require_pass(checker.check_linear_segment(leg_id::right_middle,
        start, end, 1, plane), 2);
    const auto dip = checker.check_linear_segment(leg_id::right_middle,
        start, end, 2, plane);
    require_failure(dip, leg_path_check_status::foot_clearance_below_minimum, 1, 0.5);
    const double expected_z = 0.021 + 0.0742 * std::sin(degrees_to_radians(-30))
        + 0.11253 * std::sin(degrees_to_radians(-88.2));
    near(dip.failed_sample->angles.lower_leg_rad, degrees_to_radians(-58.2),
        "failing sample preserves interpolated knee");
    near(dip.failed_sample->point_in_body.z, expected_z, "independent body Z at midpoint");
    near(*dip.failed_clearance_m, expected_z + 0.12, "signed clearance at midpoint");

    const auto last = checker.check_linear_segment(leg_id::left_rear,
        degrees(0, 0, -30), degrees(0, 0, -60), 2,
        {{{0, 0, 1}, 0.07}, 0});
    require_failure(last, leg_path_check_status::foot_clearance_below_minimum, 2, 1);
    const auto first = checker.check_linear_segment(leg_id::right_middle,
        start, end, 2, {{{0, 0, 1}, 0.11}, 0});
    require_failure(first, leg_path_check_status::foot_clearance_below_minimum, 0, 0);

    // 右中零位 p=(−0.00004,−0.30773,0.021)，倾斜面需计入 Y 分量。
    const auto tilted = checker.check_linear_segment(leg_id::right_middle,
        zero, zero, 2, {{{0, 0.6, 0.8}, 0.125}, 0});
    require_failure(tilted, leg_path_check_status::foot_clearance_below_minimum, 0, 0);
    near(*tilted.failed_clearance_m, -0.042838, "independent tilted plane distance");
    require_pass(checker.check_linear_segment(leg_id::right_middle,
        zero, zero, 2, {{{0, -0.6, -0.8}, -0.125}, 0}), 3);
}

void test_joint_limits(const robot_model& model) {
    const leg_path_checker checker(model);
    const leg_joint_angles zero{0, 0, 0};
    const std::array<double leg_joint_angles::*, 3> members{
        &leg_joint_angles::yaw_rad, &leg_joint_angles::upper_leg_rad,
        &leg_joint_angles::lower_leg_rad};
    for (const auto leg : leg_order) {
        for (std::size_t axis = 0; axis < joints_per_leg; ++axis) {
            for (const double sign : {-1.0, 1.0}) {
                auto outside = zero;
                outside.*members[axis] = degrees_to_radians(sign * 90);
                const auto first = checker.check_linear_segment(leg,
                    outside, zero, 2, permissive);
                require_failure(first, leg_path_check_status::joint_limit_exceeded, 0, 0);
                require(first.failed_joint == joint_order[axis], "physical joint type reported");
                const auto last = checker.check_linear_segment(leg,
                    zero, outside, 2, permissive);
                require_failure(last, leg_path_check_status::joint_limit_exceeded, 2, 1);
                require(last.failed_joint == joint_order[axis], "last-point joint type");

                auto boundary = zero;
                boundary.*members[axis] = degrees_to_radians(sign * 86.4);
                require_pass(checker.check_linear_segment(leg, boundary, boundary, 1,
                    permissive), 2);
                auto numerical_roundoff = boundary;
                numerical_roundoff.*members[axis] += sign * 0.5e-12;
                require_pass(checker.check_linear_segment(leg,
                    numerical_roundoff, numerical_roundoff, 1, permissive), 2);
                boundary.*members[axis] += sign * 2e-12;
                require_failure(checker.check_linear_segment(leg, boundary, boundary, 1,
                    permissive), leg_path_check_status::joint_limit_exceeded, 0, 0);
            }
        }
    }
    const auto both = checker.check_linear_segment(leg_id::right_middle,
        degrees(90, 90, 90), zero, 2, {{{0, 0, 1}, -1}, 0});
    require_failure(both, leg_path_check_status::joint_limit_exceeded, 0, 0);
    require(both.failed_joint == joint_type::body_yaw,
        "within one point yaw precedes other joints and plane failure");
}

void test_invalid_constraints(const robot_model& model) {
    const leg_path_checker checker(model);
    const auto angles = degrees(0, 12, -72);
    const std::array<double vector3::*, 3> components{&vector3::x, &vector3::y, &vector3::z};
    const std::array<double, 3> nonfinite{std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
    for (const double value : nonfinite) {
        for (const auto component : components) {
            auto constraints = permissive;
            constraints.ground.unit_normal.*component = value;
            require_throws<std::invalid_argument>([&] {
                (void)checker.check_linear_segment(leg_id::right_middle,
                    angles, angles, 1, constraints);
            }, "nonfinite normal component");
        }
        auto constraints = permissive;
        constraints.ground.offset_m = value;
        require_throws<std::invalid_argument>([&] {
            (void)checker.check_linear_segment(leg_id::right_middle,
                angles, angles, 1, constraints);
        }, "nonfinite offset");
        constraints = permissive;
        constraints.minimum_clearance_m = value;
        require_throws<std::invalid_argument>([&] {
            (void)checker.check_linear_segment(leg_id::right_middle,
                angles, angles, 1, constraints);
        }, "nonfinite minimum clearance");
    }
    for (const auto normal : std::array<vector3, 4>{{{0, 0, 0}, {0, 0, 2},
        {0, 0, 0.5}, {std::numeric_limits<double>::max(), 0, 0}}}) {
        require_throws<std::invalid_argument>([&] {
            (void)checker.check_linear_segment(leg_id::right_middle,
                angles, angles, 1, {{normal, 0}, 0});
        }, "non-unit normal is rejected rather than normalized");
    }
    require_throws<std::invalid_argument>([&] {
        (void)checker.check_linear_segment(static_cast<leg_id>(99),
            angles, angles, 1, {{{0, 0, 1}, 1}, -0.001});
    }, "constraints checked before model lookup");
}

void test_errors(const robot_model& model, const json& root) {
    const leg_path_checker checker(model);
    const auto angles = degrees(0, 12, -72);
    require_throws<std::out_of_range>([&] {
        (void)checker.check_linear_segment(static_cast<leg_id>(99),
            angles, angles, 1, permissive);
    }, "invalid leg");
    require_throws<std::invalid_argument>([&] {
        (void)checker.check_linear_segment(leg_id::right_middle,
            angles, angles, 0, permissive);
    }, "zero intervals");
    require_throws<std::length_error>([&] {
        (void)checker.check_linear_segment(leg_id::right_middle,
            angles, angles, std::numeric_limits<std::size_t>::max(), permissive);
    }, "interval overflow");
    auto bad = angles;
    bad.lower_leg_rad = std::numeric_limits<double>::quiet_NaN();
    require_throws<std::invalid_argument>([&] {
        (void)checker.check_linear_segment(leg_id::right_middle,
            angles, bad, 1, permissive);
    }, "invalid angle propagates");

    for (const std::string field : {"joint_angle_signs", "joint_reference_angles_deg",
        "installed_joint_limits"}) {
        auto missing = root;
        missing["mounting"][field]["18"] = nullptr;
        const leg_path_checker incomplete(robot_model::from_json_text(missing.dump()));
        require_throws<std::logic_error>([&] {
            (void)incomplete.check_linear_segment(leg_id::left_rear,
                angles, angles, 1, permissive);
        }, "missing mapping or limits");
        require_pass(incomplete.check_linear_segment(leg_id::right_middle,
            angles, angles, 1, permissive), 2);
    }
    auto missing = root;
    missing["mounting"]["fixed_body_to_first_axis_transforms"]["left_rear"] = nullptr;
    const leg_path_checker incomplete(robot_model::from_json_text(missing.dump()));
    require_throws<std::logic_error>([&] {
        (void)incomplete.check_linear_segment(leg_id::left_rear,
            angles, angles, 1, permissive);
    }, "missing installation propagates");

    // 纯合成数值模型：所有配置分量有限，距离求和仍可能溢出。
    auto large = root;
    large["mounting"]["leg_root_positions_mm"]["right_middle"] = {0, 0, 1e308};
    large["mounting"]["fixed_body_to_first_axis_transforms"]["right_middle"]
        ["translation_mm"] = {0, 0, 1e308};
    const leg_path_checker extreme(robot_model::from_json_text(large.dump()));
    require_throws<std::overflow_error>([&] {
        (void)extreme.check_linear_segment(leg_id::right_middle,
            angles, angles, 1, {{{0, 0, 1}, std::numeric_limits<double>::max()}, 0});
    }, "nonfinite computed clearance cannot pass");
}

}  // namespace

int main(int argc, char** argv) {
    try {
        require(argc == 2, "usage: path_checker_tests robot_hardware.json");
        std::ifstream input(argv[1]);
        require(input.good(), "hardware config opens");
        const auto root = json::parse(input);
        const auto model = robot_model::from_file(argv[1]);
        test_clearance(model);
        test_joint_limits(model);
        test_invalid_constraints(model);
        test_errors(model, root);
        std::cout << "PASS: clearance boundaries, intermediate dip, tilted planes, "
            "eighteen-axis limits, failure metadata and invalid input.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
