#include "hexapod_core/foot_trajectory.hpp"
#include "hexapod_core/body_segment_planner.hpp"

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

void near(double actual, double expected, const std::string& message) {
    require(std::isfinite(actual) && std::abs(actual - expected) < 1e-12, message);
}

void point_near(const vector3& actual, const vector3& expected,
    const std::string& message) {
    near(actual.x, expected.x, message + " X");
    near(actual.y, expected.y, message + " Y");
    near(actual.z, expected.z, message + " Z");
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

const swing_trajectory_request simple{{0, 0, 0}, {0.04, 0, 0}, {0, 0, 1}, 0.02};

vector3 rotate_and_translate(const vector3& point) {
    return {point.z + 0.1, point.x - 0.2, point.y + 0.3};
}

void test_known_geometry() {
    const std::array<vector3, 5> expected{{
        {0, 0, 0}, {0.004140625, 0, 0.0084375}, {0.02, 0, 0.02},
        {0.035859375, 0, 0.0084375}, {0.04, 0, 0}
    }};
    const swing_trajectory_request reverse{simple.end_in_planning_frame,
        simple.start_in_planning_frame, {0, 0, 1}, 0.02};
    for (std::size_t index = 0; index < expected.size(); ++index) {
        const double ratio = index * 0.25;
        point_near(sample_swing_trajectory(simple, ratio), expected[index],
            "independent quarter-point expectation");
        point_near(sample_swing_trajectory(reverse, 1 - ratio), expected[index],
            "reversed endpoints traverse same curve");
    }
    const auto first = sample_swing_trajectory(simple, 0);
    const auto last = sample_swing_trajectory(simple, 1);
    require(first.x == 0 && first.y == 0 && first.z == 0
        && last.x == 0.04 && last.y == 0 && last.z == 0,
        "exact endpoint values");
    point_near(sample_swing_trajectory(
        {{0.01, -0.02, 0.03}, {0.05, 0.04, -0.01}, {0, 0.6, 0.8}, 0.02}, 0.5),
        {0.03, 0.022, 0.026}, "tilted lift direction and unequal endpoint heights");
    point_near(sample_swing_trajectory(
        {{0, 0, 0}, {0.04, 0, 0}, {0, 0, 1}, 0}, 0.25),
        {0.004140625, 0, 0}, "zero height retains smooth baseline");
    point_near(sample_swing_trajectory(
        {{0.01, 0.02, 0.03}, {0.01, 0.02, 0.03}, {1, 0, 0}, 0.02}, 0.5),
        {0.03, 0.02, 0.03}, "same endpoints can still lift along explicit X direction");

    const swing_trajectory_request transformed{
        rotate_and_translate(simple.start_in_planning_frame),
        rotate_and_translate(simple.end_in_planning_frame), {1, 0, 0}, 0.02};
    double previous_x = -1;
    for (int index = 0; index <= 128; ++index) {
        const double ratio = index / 128.0;
        const auto point = sample_swing_trajectory(simple, ratio);
        require(point.x >= previous_x && point.x >= 0 && point.x <= 0.04,
            "baseline progress stays ordered and within endpoints");
        require(point.z >= 0 && point.z <= 0.02, "lift remains between zero and peak");
        previous_x = point.x;
        point_near(sample_swing_trajectory(transformed, ratio), rotate_and_translate(point),
            "fixed rigid frame change preserves curve geometry");
        point_near(sample_swing_trajectory(reverse, 1 - ratio), point,
            "dense reverse symmetry");
    }
    // 起止附近位移按三阶缩小，数值验证端点的平缓性质。
    for (const bool at_end : {false, true}) {
        const auto endpoint = at_end ? simple.end_in_planning_frame : simple.start_in_planning_frame;
        const auto one = sample_swing_trajectory(simple, at_end ? 0.999 : 0.001);
        const auto two = sample_swing_trajectory(simple, at_end ? 0.998 : 0.002);
        const double d1 = std::hypot(one.x - endpoint.x, one.y - endpoint.y, one.z - endpoint.z);
        const double d2 = std::hypot(two.x - endpoint.x, two.y - endpoint.y, two.z - endpoint.z);
        require(d1 > 0 && d1 < 2e-9 && d2 / d1 > 7.9 && d2 / d1 < 8.1,
            "near-endpoint displacement has expected cubic flattening");
    }
    point_near(sample_swing_trajectory(simple, std::nextafter(0.0, 1.0)),
        simple.start_in_planning_frame, "nearest interior start ratio stays finite");
    point_near(sample_swing_trajectory(simple, std::nextafter(1.0, 0.0)),
        simple.end_in_planning_frame, "nearest interior end ratio stays finite");
}

void test_invalid_input() {
    for (const double ratio : {-0.1, 1.1, std::nextafter(0.0, -1.0),
        std::nextafter(1.0, 2.0), std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
        require_throws<std::invalid_argument>([&] { (void)sample_swing_trajectory(simple, ratio); },
            "invalid ratio is not clamped");
    }
    const std::array<vector3 swing_trajectory_request::*, 3> vectors{
        &swing_trajectory_request::start_in_planning_frame,
        &swing_trajectory_request::end_in_planning_frame,
        &swing_trajectory_request::lift_direction_in_planning_frame};
    const std::array<double vector3::*, 3> components{&vector3::x, &vector3::y, &vector3::z};
    for (const auto vector : vectors) {
        for (const auto component : components) {
            for (const double value : {std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
                auto bad = simple;
                (bad.*vector).*component = value;
                for (const double ratio : {0.0, 0.5, 1.0}) {
                    require_throws<std::invalid_argument>([&] {
                        (void)sample_swing_trajectory(bad, ratio);
                    }, "all vector inputs checked even at endpoints");
                }
            }
        }
    }
    for (const auto direction : std::array<vector3, 4>{{{0, 0, 0}, {0, 0, 2},
        {0, 0, 0.5}, {std::numeric_limits<double>::max(), 0, 0}}}) {
        auto bad = simple;
        bad.lift_direction_in_planning_frame = direction;
        require_throws<std::invalid_argument>([&] { (void)sample_swing_trajectory(bad, 0); },
            "unit direction required without automatic normalization");
    }
    for (const double height : {-1.0, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
        auto bad = simple;
        bad.lift_height_m = height;
        require_throws<std::invalid_argument>([&] { (void)sample_swing_trajectory(bad, 1); },
            "invalid height checked at endpoint");
    }
    const double large = std::numeric_limits<double>::max();
    point_near(sample_swing_trajectory({{-large, 0, 0}, {large, 0, 0}, {0, 0, 1}, 0}, 0.5),
        {0, 0, 0}, "weighted baseline avoids endpoint difference overflow");
    const swing_trajectory_request overflow{{large, 0, 0}, {large, 0, 0}, {1, 0, 0}, large};
    require_throws<std::overflow_error>([&] { (void)sample_swing_trajectory(overflow, 0.5); },
        "unrepresentable generated point throws rather than passing");
}

void test_offline_single_leg_sequence(const std::string& config_path) {
    const auto model = robot_model::from_file(config_path);
    const leg_kinematics kinematics(model);
    const joint_mapper mapper(model);
    const body_segment_planner planner(model);
    std::array<leg_joint_angles, leg_count> reference{};
    std::array<vector3, leg_count> fixed_points{};
    std::array<leg_branch_state, leg_count> branches{};
    servo_positions initial_positions{};
    // 候选理想姿态仅用于离线回归，不表示真实初始姿态或已验证站姿。
    const leg_joint_angles candidate{0, degrees_to_radians(12), degrees_to_radians(-72)};
    for (const auto leg : leg_order) {
        reference[leg_index(leg)] = candidate;
        const std::array<double, 3> angles{candidate.yaw_rad, candidate.upper_leg_rad,
            candidate.lower_leg_rad};
        for (std::size_t axis = 0; axis < joints_per_leg; ++axis) {
            const auto id = model.joint(leg, joint_order[axis]).binding.id;
            initial_positions[id.index()] = mapper.joint_angle_rad_to_servo_position(id, angles[axis]);
        }
        fixed_points[leg_index(leg)] = kinematics.forward_in_body_frame(leg, candidate);
    }
    const auto moving_leg = leg_id::right_middle;
    const auto moving_index = leg_index(moving_leg);
    const auto start = fixed_points[moving_index];
    const swing_trajectory_request swing{start, {start.x + 0.010, start.y, start.z},
        {0, 0, 1}, 0.008};
    // 合成参考平面比初始脚点低 2 mm，只用于本离线集成用例。
    const ground_plane_in_body ground{{0, 0, 1}, -start.z + 0.002};
    std::array<std::array<double, 3>, 6> speed_limits{};
    for (auto& row : speed_limits) row.fill(1);
    const leg_selection_options options{knee_singularity_policy::reject,
        branch_continuity_policy::keep_reference_branch};
    constexpr std::size_t segment_count = 20;
    double maximum_foot_error_m = 0;
    for (std::size_t segment = 1; segment <= segment_count; ++segment) {
        auto points = fixed_points;
        const double ratio = static_cast<double>(segment) / segment_count;
        points[moving_index] = sample_swing_trajectory(swing, ratio);
        const body_ik_request target{points, reference, branches, options,
            body_connection_request{2.0 / segment_count, speed_limits}};
        const auto result = planner.plan({target, ground, {}, 4});
        require(result.status == body_segment_status::checks_passed && result.solution,
            "curve-to-body-planner sequence segment " + std::to_string(segment));
        const auto& solution = *result.solution;
        for (const auto leg : leg_order) {
            for (const auto joint : joint_order) {
                const auto id = model.joint(leg, joint).binding.id;
                require(solution.targets.nominal_positions[id.index()]
                    && !solution.targets.commanded_positions[id.index()],
                    "complete nominal target without execution command");
                if (leg != moving_leg) {
                    require(solution.targets.nominal_positions[id.index()]->value()
                        == initial_positions[id.index()]->value(), "other fifteen axes remain fixed");
                }
            }
        }
        const auto actual_nominal_point = kinematics.forward_in_body_frame(
            moving_leg, solution.nominal_angles[moving_index]);
        const auto& ideal_point = points[moving_index];
        const double error = std::hypot(actual_nominal_point.x - ideal_point.x,
            actual_nominal_point.y - ideal_point.y, actual_nominal_point.z - ideal_point.z);
        maximum_foot_error_m = std::max(maximum_foot_error_m, error);
        require(error < 0.002, "offline nominal quantization error stays within test bound");
        reference = solution.nominal_angles;
        branches = solution.branches;
    }
    std::cout << "OFFLINE SINGLE-LEG SEQUENCE: 20 segments passed; max nominal FK error = "
        << maximum_foot_error_m * 1000 << " mm.\n";
}

}  // namespace

int main(int argc, char** argv) {
    try {
        require(argc == 2, "usage: foot_trajectory_tests robot_hardware.json");
        test_known_geometry();
        test_invalid_input();
        test_offline_single_leg_sequence(argv[1]);
        std::cout << "PASS: swing geometry, endpoint flattening, frame covariance, "
            "invalid input and offline single-leg integration.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
