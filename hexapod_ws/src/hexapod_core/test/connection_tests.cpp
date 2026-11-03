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

const leg_selection_options nearest{
    knee_singularity_policy::allow, branch_continuity_policy::nearest};
const leg_selection_options keep{
    knee_singularity_policy::reject,
    branch_continuity_policy::keep_reference_branch};

// 使用可精确表示的二进制数检查等号边界和逐轴拒绝。
void test_average_speed(const leg_candidate_selector& selector) {
    const leg_joint_angles reference{0, 0, 0};
    const leg_joint_angles target{0.25, 0.5, -0.75};
    const leg_connection_request request{2.0, {0.125, 0.25, 0.375}};
    for (const auto leg : leg_order) {
        const auto equal = selector.check_connection(leg, reference, target, request);
        require(equal.status == leg_connection_status::within_average_speed_limits,
            "exact speed boundary passes");
        require(equal.joint_passed == std::array<bool, 3>{true, true, true},
            "all axes pass at equality");
        const auto still = selector.check_connection(leg, target, target, request);
        require(still.status == leg_connection_status::within_average_speed_limits,
            "stationary target passes positive budgets");
        for (std::size_t axis = 0; axis < joints_per_leg; ++axis) {
            auto tighter = request;
            tighter.max_joint_speed_rad_s[axis] = std::nextafter(
                request.max_joint_speed_rad_s[axis], 0.0);
            const auto rejected = selector.check_connection(leg, target, reference, tighter);
            require(rejected.status == leg_connection_status::average_speed_limit_exceeded,
                "reverse motion over one limit fails");
            for (std::size_t other = 0; other < joints_per_leg; ++other) {
                require(rejected.joint_passed[other] == (axis != other),
                    "failed axis is reported independently");
            }
        }
    }
    const auto tiny = selector.check_connection(leg_id::right_middle,
        reference, target, {std::numeric_limits<double>::denorm_min(), {1, 1, 1}});
    require(tiny.status == leg_connection_status::average_speed_limit_exceeded,
        "overflowing required average speed is rejected");
}

void test_invalid_requests(const leg_kinematics& kinematics,
    const leg_candidate_selector& selector) {
    const auto reference = degrees(0, 12, -72);
    const auto ik = kinematics.inverse_candidates_in_leg_frame(
        kinematics.forward_in_leg_frame(reference));
    const std::array<double, 5> invalid{0, -1,
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()};
    for (const double value : invalid) {
        const leg_connection_request bad_duration{value, {1, 1, 1}};
        require_throws<std::invalid_argument>([&] {
            (void)selector.check_connection(leg_id::right_middle,
                reference, reference, bad_duration);
        }, "invalid direct duration");
        require_throws<std::invalid_argument>([&] {
            (void)selector.select_candidate(leg_id::right_middle,
                ik, reference, {}, keep, bad_duration);
        }, "invalid selection duration");
        for (std::size_t axis = 0; axis < joints_per_leg; ++axis) {
            leg_connection_request bad_speed{1, {1, 1, 1}};
            bad_speed.max_joint_speed_rad_s[axis] = value;
            require_throws<std::invalid_argument>([&] {
                (void)selector.check_connection(leg_id::right_middle,
                    reference, reference, bad_speed);
            }, "invalid direct speed");
            require_throws<std::invalid_argument>([&] {
                (void)selector.select_candidate(leg_id::right_middle,
                    ik, reference, {}, keep, bad_speed);
            }, "invalid selection speed");
        }
    }
    const std::array<double leg_joint_angles::*, 3> members{
        &leg_joint_angles::yaw_rad, &leg_joint_angles::upper_leg_rad,
        &leg_joint_angles::lower_leg_rad};
    for (const auto member : members) {
        auto bad = reference;
        bad.*member = std::numeric_limits<double>::quiet_NaN();
        require_throws<std::invalid_argument>([&] {
            (void)selector.check_connection(leg_id::right_middle,
                bad, reference, {1, {1, 1, 1}});
        }, "nonfinite reference");
        require_throws<std::invalid_argument>([&] {
            (void)selector.check_connection(leg_id::right_middle,
                reference, bad, {1, {1, 1, 1}});
        }, "nonfinite target");
        bad.*member = degrees_to_radians(90);
        require_throws<std::out_of_range>([&] {
            (void)selector.check_connection(leg_id::right_middle,
                reference, bad, {1, {1, 1, 1}});
        }, "target outside installed limits");
    }
}

void test_selection_integration(const leg_kinematics& kinematics,
    const leg_candidate_selector& selector) {
    auto ik = kinematics.inverse_candidates_in_leg_frame(
        kinematics.forward_in_leg_frame(degrees(0, 12, -72)));
    const auto reference = degrees(0, 10, 1);
    const auto unchecked = selector.select_candidate(leg_id::right_middle,
        ik, reference, {}, nearest);
    require(unchecked.selected && !unchecked.connection,
        "omitted request returns no connection result");
    require(unchecked.selected->positions[2].value() == 200,
        "unconstrained nearest candidate has negative knee");

    // 最近解的膝变化为 73 度；另一解为 71 度，但大腿变化较大。
    // 72 度/s 的膝预算允许后者，验证速度检查先于评分。
    const leg_connection_request budget{1, {1, 2, degrees_to_radians(72)}};
    for (int order = 0; order < 2; ++order) {
        const auto checked = selector.select_candidate(leg_id::right_middle,
            ik, reference, {}, nearest, budget);
        require(checked.selected && checked.connection,
            "alternative candidate survives speed rejection");
        require(checked.selected->positions[1].value() == 179
            && checked.selected->positions[2].value() == 800,
            "speed-feasible alternative is selected");
        const auto own = selector.check_connection(leg_id::right_middle,
            reference, checked.selected->nominal_angles, budget);
        require(checked.connection->status == own.status
            && checked.connection->joint_passed == own.joint_passed,
            "stored check belongs to selected candidate");
        std::reverse(ik.solution.begin(), ik.solution.end());
    }
    const auto rejected = selector.select_candidate(leg_id::right_middle,
        ik, reference, {}, nearest, leg_connection_request{1, {0.001, 0.001, 0.001}});
    require(rejected.status == leg_selection_status::no_candidate_within_speed_limits
        && !rejected.selected && !rejected.cost && !rejected.connection,
        "all speed rejections return no selected data");

    // 实际检查的是量化后的名义角，不是理想角。
    for (const auto leg : leg_order) {
        const auto rounded_down = kinematics.inverse_candidates_in_leg_frame(
            kinematics.forward_in_leg_frame(degrees(0, 12.1, -72)));
        const auto pass = selector.select_candidate(leg, rounded_down,
            degrees(0, 11.9, -72), {}, keep,
            leg_connection_request{1, {1, degrees_to_radians(0.15), 1}});
        require(pass.selected && pass.connection,
            "rounded-down nominal delta passes despite ideal delta");
        const auto rounded_up = kinematics.inverse_candidates_in_leg_frame(
            kinematics.forward_in_leg_frame(degrees(0, 12.13, -72)));
        const auto fail = selector.select_candidate(leg, rounded_up,
            degrees(0, 12, -72), {}, keep,
            leg_connection_request{1, {1, degrees_to_radians(0.18), 1}});
        require(fail.status == leg_selection_status::no_candidate_within_speed_limits,
            "rounded-up nominal delta fails despite smaller ideal delta");
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        require(argc == 2, "usage: connection_tests robot_hardware.json");
        const auto model = robot_model::from_file(argv[1]);
        const leg_kinematics kinematics(model);
        const leg_candidate_selector selector(model);
        test_average_speed(selector);
        test_invalid_requests(kinematics, selector);
        test_selection_integration(kinematics, selector);
        std::cout << "PASS: average-speed boundaries, invalid input, candidate fallback "
            "and nominal quantization.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
