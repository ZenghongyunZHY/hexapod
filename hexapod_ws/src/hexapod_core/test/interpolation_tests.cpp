#include "hexapod_core/joint_interpolation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
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

void angles_near(const leg_joint_angles& actual, const leg_joint_angles& expected,
    const std::string& message) {
    near(actual.yaw_rad, expected.yaw_rad, message + " yaw");
    near(actual.upper_leg_rad, expected.upper_leg_rad, message + " upper");
    near(actual.lower_leg_rad, expected.lower_leg_rad, message + " knee");
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

void test_known_samples() {
    const auto start = degrees(0, 12, -72);
    const auto end = degrees(6, 20, -60);
    const std::array<double, 5> ratios{0, 0.25, 0.5, 0.75, 1};
    const std::array<leg_joint_angles, 5> expected{
        start, degrees(1.5, 14, -69), degrees(3, 16, -66),
        degrees(4.5, 18, -63), end};
    for (std::size_t index = 0; index < ratios.size(); ++index) {
        angles_near(interpolate_leg_joint_angles(start, end, ratios[index]),
            expected[index], "independent quarter-point expectation");
        angles_near(interpolate_leg_joint_angles(end, start, 1 - ratios[index]),
            expected[index], "reverse segment agrees");
        angles_near(interpolate_leg_joint_angles(start, start, ratios[index]),
            start, "stationary segment stays fixed");
    }
    // 输入引用不被修改。
    angles_near(start, degrees(0, 12, -72), "start remains unchanged");
    angles_near(end, degrees(6, 20, -60), "end remains unchanged");

    // 纯数值用例，不是机器可执行姿态；明确检查普通插值不改为整圈最短路。
    angles_near(interpolate_leg_joint_angles(
        degrees(170, -170, 180), degrees(-170, 170, -180), 0.5),
        {0, 0, 0}, "ordinary angles are not wrapped");

    const std::array<double leg_joint_angles::*, 3> members{
        &leg_joint_angles::yaw_rad, &leg_joint_angles::upper_leg_rad,
        &leg_joint_angles::lower_leg_rad};
    for (int step = 0; step <= 64; ++step) {
        const auto sample = interpolate_leg_joint_angles(start, end, step / 64.0);
        for (const auto member : members) {
            require(sample.*member >= std::min(start.*member, end.*member)
                && sample.*member <= std::max(start.*member, end.*member),
                "ordinary samples remain between each pair of endpoints");
        }
    }
}

void test_invalid_input() {
    const leg_joint_angles start{0.25, -0.5, 0.75};
    const leg_joint_angles end{-0.75, 1, -1.25};
    const std::array<double, 7> invalid_ratios{
        -0.1, 1.1, std::nextafter(0.0, -1.0), std::nextafter(1.0, 2.0),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()};
    for (const double ratio : invalid_ratios) {
        require_throws<std::invalid_argument>([&] {
            (void)interpolate_leg_joint_angles(start, end, ratio);
        }, "invalid ratio is rejected without clamping");
    }
    const std::array<double leg_joint_angles::*, 3> members{
        &leg_joint_angles::yaw_rad, &leg_joint_angles::upper_leg_rad,
        &leg_joint_angles::lower_leg_rad};
    for (const double value : {std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()}) {
        for (const auto member : members) {
            auto bad_start = start;
            auto bad_end = end;
            bad_start.*member = value;
            bad_end.*member = value;
            for (const double ratio : {0.0, 0.5, 1.0}) {
                require_throws<std::invalid_argument>([&] {
                    (void)interpolate_leg_joint_angles(bad_start, end, ratio);
                }, "bad start is checked even at end ratio");
                require_throws<std::invalid_argument>([&] {
                    (void)interpolate_leg_joint_angles(start, bad_end, ratio);
                }, "bad end is checked even at start ratio");
            }
        }
    }
}

void test_large_finite_values() {
    // 纯浮点边界用例，不表示安装角范围。end-start 会溢出，但加权中点可表示。
    const double large = std::numeric_limits<double>::max();
    const leg_joint_angles start{-large, large, -large};
    const leg_joint_angles end{large, -large, -large};
    const auto middle = interpolate_leg_joint_angles(start, end, 0.5);
    require(middle.yaw_rad == 0 && middle.upper_leg_rad == 0
        && middle.lower_leg_rad == -large, "large finite midpoint avoids delta overflow");
    const auto first = interpolate_leg_joint_angles(start, end, 0);
    const auto last = interpolate_leg_joint_angles(start, end, 1);
    require(first.yaw_rad == start.yaw_rad && first.upper_leg_rad == start.upper_leg_rad
        && first.lower_leg_rad == start.lower_leg_rad, "start endpoint is returned exactly");
    require(last.yaw_rad == end.yaw_rad && last.upper_leg_rad == end.upper_leg_rad
        && last.lower_leg_rad == end.lower_leg_rad, "end endpoint is returned exactly");
}

void test_fk_path_counterexample(const std::string& config_path) {
    const auto model = robot_model::from_file(config_path);
    const leg_kinematics kinematics(model);
    // 合成离线运动段，非默认站姿、步态参数或实机动作。
    const auto start = degrees(0, -30, -30);
    const auto end = degrees(0, -30, -86.4);
    const auto middle = interpolate_leg_joint_angles(start, end, 0.5);
    const auto first_point = kinematics.forward_in_leg_frame(start);
    const auto middle_point = kinematics.forward_in_leg_frame(middle);
    const auto last_point = kinematics.forward_in_leg_frame(end);
    require(middle_point.z < std::min(first_point.z, last_point.z) - 0.005,
        "joint midpoint can put foot below both endpoints");
    // 该 5 mm 只是本反例的断言，不是机器人离地余量配置。
    std::cout << std::fixed << std::setprecision(3)
        << "OFFLINE EXAMPLE: local foot Z start/middle/end = "
        << first_point.z * 1000 << "/" << middle_point.z * 1000
        << "/" << last_point.z * 1000 << " mm.\n";
}

}  // namespace

int main(int argc, char** argv) {
    try {
        require(argc == 2, "usage: interpolation_tests robot_hardware.json");
        test_known_samples();
        test_invalid_input();
        test_large_finite_values();
        test_fk_path_counterexample(argv[1]);
        std::cout << "PASS: joint interpolation endpoints, fractions, invalid input, "
            "finite extremes and FK path counterexample.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
