#include "hexapod_core/leg_path_sampler.hpp"

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
    require(std::isfinite(actual) && std::abs(actual - expected) < 1e-12, message);
}

void same_angles(const leg_joint_angles& actual, const leg_joint_angles& expected,
    const std::string& message) {
    near(actual.yaw_rad, expected.yaw_rad, message + " yaw");
    near(actual.upper_leg_rad, expected.upper_leg_rad, message + " upper");
    near(actual.lower_leg_rad, expected.lower_leg_rad, message + " knee");
}

void same_point(const vector3& actual, const vector3& expected,
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

leg_joint_angles degrees(double yaw, double upper, double lower) {
    return {degrees_to_radians(yaw), degrees_to_radians(upper),
        degrees_to_radians(lower)};
}

void test_six_leg_geometry(const leg_path_sampler& sampler) {
    // 独立列出的 CAD 根部和安装角；不调用生产坐标变换生成预期。
    const std::array<vector3, 6> roots{{
        {0.10217, -0.063, 0.021}, {-0.00004, -0.079, 0.021},
        {-0.10223, -0.06294, 0.021}, {0.10223, 0.06294, 0.021},
        {0.00004, 0.079, 0.021}, {-0.10217, 0.063, 0.021}
    }};
    const std::array<double, 6> mount_degrees{45, 0, -45, 135, 180, -135};
    const auto start = degrees(0, 0, 0);
    const auto end = degrees(48, 0, 0);
    for (const auto leg : leg_order) {
        const auto index = leg_index(leg);
        for (const std::size_t count : {1U, 2U, 4U, 17U, 32U}) {
            const auto path = sampler.sample_linear_segment(leg, start, end, count);
            require(path.size() == count + 1, "N intervals produce N+1 samples");
            require(path.front().ratio == 0 && path.back().ratio == 1,
                "exact endpoint ratios");
            require(path.front().angles.yaw_rad == start.yaw_rad
                && path.back().angles.yaw_rad == end.yaw_rad,
                "exact endpoint angles");
            for (std::size_t sample = 0; sample < path.size(); ++sample) {
                const double ratio = static_cast<double>(sample) / count;
                near(path[sample].ratio, ratio, "uniform ratio");
                if (sample != 0) {
                    require(path[sample].ratio > path[sample - 1].ratio,
                        "ordinary sample ratios increase strictly");
                }
                same_angles(path[sample].angles, degrees(48 * ratio, 0, 0),
                    "independent yaw fraction");
                const double angle = degrees_to_radians(mount_degrees[index] + 48 * ratio);
                const auto& root = roots[index];
                same_point(path[sample].point_in_body,
                    {root.x + 0.22873 * std::sin(angle),
                        root.y - 0.22873 * std::cos(angle), root.z},
                    "independent body-space circular path");
            }
        }
    }
}

void test_general_segments(const robot_model& model) {
    const leg_path_sampler sampler(model);
    const leg_kinematics kinematics(model);
    const auto start = degrees(0, 12, -72);
    const auto end = degrees(6, 20, -60);
    const std::array<leg_joint_angles, 5> expected{
        start, degrees(1.5, 14, -69), degrees(3, 16, -66),
        degrees(4.5, 18, -63), end};
    for (const auto leg : leg_order) {
        const auto path = sampler.sample_linear_segment(leg, start, end, 4);
        const auto reverse = sampler.sample_linear_segment(leg, end, start, 4);
        const auto stationary = sampler.sample_linear_segment(leg, start, start, 4);
        for (std::size_t sample = 0; sample < expected.size(); ++sample) {
            same_angles(path[sample].angles, expected[sample],
                "independent combined-joint fraction");
            same_point(path[sample].point_in_body,
                kinematics.forward_in_body_frame(leg, expected[sample]),
                "saved body point agrees with expected-angle public FK");
            same_point(reverse[4 - sample].point_in_body, path[sample].point_in_body,
                "reverse segment traverses same points");
            same_angles(stationary[sample].angles, start, "stationary angles");
            same_point(stationary[sample].point_in_body, stationary.front().point_in_body,
                "stationary foot stays fixed");
        }
    }
    same_angles(start, degrees(0, 12, -72), "start input unchanged");
    same_angles(end, degrees(6, 20, -60), "end input unchanged");

    const auto dip = sampler.sample_linear_segment(leg_id::right_middle,
        degrees(0, -30, -30), degrees(0, -30, -86.4), 2);
    require(dip[1].point_in_body.z
        < std::min(dip[0].point_in_body.z, dip[2].point_in_body.z) - 0.005,
        "body-space samples retain intermediate foot dip");
    // 采样器负责计算，不负责执行限位；此用例不作为实机姿态。
    const auto diagnostic = sampler.sample_linear_segment(leg_id::right_middle,
        degrees(0, 0, 0), degrees(90, 0, 0), 2);
    same_angles(diagnostic.back().angles, degrees(90, 0, 0),
        "sampler does not silently clamp out-of-limit diagnostic input");
}

void test_errors(const robot_model& model, const json& root) {
    const leg_path_sampler sampler(model);
    const auto angles = degrees(0, 12, -72);
    require_throws<std::invalid_argument>([&] {
        (void)sampler.sample_linear_segment(leg_id::right_middle, angles, angles, 0);
    }, "zero intervals");
    for (const auto count : {std::vector<leg_path_sample>{}.max_size(),
        std::numeric_limits<std::size_t>::max()}) {
        require_throws<std::length_error>([&] {
            (void)sampler.sample_linear_segment(leg_id::right_middle, angles, angles, count);
        }, "impossible counts rejected before allocation or addition overflow");
    }
    require_throws<std::out_of_range>([&] {
        (void)sampler.sample_linear_segment(static_cast<leg_id>(99), angles, angles, 1);
    }, "invalid leg");
    const std::array<double leg_joint_angles::*, 3> members{
        &leg_joint_angles::yaw_rad, &leg_joint_angles::upper_leg_rad,
        &leg_joint_angles::lower_leg_rad};
    for (const auto member : members) {
        for (const double value : {std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity()}) {
            auto bad = angles;
            bad.*member = value;
            require_throws<std::invalid_argument>([&] {
                (void)sampler.sample_linear_segment(leg_id::right_middle, bad, angles, 4);
            }, "invalid start component");
            require_throws<std::invalid_argument>([&] {
                (void)sampler.sample_linear_segment(leg_id::right_middle, angles, bad, 4);
            }, "invalid end component");
        }
    }
    auto missing = root;
    missing["mounting"]["fixed_body_to_first_axis_transforms"]["left_rear"] = nullptr;
    const leg_path_sampler incomplete(robot_model::from_json_text(missing.dump()));
    require_throws<std::logic_error>([&] {
        (void)incomplete.sample_linear_segment(leg_id::left_rear, angles, angles, 4);
    }, "missing mounting");
    require(incomplete.sample_linear_segment(leg_id::right_middle, angles, angles, 1).size() == 2,
        "other complete leg remains usable");
}

}  // namespace

int main(int argc, char** argv) {
    try {
        require(argc == 2, "usage: path_sampler_tests robot_hardware.json");
        std::ifstream input(argv[1]);
        require(input.good(), "hardware config opens");
        const auto root = json::parse(input);
        const auto model = robot_model::from_file(argv[1]);
        test_six_leg_geometry(leg_path_sampler(model));
        test_general_segments(model);
        test_errors(model, root);
        std::cout << "PASS: path sample counts, independent six-leg body geometry, "
            "fractions, reverse/stationary paths, intermediate dip and invalid input.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
