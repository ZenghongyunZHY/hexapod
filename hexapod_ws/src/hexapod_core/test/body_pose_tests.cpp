#include "hexapod_core/body_pose.hpp"

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
    const double scale = std::max(1.0, std::abs(expected));
    require(std::isfinite(actual) && std::abs(actual - expected) <= 1e-12 * scale,
        message);
}

void point_near(const vector3& actual, const vector3& expected,
    const std::string& message) {
    near(actual.x, expected.x, message + " x");
    near(actual.y, expected.y, message + " y");
    near(actual.z, expected.z, message + " z");
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

void test_independent_geometry() {
    const planar_body_pose identity{{0.0, 0.0, 0.0}, 0.0};
    const vector3 point{0.2, -0.15, -0.1};
    point_near(body_to_planning_frame(identity, point), point, "identity forward");
    point_near(planning_to_body_frame(identity, point), point, "identity inverse");

    const planar_body_pose translation{{0.1, -0.2, 0.3}, 0.0};
    point_near(body_to_planning_frame(translation, point),
        {0.3, -0.35, 0.2}, "independent translation forward");
    point_near(planning_to_body_frame(translation, {0.3, -0.35, 0.2}),
        point, "independent translation inverse");

    const planar_body_pose left_turn{{0.1, -0.2, 0.3}, pi / 2.0};
    point_near(body_to_planning_frame(left_turn, {0.2, 0.05, -0.1}),
        {0.05, 0.0, 0.2}, "positive quarter turn and translation");
    point_near(planning_to_body_frame(left_turn, {0.3, -0.1, 0.2}),
        {0.1, -0.2, -0.1}, "inverse positive quarter turn");

    const planar_body_pose right_turn{{0.1, -0.2, 0.3}, -pi / 2.0};
    point_near(body_to_planning_frame(right_turn, {0.2, 0.05, -0.1}),
        {0.15, -0.4, 0.2}, "negative quarter turn and translation");
    point_near(planning_to_body_frame(right_turn, {0.3, -0.1, 0.2}),
        {-0.1, 0.2, -0.1}, "inverse negative quarter turn");

    const planar_body_pose half_turn{{0.1, -0.2, 0.3}, pi};
    point_near(body_to_planning_frame(half_turn, {0.2, 0.05, -0.1}),
        {-0.1, -0.25, 0.2}, "half turn");

    // 30 度的独立几何预期：单位 X 向量转为 (sqrt(3)/2, 1/2)。
    const planar_body_pose thirty_degrees{{0.0, 0.0, 0.0}, pi / 6.0};
    point_near(body_to_planning_frame(thirty_degrees, {1.0, 0.0, 0.0}),
        {std::sqrt(3.0) / 2.0, 0.5, 0.0}, "independent thirty-degree direction");

    for (const double yaw : {-pi, -pi / 2.0, 0.0, pi / 3.0, 4.0 * pi}) {
        const planar_body_pose pose{{0.13, -0.27, 0.11}, yaw};
        point_near(body_to_planning_frame(pose, {0.0, 0.0, 0.0}),
            pose.body_origin_in_planning_frame, "body origin maps to pose translation");
        point_near(planning_to_body_frame(pose, pose.body_origin_in_planning_frame),
            {0.0, 0.0, 0.0}, "pose translation maps to body origin");
    }
}

void test_round_trips_and_rigidity() {
    const std::array<double, 9> angles{
        -4.0 * pi, -pi, -pi / 2.0, -0.37, 0.0, 0.37, pi / 2.0, pi, 4.0 * pi};
    const std::array<vector3, 4> points{{
        {0.0, 0.0, 0.0}, {0.2, -0.15, -0.1},
        {-0.14, 0.03, 0.01}, {0.41, 0.32, -0.27}}};
    for (const double yaw : angles) {
        const planar_body_pose pose{{0.13, -0.27, 0.11}, yaw};
        for (const auto& point : points) {
            const auto transformed = body_to_planning_frame(pose, point);
            point_near(planning_to_body_frame(pose, transformed), point,
                "body to planning to body round trip");
            const auto inverse = planning_to_body_frame(pose, point);
            point_near(body_to_planning_frame(pose, inverse), point,
                "planning to body to planning round trip");
        }
        for (std::size_t first = 0; first < points.size(); ++first) {
            for (std::size_t second = first + 1; second < points.size(); ++second) {
                const auto& left = points[first];
                const auto& right = points[second];
                const auto moved_left = body_to_planning_frame(pose, left);
                const auto moved_right = body_to_planning_frame(pose, right);
                const double original_distance = std::hypot(
                    left.x - right.x, left.y - right.y, left.z - right.z);
                near(std::hypot(moved_left.x - moved_right.x,
                         moved_left.y - moved_right.y, moved_left.z - moved_right.z),
                    original_distance, "rigid transform preserves pairwise distances");
            }
        }
        require(pose.yaw_rad == yaw
            && pose.body_origin_in_planning_frame.x == 0.13
            && pose.body_origin_in_planning_frame.y == -0.27
            && pose.body_origin_in_planning_frame.z == 0.11,
            "input pose remains unchanged");
    }
}

void test_fixed_support_point() {
    // 合成坐标用例，不表示实际站姿、地面标定或触地观测。
    const vector3 fixed_point{0.2, -0.15, 0.0};
    const planar_body_pose initial{{0.0, 0.0, 0.1}, 0.0};
    const planar_body_pose forward{{0.01, 0.0, 0.1}, 0.0};
    const planar_body_pose sideways{{0.0, 0.01, 0.1}, 0.0};
    const planar_body_pose turning{{0.01, 0.02, 0.1}, pi / 2.0};
    point_near(planning_to_body_frame(initial, fixed_point),
        {0.2, -0.15, -0.1}, "initial support point");
    point_near(planning_to_body_frame(forward, fixed_point),
        {0.19, -0.15, -0.1}, "body forward makes support target move backward");
    point_near(planning_to_body_frame(sideways, fixed_point),
        {0.2, -0.16, -0.1}, "body left makes support target move right");
    point_near(planning_to_body_frame(turning, fixed_point),
        {-0.17, -0.19, -0.1}, "combined body translation and turn");
    for (const auto& pose : {initial, forward, sideways, turning}) {
        const auto body_point = planning_to_body_frame(pose, fixed_point);
        point_near(body_to_planning_frame(pose, body_point), fixed_point,
            "same planning support point under different poses");
    }
    require(fixed_point.x == 0.2 && fixed_point.y == -0.15 && fixed_point.z == 0.0,
        "planning support point remains unchanged");
}

void test_invalid_input() {
    const planar_body_pose valid{{0.1, -0.2, 0.3}, 0.4};
    const vector3 point{0.2, -0.15, -0.1};
    const std::array<double vector3::*, 3> coordinates{
        &vector3::x, &vector3::y, &vector3::z};
    for (const double bad : {std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::infinity(),
             -std::numeric_limits<double>::infinity()}) {
        for (const auto coordinate : coordinates) {
            auto invalid_pose = valid;
            invalid_pose.body_origin_in_planning_frame.*coordinate = bad;
            require_throws<std::invalid_argument>([&] {
                (void)body_to_planning_frame(invalid_pose, point);
            }, "forward rejects nonfinite origin coordinate");
            require_throws<std::invalid_argument>([&] {
                (void)planning_to_body_frame(invalid_pose, point);
            }, "inverse rejects nonfinite origin coordinate");

            auto invalid_point = point;
            invalid_point.*coordinate = bad;
            require_throws<std::invalid_argument>([&] {
                (void)body_to_planning_frame(valid, invalid_point);
            }, "forward rejects nonfinite point coordinate");
            require_throws<std::invalid_argument>([&] {
                (void)planning_to_body_frame(valid, invalid_point);
            }, "inverse rejects nonfinite point coordinate");
        }
        auto invalid_pose = valid;
        invalid_pose.yaw_rad = bad;
        require_throws<std::invalid_argument>([&] {
            (void)body_to_planning_frame(invalid_pose, point);
        }, "forward rejects nonfinite yaw");
        require_throws<std::invalid_argument>([&] {
            (void)planning_to_body_frame(invalid_pose, point);
        }, "inverse rejects nonfinite yaw");
    }
}

void test_finite_extremes_and_overflow() {
    const double maximum = std::numeric_limits<double>::max();
    const std::array<double vector3::*, 3> coordinates{
        &vector3::x, &vector3::y, &vector3::z};
    for (const auto coordinate : coordinates) {
        for (const double sign : {-1.0, 1.0}) {
            vector3 origin{0.0, 0.0, 0.0};
            vector3 point{0.0, 0.0, 0.0};
            origin.*coordinate = sign * maximum;
            point.*coordinate = sign * maximum;
            const planar_body_pose forward_pose{origin, 0.0};
            require_throws<std::overflow_error>([&] {
                (void)body_to_planning_frame(forward_pose, point);
            }, "translation addition overflow");
            origin.*coordinate = -sign * maximum;
            const planar_body_pose inverse_pose{origin, 0.0};
            require_throws<std::overflow_error>([&] {
                (void)planning_to_body_frame(inverse_pose, point);
            }, "relative coordinate subtraction overflow");
        }
    }
    const planar_body_pose diagonal{{0.0, 0.0, 0.0}, pi / 4.0};
    require_throws<std::overflow_error>([&] {
        (void)body_to_planning_frame(diagonal, {maximum, -maximum, 0.0});
    }, "rotation output overflow forward");
    require_throws<std::overflow_error>([&] {
        (void)planning_to_body_frame(diagonal, {maximum, maximum, 0.0});
    }, "rotation output overflow inverse");

    const planar_body_pose identity{{0.0, 0.0, 0.0}, 0.0};
    const double tiny = std::numeric_limits<double>::denorm_min();
    for (const auto& point : {vector3{1e300, -1e300, 1e300}, vector3{tiny, -tiny, tiny}}) {
        const auto forward = body_to_planning_frame(identity, point);
        const auto inverse = planning_to_body_frame(identity, point);
        require(forward.x == point.x && forward.y == point.y && forward.z == point.z
            && inverse.x == point.x && inverse.y == point.y && inverse.z == point.z,
            "representable extreme identity is preserved");
    }
    for (const auto& result : {
             body_to_planning_frame({{0.0, 0.0, 0.0}, maximum}, {1.0, 2.0, 3.0}),
             planning_to_body_frame({{0.0, 0.0, 0.0}, -maximum}, {1.0, 2.0, 3.0})}) {
        require(std::isfinite(result.x) && std::isfinite(result.y) && std::isfinite(result.z),
            "finite large yaw produces finite coordinates");
    }
}

}  // namespace

int main() {
    try {
        test_independent_geometry();
        test_round_trips_and_rigidity();
        test_fixed_support_point();
        test_invalid_input();
        test_finite_extremes_and_overflow();
        std::cout << "PASS: independent translations and rotations, round trips, "
            "rigidity, fixed support point, invalid input and numeric overflow.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
