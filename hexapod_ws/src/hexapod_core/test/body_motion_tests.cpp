#include "hexapod_core/body_motion.hpp"

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
    require(std::isfinite(actual)
        && std::abs(actual - expected) <= 1e-12 * std::max(1.0, std::abs(expected)),
        message);
}

void pose_near(const planar_body_pose& actual, const planar_body_pose& expected,
    const std::string& message) {
    near(actual.body_origin_in_planning_frame.x,
        expected.body_origin_in_planning_frame.x, message + " x");
    near(actual.body_origin_in_planning_frame.y,
        expected.body_origin_in_planning_frame.y, message + " y");
    near(actual.body_origin_in_planning_frame.z,
        expected.body_origin_in_planning_frame.z, message + " z");
    near(actual.yaw_rad, expected.yaw_rad, message + " yaw");
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

void test_known_motions() {
    const planar_body_pose start{{0.1, -0.2, 0.3}, 0.0};
    pose_near(advance_body_pose(start, {0.02, 0.0, 0.0}, 2.0),
        {{0.14, -0.2, 0.3}, 0.0}, "independent forward translation");
    pose_near(advance_body_pose(start, {-0.02, 0.01, 0.0}, 2.0),
        {{0.06, -0.18, 0.3}, 0.0}, "backward and sideways translation");
    pose_near(advance_body_pose({{0.1, -0.2, 0.3}, pi / 2.0},
        {0.02, 0.01, 0.0}, 2.0),
        {{0.08, -0.16, 0.3}, pi / 2.0}, "body velocities use rotated heading");
    pose_near(advance_body_pose(start, {0.0, 0.0, -0.5}, 3.0),
        {{0.1, -0.2, 0.3}, -1.5}, "pure clockwise rotation");

    // 四分之一圆弧：半径 v/w，起点切线沿 +X。
    const planar_body_pose origin{{0.0, 0.0, 0.1}, 0.0};
    const double radius = 0.04 / pi;
    pose_near(advance_body_pose(origin, {0.02, 0.0, pi / 2.0}, 1.0),
        {{radius, radius, 0.1}, pi / 2.0}, "counterclockwise quarter circle");
    pose_near(advance_body_pose(origin, {0.02, 0.0, -pi / 2.0}, 1.0),
        {{radius, -radius, 0.1}, -pi / 2.0}, "clockwise quarter circle");
    pose_near(advance_body_pose(origin, {0.0, 0.02, pi / 2.0}, 1.0),
        {{-radius, radius, 0.1}, pi / 2.0}, "lateral velocity quarter circle");
    pose_near(advance_body_pose(origin, {0.02, 0.01, pi / 2.0}, 1.0),
        {{0.02 / pi, 0.06 / pi, 0.1}, pi / 2.0}, "combined body velocity circle");
    pose_near(advance_body_pose(origin, {0.02, -0.01, pi / 2.0}, 4.0),
        {{0.0, 0.0, 0.1}, 2.0 * pi}, "complete circle keeps unwrapped heading");
    pose_near(advance_body_pose({{0.0, 0.0, 0.1}, 4.0 * pi},
        {0.0, 0.0, pi / 2.0}, 1.0),
        {{0.0, 0.0, 0.1}, 4.5 * pi}, "accumulated heading is not normalized");
    pose_near(advance_body_pose(start, {0.1, -0.2, 0.3}, 0.0), start,
        "zero duration returns initial pose");
    pose_near(advance_body_pose(start, {0.0, 0.0, 0.0}, 100.0), start,
        "stationary command");
}

planar_body_pose numerical_reference(const planar_body_pose& start,
    const planar_body_velocity& velocity, double duration) {
    // 独立参考：对规划参考系中的瞬时速度做复合 Simpson 积分，
    // 不复用生产代码的 sinc/cosc 圆弧公式。
    constexpr int intervals = 4096;
    const long double step = static_cast<long double>(duration) / intervals;
    long double sum_x = 0.0L;
    long double sum_y = 0.0L;
    for (int index = 0; index <= intervals; ++index) {
        const long double time = step * index;
        const long double yaw = static_cast<long double>(start.yaw_rad)
            + static_cast<long double>(velocity.wz_rad_s) * time;
        const int weight = (index == 0 || index == intervals) ? 1
            : ((index % 2 == 0) ? 2 : 4);
        sum_x += weight * (std::cos(yaw) * velocity.vx_m_s
            - std::sin(yaw) * velocity.vy_m_s);
        sum_y += weight * (std::sin(yaw) * velocity.vx_m_s
            + std::cos(yaw) * velocity.vy_m_s);
    }
    return {{start.body_origin_in_planning_frame.x + static_cast<double>(sum_x * step / 3),
                start.body_origin_in_planning_frame.y + static_cast<double>(sum_y * step / 3),
                start.body_origin_in_planning_frame.z},
        start.yaw_rad + velocity.wz_rad_s * duration};
}

void test_numerical_reference_and_small_angles() {
    const planar_body_pose start{{0.13, -0.27, 0.11}, 0.37};
    for (const double rate : {-2.0, -0.3, -1e-4, 0.0, 1e-4, 0.3, 2.0}) {
        for (const double duration : {0.01, 0.5, 2.0}) {
            const planar_body_velocity velocity{0.02, -0.013, rate};
            pose_near(advance_body_pose(start, velocity, duration),
                numerical_reference(start, velocity, duration), "independent numerical integral");
        }
    }
    for (const double angle : {-std::nextafter(1e-4, 1.0), -1e-4,
             -std::nextafter(1e-4, 0.0), std::nextafter(1e-4, 0.0),
             1e-4, std::nextafter(1e-4, 1.0)}) {
        const planar_body_velocity velocity{0.02, -0.013, angle};
        pose_near(advance_body_pose(start, velocity, 1.0),
            numerical_reference(start, velocity, 1.0), "both sides of numeric branch threshold");
    }
    const planar_body_pose origin{{0.0, 0.0, 0.1}, 0.0};
    for (const double rate : {1e-12, -1e-12, 1e-100, -1e-100, 1e-200, -1e-200}) {
        const auto result = advance_body_pose(origin, {0.1, 0.0, rate}, 1.0);
        const double expected_y = 0.05 * rate;
        // 相对误差断言，防止绝对容差让错误的零侧向位移也通过。
        require(result.body_origin_in_planning_frame.y != 0.0
            && std::abs(result.body_origin_in_planning_frame.y / expected_y - 1.0) < 1e-12,
            "tiny angular velocity retains signed lateral displacement");
        near(result.body_origin_in_planning_frame.x, 0.1, "tiny-rate forward displacement");
    }
}

void test_composition_and_inputs() {
    const planar_body_pose start{{0.13, -0.27, 0.11}, -0.37};
    const planar_body_velocity velocity{0.02, -0.013, 0.7};
    const auto whole = advance_body_pose(start, velocity, 2.0);
    for (const int count : {2, 7, 100}) {
        auto divided = start;
        for (int segment = 0; segment < count; ++segment) {
            divided = advance_body_pose(divided, velocity, 2.0 / count);
        }
        pose_near(divided, whole, "same constant body velocity composes across segments");
    }
    pose_near(advance_body_pose(whole, {-0.02, 0.013, -0.7}, 2.0), start,
        "opposite constant twist reverses motion");
    const auto turned = advance_body_pose(
        {{0.0, 0.0, 0.11}, pi / 2.0}, velocity, 2.0);
    const auto unturned = advance_body_pose(
        {{0.0, 0.0, 0.11}, 0.0}, velocity, 2.0);
    near(turned.body_origin_in_planning_frame.x,
        -unturned.body_origin_in_planning_frame.y, "quarter-turned initial frame x");
    near(turned.body_origin_in_planning_frame.y,
        unturned.body_origin_in_planning_frame.x, "quarter-turned initial frame y");
    require(start.body_origin_in_planning_frame.x == 0.13
        && start.body_origin_in_planning_frame.y == -0.27
        && start.body_origin_in_planning_frame.z == 0.11 && start.yaw_rad == -0.37
        && velocity.vx_m_s == 0.02 && velocity.vy_m_s == -0.013
        && velocity.wz_rad_s == 0.7, "pose and velocity inputs remain unchanged");
}

void test_invalid_input() {
    const planar_body_pose valid{{0.1, -0.2, 0.3}, 0.4};
    const planar_body_velocity velocity{0.02, -0.01, 0.3};
    const std::array<double vector3::*, 3> coordinates{
        &vector3::x, &vector3::y, &vector3::z};
    const std::array<double planar_body_velocity::*, 3> components{
        &planar_body_velocity::vx_m_s, &planar_body_velocity::vy_m_s,
        &planar_body_velocity::wz_rad_s};
    for (const double bad : {std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::infinity(),
             -std::numeric_limits<double>::infinity()}) {
        // 零时长仍应校验整份输入。
        for (const double duration : {0.0, 1.0}) {
            for (const auto coordinate : coordinates) {
                auto invalid = valid;
                invalid.body_origin_in_planning_frame.*coordinate = bad;
                require_throws<std::invalid_argument>([&] {
                    (void)advance_body_pose(invalid, velocity, duration);
                }, "nonfinite origin coordinate");
            }
            auto invalid = valid;
            invalid.yaw_rad = bad;
            require_throws<std::invalid_argument>([&] {
                (void)advance_body_pose(invalid, velocity, duration);
            }, "nonfinite initial yaw");
            for (const auto component : components) {
                auto invalid_velocity = velocity;
                invalid_velocity.*component = bad;
                require_throws<std::invalid_argument>([&] {
                    (void)advance_body_pose(valid, invalid_velocity, duration);
                }, "nonfinite velocity component");
            }
        }
        require_throws<std::invalid_argument>([&] {
            (void)advance_body_pose(valid, velocity, bad);
        }, "nonfinite duration");
    }
    for (const double duration : {-1.0, std::nextafter(0.0, -1.0)}) {
        require_throws<std::invalid_argument>([&] {
            (void)advance_body_pose(valid, velocity, duration);
        }, "negative duration");
    }
}

void test_overflow_and_finite_extremes() {
    const double maximum = std::numeric_limits<double>::max();
    const planar_body_pose origin{{0.0, 0.0, 0.1}, 0.0};
    for (const auto& velocity : {
             planar_body_velocity{maximum, 0.0, 0.0},
             planar_body_velocity{0.0, maximum, 0.0},
             planar_body_velocity{0.0, 0.0, maximum}}) {
        require_throws<std::overflow_error>([&] {
            (void)advance_body_pose(origin, velocity, 2.0);
        }, "velocity-time product overflow");
    }
    require_throws<std::overflow_error>([&] {
        (void)advance_body_pose({{0.0, 0.0, 0.1}, maximum}, {0.0, 0.0, maximum}, 1.0);
    }, "accumulated yaw overflow");
    require_throws<std::overflow_error>([&] {
        (void)advance_body_pose(origin, {maximum, -maximum, pi / 2.0}, 1.0);
    }, "local arc displacement overflow");
    require_throws<std::overflow_error>([&] {
        (void)advance_body_pose({{maximum, 0.0, 0.1}, 0.0}, {maximum, 0.0, 0.0}, 1.0);
    }, "planning origin addition overflow");
    require_throws<std::overflow_error>([&] {
        (void)advance_body_pose({{0.0, 0.0, 0.1}, pi / 4.0},
            {maximum, -maximum, 0.0}, 1.0);
    }, "initial heading rotation overflow");

    const double tiny = std::numeric_limits<double>::denorm_min();
    const auto tiny_motion = advance_body_pose(origin, {1.0, 0.0, 0.0}, tiny);
    require(tiny_motion.body_origin_in_planning_frame.x == tiny,
        "representable subnormal displacement is preserved");
    pose_near(advance_body_pose(origin, {maximum, maximum, maximum}, 0.0), origin,
        "finite extreme velocities with zero duration");
    const auto huge_yaw = advance_body_pose(origin, {0.01, -0.02, maximum}, 1.0);
    require(std::isfinite(huge_yaw.body_origin_in_planning_frame.x)
        && std::isfinite(huge_yaw.body_origin_in_planning_frame.y)
        && huge_yaw.body_origin_in_planning_frame.z == 0.1
        && huge_yaw.yaw_rad == maximum, "finite extreme turn remains finite");
}

}  // namespace

int main() {
    try {
        test_known_motions();
        test_numerical_reference_and_small_angles();
        test_composition_and_inputs();
        test_invalid_input();
        test_overflow_and_finite_extremes();
        std::cout << "PASS: translations, circular arcs, independent Simpson integral, "
            "tiny turns, segment composition, input validation and overflow.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
