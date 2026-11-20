#include "hexapod_core/body_motion.hpp"

#include <cmath>
#include <stdexcept>

namespace hexapod_core {

namespace {

void require_motion_inputs(const planar_body_pose& pose, const planar_body_velocity& velocity, double duration_s) {
    const auto& origin = pose.body_origin_in_planning_frame;

    if (!std::isfinite(origin.x) || !std::isfinite(origin.y) || !std::isfinite(origin.z) || !std::isfinite(pose.yaw_rad)) {
        throw std::invalid_argument( "Initial body pose must be finite");
    }

    if (!std::isfinite(velocity.vx_m_s) || !std::isfinite(velocity.vy_m_s) || !std::isfinite(velocity.wz_rad_s)) {
        throw std::invalid_argument("Body velocity must be finite");
    }

    if (!std::isfinite(duration_s) || duration_s < 0.0) {
        throw std::invalid_argument("Motion duration must be finite and nonnegative");
    }
}

void require_finite_result(double value, const char* message) {
    if (!std::isfinite(value)) {
        throw std::overflow_error(message);
    }
}

// 恒定机身速度积分使用的圆弧系数。
struct arc_factors {
    // sin(angle) / angle，angle=0 时取极限 1。
    double sinc;

    // (1-cos(angle)) / angle，angle=0 时取极限 0。
    double cosc;
};

//计算 arc_factors
arc_factors calculate_arc_factors(double angle) {
    // 这里只是数值计算分支的阈值，不是机械转向阈值。
    constexpr double small_angle_threshold = 1e-4;

    if (std::abs(angle) < small_angle_threshold) {
        const double squared = angle * angle;

        // sin(a)/a
        // = 1 - a²/6 + a⁴/120 - a⁶/5040 + ...
        const double sinc = 1.0 + squared * (-1.0 / 6.0 + squared * (1.0 / 120.0 - squared / 5040.0));

        // (1-cos(a))/a
        // = a/2 - a³/24 + a⁵/720 - a⁷/40320 + ...
        const double cosc = angle * (0.5 + squared * (-1.0 / 24.0 + squared * (1.0 / 720.0 - squared / 40320.0)));

        return arc_factors{sinc, cosc};
    }

    const double sinc = std::sin(angle) / angle;

    // 使用 1-cos(a) = 2*sin²(a/2)，
    // 避免直接计算两个接近数的差。
    const double half_sine = std::sin(0.5 * angle);

    const double cosc = (2.0 * half_sine) * (half_sine / angle);

    return arc_factors{sinc, cosc};
}

}  // namespace

//给定机身速度和时间，算出新的规划机身位姿
planar_body_pose advance_body_pose(const planar_body_pose& initial_pose, const planar_body_velocity& velocity, double duration_s) {

    require_motion_inputs(initial_pose, velocity, duration_s);

    // 即使时间为 0，也先拒绝非法位姿或速度。
    if (duration_s == 0.0) {
        return initial_pose;
    }

    const double delta_yaw = velocity.wz_rad_s * duration_s;

    require_finite_result(delta_yaw, "Yaw increment overflowed");

    const double next_yaw = initial_pose.yaw_rad + delta_yaw;

    require_finite_result( next_yaw, "Integrated body yaw overflowed");

    const double forward_distance = velocity.vx_m_s * duration_s;

    const double lateral_distance = velocity.vy_m_s * duration_s;

    require_finite_result(forward_distance, "Forward velocity-time product overflowed");

    require_finite_result(lateral_distance, "Lateral velocity-time product overflowed");

    const auto factors = calculate_arc_factors(delta_yaw);

    // 整段平移量，沿运动段起点的机身坐标轴表达。
    //
    // 这里已经计入运动过程中机身朝向的连续变化。
    const vector3 displacement_in_initial_body{

        factors.sinc * forward_distance - factors.cosc * lateral_distance,

        factors.cosc * forward_distance + factors.sinc * lateral_distance,

        0.0
    };

    require_finite_result(displacement_in_initial_body.x, "Integrated local X displacement overflowed");

    require_finite_result(displacement_in_initial_body.y, "Integrated local Y displacement overflowed");

    // 将相对起始机身原点的位移旋转到规划参考系，
    // 再加上起始机身原点，得到新的机身原点位置。
    //“displacement_in_initial_body” 已经计入了这段时间内 delta_yaw 对平移轨迹的影响。 它表示整段运动结束后的总位移，沿起点机身坐标轴表达。
    //所以选择先进行平移，再进行旋转
    const auto next_origin = body_to_planning_frame(initial_pose, displacement_in_initial_body);

    return planar_body_pose{next_origin, next_yaw};
}

}  // namespace hexapod_core
