#include "hexapod_core/gait_foot_state.hpp"

#include <cmath>
#include <stdexcept>

namespace hexapod_core {

namespace {

void require_finite_stance_point(const vector3& point) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        throw std::invalid_argument("Initial stance point must be finite");
    }
}

}  // namespace

gait_foot_state::gait_foot_state(const std::array<vector3, leg_count>& initial_stance_points) : stance_points_(initial_stance_points), swings_{} {
    for (const auto& point : stance_points_) {
        require_finite_stance_point(point);
    }
}

const std::array<vector3, leg_count>& gait_foot_state::stance_points() const noexcept {
    return stance_points_;
}

const std::array<std::optional<swing_trajectory_request>, leg_count>&  gait_foot_state::swings() const noexcept {
    return swings_;
}

gait_foot_state gait_foot_state::begin_swing(leg_id leg, const vector3& landing_point_in_planning_frame, const vector3& lift_direction_in_planning_frame, double lift_height_m) const {
    const auto index = leg_index(leg);

    if (swings_[index]) {
        throw std::logic_error("Cannot begin a swing while a swing is already stored");
    }

    // 起点使用已保存的规划支撑点。
    // 同一次摆动期间，这份曲线的起终点保持固定。
    const swing_trajectory_request request{
        stance_points_[index],
        landing_point_in_planning_frame,
        lift_direction_in_planning_frame,
        lift_height_m
    };

    // 已有采样函数会先校验整份几何输入，再返回端点。
    // 这里复用其输入校验；后续仍需采样及路径检查。
    (void)sample_swing_trajectory(request, 0.0);

    auto next = *this;

    next.swings_[index] = request;

    return next;
}

gait_foot_state gait_foot_state::finish_swing(leg_id leg) const {
    const auto index = leg_index(leg);

    if (!swings_[index]) {
        throw std::logic_error("Cannot finish a swing without a stored trajectory");
    }

    const auto& completed_swing = *swings_[index];

    auto next = *this;

    // 计划摆动终点成为下一段支撑期的固定锚点。
    next.stance_points_[index] = completed_swing.end_in_planning_frame;

    next.swings_[index].reset();

    return next;
}

}  // namespace hexapod_core
