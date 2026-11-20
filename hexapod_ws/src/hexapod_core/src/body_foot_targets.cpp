#include "hexapod_core/body_foot_targets.hpp"

#include <cmath>
#include <stdexcept>

namespace hexapod_core {

namespace {

void require_finite_vector(const vector3& value, const char* message) {
    if (!std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z)) {
        throw std::invalid_argument(message);
    }
}

void require_foot_target_request(const body_foot_target_request& request) {
    require_finite_vector(request.body_pose.body_origin_in_planning_frame, "Body origin must be finite");

    if (!std::isfinite(request.body_pose.yaw_rad)) {
        throw std::invalid_argument("Body yaw must be finite");
    }

    // 先检查六腿相位、支撑点和摆动请求是否齐备。
    for (const auto leg : leg_order) {
        const auto index = leg_index(leg);

        require_finite_vector(request.stance_points_in_planning_frame[index], "Stored stance point must be finite");

        const auto& phase = request.phases[index];

        if (!std::isfinite(phase.phase_ratio) || phase.phase_ratio < 0.0 || phase.phase_ratio > 1.0) {
            throw std::invalid_argument("Leg phase ratio must be finite and within [0, 1]");
        }

        switch (phase.phase) {
            case leg_phase::stance:
                break;

            case leg_phase::swing:
                if (!request.swings[index]) {
                    throw std::invalid_argument("Swing phase requires a stored swing trajectory");
                }
                break;

            default:
                throw std::invalid_argument("Invalid leg phase");
        }
    }
}

}  // namespace

body_foot_targets sample_body_foot_targets(const body_foot_target_request& request) {
    require_foot_target_request(request);

    // 结果只在函数内部暂存，六腿全部完成后再返回。
    body_foot_targets result{};

    for (const auto leg : leg_order) {

        const auto index = leg_index(leg);
        const auto& phase = request.phases[index];

        // 支撑期使用规划参考系中的固定落脚点。
        vector3 point_in_planning = request.stance_points_in_planning_frame[index];

        if (phase.phase == leg_phase::swing) {
            // 请求存在性已在前面检查。
            const auto& swing = *request.swings[index];

            // 复用已有曲线函数的几何计算及输入校验。
            point_in_planning = sample_swing_trajectory(swing, phase.phase_ratio);
        }

        result.points_in_planning_frame[index] = point_in_planning;

        // 支撑点和摆动点统一转换到当前机身坐标。
        result.points_in_body[index] = planning_to_body_frame(request.body_pose, point_in_planning);
    }

    return result;
}

}  // namespace hexapod_core
