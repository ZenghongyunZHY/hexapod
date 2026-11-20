#include "hexapod_core/swing_landing.hpp"

#include <cmath>
#include <stdexcept>

namespace hexapod_core {

namespace {

void require_landing_request(const swing_landing_request& request) {
    const auto& reference = request.midpoint_reference_in_body;

    if (!std::isfinite(reference.x_m) || !std::isfinite(reference.y_m)) {
        throw std::invalid_argument("Stance midpoint reference must be finite");
    }

    if (!std::isfinite(request.swing_duration_s) || request.swing_duration_s <= 0.0) {
        throw std::invalid_argument("Swing duration must be finite and positive");
    }

    if (!std::isfinite(request.next_stance_duration_s) || request.next_stance_duration_s <= 0.0) {
        throw std::invalid_argument("Next stance duration must be finite and positive");
    }

    if (!std::isfinite(request.ground_height_m)) {
        throw std::invalid_argument("Planning ground height must be finite");
    }
}

}  // namespace

vector3 plan_swing_landing_point(const swing_landing_request& request) {
    require_landing_request(request);

    // 从起脚时刻，预测到本次摆动结束后的下一支撑期中点。
    const double prediction_duration_s = request.swing_duration_s + 0.5 * request.next_stance_duration_s;

    if (!std::isfinite(prediction_duration_s)) {
        throw std::overflow_error("Landing prediction duration overflowed");
    }

    // 位姿与速度的有限性由已有积分函数检查。
    const auto midpoint_pose = advance_body_pose(request.lift_off_pose, request.velocity, prediction_duration_s);

    const auto& reference = request.midpoint_reference_in_body;

    // 当前模型只有偏航，水平坐标转换不依赖 Z。
    // 此处 z=0 用于表达水平参考位置；
    // 最终落脚高度由 ground_height_m 明确给出。
    const vector3 horizontal_reference{reference.x_m, reference.y_m,  0.0};

    // 在预测的支撑中点机身位姿下，将参考脚位
    // 转换为规划参考系中的固定位置。
    const auto reference_in_planning = body_to_planning_frame(midpoint_pose, horizontal_reference);

    return vector3{reference_in_planning.x, reference_in_planning.y, request.ground_height_m};
}

}  // namespace hexapod_core
