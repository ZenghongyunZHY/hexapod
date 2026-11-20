#pragma once

#include "hexapod_core/tripod_phase.hpp"

#include <optional>

namespace hexapod_core {

// 离线名义几何检查使用的显式误差预算。
// 单位均为 m，不表示实测舵机精度。
struct foot_clearance_policy {
    // 接触阶段允许低于参考平面的最大距离。
    double contact_below_tolerance_m;

    // 接触阶段允许高于参考平面的最大距离。
    double contact_above_tolerance_m;

    // 摆动内部允许低于理想高度的最大距离。
    double swing_height_error_m;

    foot_clearance_policy(
        double contact_below_tolerance,
        double contact_above_tolerance,
        double swing_height_error
    )
        : contact_below_tolerance_m(contact_below_tolerance),
          contact_above_tolerance_m(contact_above_tolerance),
          swing_height_error_m(swing_height_error) {
    }
};

// 某个采样时刻的有符号离面距离范围。
struct foot_clearance_bounds {
    double minimum_clearance_m;

    std::optional<double> maximum_clearance_m;

    foot_clearance_bounds(double minimum_clearance, std::optional<double> maximum_clearance = std::nullopt)
        : minimum_clearance_m(minimum_clearance),
          maximum_clearance_m(maximum_clearance) {
    }
};

// 检查三个预算是否有限且非负。
// 非法输入抛出 std::invalid_argument。
void require_foot_clearance_policy(const foot_clearance_policy& policy);

// 根据计划阶段与理想高度，生成当前时刻的约束。
//
// phase_ratio 必须有限且位于 [0, 1]。
// ideal_clearance_m 必须有限且非负。
//
// 支撑期和摆动两端使用接触范围。
// 摆动内部使用随理想高度变化的下界，不设置上界。
//
// 返回的是单个时刻的范围，不是整个运动段的固定范围。
foot_clearance_bounds make_foot_clearance_bounds(const leg_phase_state& phase, double ideal_clearance_m, const foot_clearance_policy& policy);

}  // namespace hexapod_core
