#pragma once

#include "hexapod_core/body_segment_planner.hpp"
#include "hexapod_core/foot_clearance_policy.hpp"
#include "hexapod_core/tripod_foot_cycle.hpp"

#include <cstddef>

namespace hexapod_core {

// 根据完整周期中相邻的两个几何帧，
// 生成六腿运动段内部的逐采样点约束。
//
// previous/current 必须来自同一个几何周期，且在序列中相邻。
// 不能跳过中间帧或阶段边界。
//
// 当前仅适用于：
// 固定机身高度、纯偏航姿态、水平规划地面。
//
// interval_count 个区间对应每腿 interval_count + 1 份约束。
//
// 摆动内部重新采样原曲线，不线性插值两端脚点高度。
// 支撑段使用固定规划脚点的实际理想高度。
//
// 不修改输入，不进行 IK，不执行动作。
body_path_constraint_samples make_tripod_segment_constraints(
    const tripod_foot_frame& previous,
    const tripod_foot_frame& current,
    double ground_height_m,
    std::size_t interval_count,
    const foot_clearance_policy& policy
);

}  // namespace hexapod_core
