#pragma once

#include "hexapod_core/foot_trajectory.hpp"

#include <array>
#include <optional>

namespace hexapod_core {

// 保存六腿规划脚点和摆动曲线。
// 所有位置与方向都在同一个规划参考系中表达。
class gait_foot_state {
 public:
    // 显式提供六腿初始规划支撑点。
    // 全部坐标必须有限，数组按 leg_order 排列。
    // 初始化时，各腿均没有进行中的摆动曲线。
    explicit gait_foot_state(const std::array<vector3, leg_count>& initial_stance_points);

    // 读取已保存的规划支撑点。
    const std::array<vector3, leg_count>& stance_points() const noexcept;

    // 读取已保存的摆动曲线。
    const std::array<std::optional<swing_trajectory_request>, leg_count>& swings() const noexcept;

    // 从当前保存的支撑点开始一次计划摆动。
    //
    // 该腿已有摆动曲线时抛 std::logic_error。
    // 起点取保存的支撑点，其余几何输入由调用方提供。
    //
    // 返回更新后的副本，原对象保持不变。
    [[nodiscard]] gait_foot_state begin_swing(leg_id leg, const vector3& landing_point_in_planning_frame, const vector3& lift_direction_in_planning_frame, double lift_height_m) const;

    // 将摆动终点保存为新的规划支撑点，并清除摆动曲线。
    //
    // 调用方应在计划摆动终点已采样并被接受后调用。
    // 这表示计划阶段结束，不表示观测到真实触地。
    // 该腿没有摆动曲线时抛 std::logic_error。
    //
    // 返回更新后的副本，原对象保持不变。
    [[nodiscard]] gait_foot_state finish_swing(leg_id leg) const;

 private:
    std::array<vector3, leg_count> stance_points_;

    std::array<std::optional<swing_trajectory_request>, leg_count> swings_;
};

}  // namespace hexapod_core
