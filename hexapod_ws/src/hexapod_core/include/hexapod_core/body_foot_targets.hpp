#pragma once

#include "hexapod_core/body_pose.hpp"
#include "hexapod_core/foot_trajectory.hpp"
#include "hexapod_core/tripod_phase.hpp"

#include <array>
#include <optional>

namespace hexapod_core {

// 一次六腿脚点采样所需的完整输入。
// 所有腿数组均按 leg_order 排列。
struct body_foot_target_request {
    
    // 当前采样时刻的规划机身位姿。
    planar_body_pose body_pose;

    // 当前六腿计划阶段和阶段内部进度。
    std::array<leg_phase_state, leg_count> phases;

    // 六腿已保存的支撑脚点，均在规划参考系中表达。
    // 全部坐标必须有限；支撑期直接使用对应点。
    std::array<vector3, leg_count> stance_points_in_planning_frame;

    // 每条腿当前已保存的摆动曲线。
    //
    // 曲线中的起终点、抬升方向都在规划参考系中表达。
    // 摆动期必须有值；支撑期不使用该可选请求。
    std::array<std::optional<swing_trajectory_request>, leg_count > swings;

    body_foot_target_request(
        const planar_body_pose& pose,
        const std::array<leg_phase_state, leg_count>& phase_states,
        const std::array<vector3, leg_count>& stance_points,
        const std::array<std::optional<swing_trajectory_request>, leg_count>& swing_requests
    )
        : body_pose(pose),
          phases(phase_states),
          stance_points_in_planning_frame(stance_points),
          swings(swing_requests) {
    }
};

// 同一组理想脚点在两种坐标系中的表达。
struct body_foot_targets {
    std::array<vector3, leg_count> points_in_planning_frame;
    std::array<vector3, leg_count> points_in_body;
};

// 根据各腿当前阶段采样完整六腿脚点。
//
// phase_ratio 必须有限且位于 [0,1]。
// 支撑期使用固定落脚点；摆动期调用已有摆动曲线。
//
// 不保存历史，不选择落脚点，不自动推进支撑/摆动状态。
// 不进行 IK、路径检查或命令发送。
// 全部六腿计算完成后才返回结果。
body_foot_targets sample_body_foot_targets(
    const body_foot_target_request& request
);

}  // namespace hexapod_core
