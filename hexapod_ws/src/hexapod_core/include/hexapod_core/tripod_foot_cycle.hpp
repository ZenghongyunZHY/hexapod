#pragma once

#include "hexapod_core/body_foot_targets.hpp"
#include "hexapod_core/gait_foot_state.hpp"
#include "hexapod_core/swing_landing.hpp"
#include "hexapod_core/tripod_sampling.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

namespace hexapod_core {

struct tripod_walking_profile {
    std::size_t double_support_intervals;
    std::size_t swing_intervals;
    // 摆动中点相对原曲线的水平外移，单位 m。
    double outward_offset_m;
};

// 一个完整周期的脚点几何规划输入。
struct tripod_foot_cycle_request {
    tripod_timing_request timing;
    std::size_t intervals_per_stage;

    planar_body_pose initial_pose;
    planar_body_velocity velocity;

    // 周期开始前的规划脚点状态，必须没有进行中的摆动。
    gait_foot_state initial_feet;

    // 六腿各自的下一支撑期中点水平参考。
    std::array<stance_midpoint_reference, leg_count> midpoint_references_in_body;

    // 规划参考系中的水平地面高度。
    double ground_height_m;

    // 本周期统一使用的额外竖直抬升量，必须有限且不小于 0。
    double lift_height_m;

    // 省略时使用均匀分段与竖直抬升。
    std::optional<tripod_walking_profile> walking_profile;

    tripod_foot_cycle_request(
        const tripod_timing_request& timing_request,
        std::size_t intervals,
        const planar_body_pose& pose,
        const planar_body_velocity& body_velocity,
        const gait_foot_state& feet,
        const std::array<stance_midpoint_reference, leg_count>& references,
        double ground_height,
        double lift_height,
        std::optional<tripod_walking_profile> profile = std::nullopt
    )
        : timing(timing_request),
          intervals_per_stage(intervals),
          initial_pose(pose),
          velocity(body_velocity),
          initial_feet(feet),
          midpoint_references_in_body(references),
          ground_height_m(ground_height),
          lift_height_m(lift_height),
          walking_profile(profile) {
    }
};

// 一个采样时刻的完整几何结果。
// 一个采样时刻的完整几何结果。
struct tripod_foot_frame {
    tripod_cycle_sample sample;
    planar_body_pose body_pose;

    // 当前时刻的阶段。
    // 落脚边界仍保留旧摆动的进度 1。
    std::array<leg_phase_state, leg_count> phases;

    body_foot_targets targets;

    // 当前边界事件全部处理完成后，
    // 离开本帧时仍有效的摆动曲线。
    //
    // 有值：下一段继续沿该曲线摆动。
    // 无值：下一段处于计划支撑。
    std::array< std::optional<swing_trajectory_request>, leg_count> outgoing_swings;

    tripod_foot_frame(
        const tripod_cycle_sample& cycle_sample,
        const planar_body_pose& pose,
        const std::array<leg_phase_state, leg_count>& phase_states,
        const body_foot_targets& foot_targets,
        const std::array<std::optional<swing_trajectory_request>, leg_count>& outgoing_swing_requests
    )
        : sample(cycle_sample),
          body_pose(pose),
          phases(phase_states),
          targets(foot_targets),
          outgoing_swings(outgoing_swing_requests) {
    }
};

struct tripod_foot_cycle {
    // 包含时间 0 和周期终点。
    std::vector<tripod_foot_frame> frames;

    // 本周期结束后的候选规划状态。
    gait_foot_state final_feet;
    planar_body_pose final_pose;

    tripod_foot_cycle(
        std::vector<tripod_foot_frame> cycle_frames,
        const gait_foot_state& feet,
        const planar_body_pose& pose
    )
        : frames(std::move(cycle_frames)),
          final_feet(feet),
          final_pose(pose) {
    }
};

// 生成一个完整三脚周期的理想脚点序列。
// 不修改输入，不进行 IK、路径检查或执行。
// final_feet/final_pose 也是候选状态，
// 上层接受整条关节规划后才能保存。
tripod_foot_cycle plan_tripod_foot_cycle(const tripod_foot_cycle_request& request);

// 水平外移需要六条腿偏航根部在机身坐标系中的位置，按 leg_order 排列。
tripod_foot_cycle plan_tripod_foot_cycle(
    const tripod_foot_cycle_request& request,
    const std::array<vector3, leg_count>& yaw_roots_in_body);

}  // namespace hexapod_core
