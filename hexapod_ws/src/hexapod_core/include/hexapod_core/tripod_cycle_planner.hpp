#pragma once

#include "hexapod_core/body_segment_planner.hpp"
#include "hexapod_core/tripod_foot_cycle.hpp"
#include "hexapod_core/foot_clearance_policy.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

namespace hexapod_core {

// 完整周期的关节规划请求。
struct tripod_cycle_request {
    tripod_foot_cycle_request foot_request;

    // 按 leg_order 排列，单位 rad。
    std::array<leg_joint_angles, leg_count> reference_angles;
    std::array<leg_branch_state, leg_count> remembered_branches;
    leg_selection_options selection_options;

    // 按 leg_order、joint_order 排列，单位 rad/s。
    std::array<std::array<double, joints_per_leg>, leg_count> max_joint_speed_rad_s;

    // 每个运动段内部的路径检查区间数。
    std::size_t path_interval_count;

    // 首帧理想脚点与输入参考角 FK 脚点的最大位置差，单位 m。
    double initial_foot_tolerance_m;

    // 无值时，整段使用固定离面下界 0。
    // 有值时，按计划相位和理想高度生成逐点约束。
    std::optional<foot_clearance_policy> clearance_policy;

    tripod_cycle_request(
        const tripod_foot_cycle_request& feet,
        const std::array<leg_joint_angles, leg_count>& references,
        const std::array<leg_branch_state, leg_count>& branches,
        const leg_selection_options& options,
        const std::array<std::array<double, joints_per_leg>, leg_count>& speed_limits,
        std::size_t path_intervals,
        double initial_tolerance,
        std::optional<foot_clearance_policy> policy = std::nullopt
    )
        : foot_request(feet),
          reference_angles(references),
          remembered_branches(branches),
          selection_options(options),
          max_joint_speed_rad_s(speed_limits),
          path_interval_count(path_intervals),
          initial_foot_tolerance_m(initial_tolerance),
          clearance_policy(std::move(policy)) {
    }
};

// 只有整周期通过后才返回的完整候选结果。
struct tripod_cycle_solution {
    tripod_foot_cycle foot_cycle;

    // segments[i] 的终点对应 foot_cycle.frames[i+1]。
    // 初始 frames[0] 是输入参考，不单独生成运动段。
    std::vector<body_ik_solution> segments;

    tripod_cycle_solution(tripod_foot_cycle geometry, std::vector<body_ik_solution> joint_segments) : foot_cycle(std::move(geometry)), segments(std::move(joint_segments)) {}
};

enum class tripod_cycle_status {
    checks_passed,
    initial_reference_mismatch,
    segment_failed
};

struct tripod_cycle_result {
    tripod_cycle_status status;

    // 初始参考不一致时有值。
    std::optional<leg_id> initial_failed_leg;
    std::optional<double> initial_foot_error_m;

    // 运动段失败时有值。
    // 指向该段终点在 foot_cycle.frames 中的下标，从 1 开始。
    std::optional<std::size_t> failed_frame_index;

    // 复用已有段规划结果，保留选择或路径失败诊断。
    std::optional<body_segment_result> segment_failure;

    // 仅 checks_passed 时有值。
    std::optional<tripod_cycle_solution> solution;

    explicit tripod_cycle_result(tripod_cycle_status checked_status)
        : status(checked_status) {
    }
};

class tripod_cycle_planner {
 public:
    explicit tripod_cycle_planner(robot_model model);

    // 规划并检查完整周期。
    //
    // 初始脚点与输入关节参考不一致时拒绝。
    // 任意运动段失败，不返回部分周期 solution。
    //
    // 只在局部推进名义关节角与分支。
    // 不修改调用方的关节参考、脚点状态或机身位姿。
    //
    // 当前仅针对固定机身高度、纯偏航和水平地面。
    // 使用已有线性关节段离散检查，不证明真实触地或无滑移。
    tripod_cycle_result plan(const tripod_cycle_request& request) const;

 private:
    leg_kinematics kinematics_;
    body_segment_planner segment_planner_;
};

}  // namespace hexapod_core
