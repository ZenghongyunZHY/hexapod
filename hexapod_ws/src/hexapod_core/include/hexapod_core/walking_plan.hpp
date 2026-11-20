#pragma once

#include "hexapod_core/tripod_sequence_timeline.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

namespace hexapod_core {

struct stance_recovery_parameters {
    tripod_timing_request timing;
    std::size_t intervals_per_stage;
    double lift_height_m;
    // 机身系站姿脚点 XY，按 leg_order 排列，单位 m。
    std::array<stance_midpoint_reference, leg_count> standing_feet_in_body;
    // 理想支撑锚点和名义 FK 脚点各自允许的三维误差，单位 m。
    double foot_tolerance_m;
};

enum class walking_plan_status {
    checks_passed,
    walking_cycle_failed,
    recovery_cycle_failed,
    recovery_terminal_mismatch
};

struct walking_plan_solution {
    // 行走周期从 0 开始编号；原地收步若有，编号为 full_cycle_count。
    std::vector<tripod_timed_segment> timeline;
    tripod_sequence_initial_state final_state;
    // 每个已检查行走周期的候选末状态，按周期顺序排列。
    std::vector<tripod_sequence_initial_state> cycle_end_states;
    std::size_t full_cycle_count;
    bool recovery_added;
};

struct walking_plan_result {
    walking_plan_status status;
    std::optional<tripod_sequence_result> walking_failure;
    std::optional<tripod_cycle_result> recovery_failure;
    std::optional<leg_id> terminal_failed_leg;
    std::optional<walking_plan_solution> solution;
};

class walking_planner {
 public:
    explicit walking_planner(robot_model model);

    // 从已知的完整周期交界状态出发，规划给定行走周期及可选原地收步。
    // 空行走请求只有在指定收步时有效。如果已经满足站姿，可成功返回空时间轴。
    // 各周期速度由调用方安排。收步固定机身位姿，不计入行走周期数。
    // 任意阶段失败只返回诊断。成功结果中的末状态仍是规划值。
    walking_plan_result plan(const tripod_sequence_schedule_request& request, const std::optional<stance_recovery_parameters>& recovery = std::nullopt) const;

 private:
    tripod_sequence_planner sequence_planner_;
    tripod_cycle_planner cycle_planner_;
    leg_kinematics kinematics_;
};

}  // namespace hexapod_core
