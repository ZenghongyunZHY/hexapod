#pragma once

#include "hexapod_core/tripod_cycle_planner.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

namespace hexapod_core {

struct tripod_sequence_request {
    tripod_cycle_request first_cycle;
    std::size_t full_cycle_count;

    // 第二周期起，理想脚点与名义 FK 脚点的最大三维距离。
    // 单位 m，由调用者显式指定。
    double continuation_foot_tolerance_m;
};

// 首周期的计划状态。初始脚点须处于支撑阶段，数组按 leg_order 排列。
struct tripod_sequence_initial_state {
    planar_body_pose body_pose;
    gait_foot_state feet;
    std::array<leg_joint_angles, leg_count> reference_angles;
    std::array<leg_branch_state, leg_count> remembered_branches;
};

// 这些参数在本周期内固定，可以在下一个完整周期改变。
struct tripod_sequence_cycle_parameters {
    tripod_timing_request timing;
    std::size_t intervals_per_stage;
    planar_body_velocity velocity;
    double lift_height_m;
};

struct tripod_sequence_schedule_request {
    tripod_sequence_initial_state initial_state;
    // 至少一个周期，按规划顺序排列。
    std::vector<tripod_sequence_cycle_parameters> cycles;

    // 整个序列共用的规划参考和检查设置。
    std::array<stance_midpoint_reference, leg_count> midpoint_references_in_body;
    double ground_height_m;
    leg_selection_options selection_options;
    std::array<std::array<double, joints_per_leg>, leg_count> max_joint_speed_rad_s;
    std::size_t path_interval_count;

    // 首周期和后续周期的脚点三维一致性容差，单位 m。
    double initial_foot_tolerance_m;
    double continuation_foot_tolerance_m;
    std::optional<foot_clearance_policy> clearance_policy{};
    std::optional<tripod_walking_profile> walking_profile{};
};

enum class tripod_sequence_status {
    checks_passed,
    cycle_failed
};

struct tripod_sequence_solution {
    // 每个周期保留自己的帧和关节段，帧时间从本周期的零时刻开始。
    std::vector<tripod_cycle_solution> cycles;
};

struct tripod_sequence_result {
    tripod_sequence_status status;

    // 下标从 0 开始。周期内部的帧和采样诊断保存在 cycle_failure。
    std::optional<std::size_t> failed_cycle_index;
    std::optional<tripod_cycle_result> cycle_failure;

    // 全部周期通过后才有值。
    std::optional<tripod_sequence_solution> solution;
};

class tripod_sequence_planner {
 public:
    explicit tripod_sequence_planner(robot_model model);

    // 用同一组参数规划指定数量的完整周期。
    tripod_sequence_result plan(const tripod_sequence_request& request) const;

    // 每周期使用自己的参数，接续上一周期通过检查后的状态。
    // 规划失败保留原单周期诊断。无效输入沿用单周期的异常语义。
    tripod_sequence_result plan(const tripod_sequence_schedule_request& request) const;

 private:
    tripod_cycle_planner cycle_planner_;
};

}  // namespace hexapod_core
