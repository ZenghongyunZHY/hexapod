#pragma once

#include "hexapod_core/leg_candidate_selector.hpp"

#include <array>
#include <optional>

namespace hexapod_core {

// 六腿共同执行一个规划段时使用的平均速度约束。
struct body_connection_request {
    // 整个规划段的时长，单位 s。
    double duration_s;

    // 第一维按 leg_order 排列。
    // 第二维按 joint_order 排列，单位 rad/s。
    std::array<std::array<double, joints_per_leg>, leg_count > max_joint_speed_rad_s;

    body_connection_request(double duration, const std::array<std::array<double, joints_per_leg>, leg_count>& speed_limits) : duration_s(duration), max_joint_speed_rad_s(speed_limits) {
    }
};

// 一次六腿目标求解的完整输入。
struct body_ik_request {
    // 六个目标脚点，均在机身坐标系中表达，单位 m。
    std::array<vector3, leg_count> points_in_body;

    // 各腿的有效参考关节角，单位 rad。
    std::array<leg_joint_angles, leg_count> reference_angles;

    // 各腿已有的分支记忆；字段无值表示没有该项记忆。
    std::array<leg_branch_state, leg_count> remembered_branches;

    // 本次六腿求解采用的候选选择策略。
    leg_selection_options selection_options;

    // 无值表示本次不要求平均速度检查。
    std::optional<body_connection_request> connection;

    body_ik_request(
        const std::array<vector3, leg_count>& points,
        const std::array<leg_joint_angles, leg_count>& references,
        const std::array<leg_branch_state, leg_count>& branches,
        const leg_selection_options& options,
        std::optional<body_connection_request> connection_request = std::nullopt
    )
        : points_in_body(points),
          reference_angles(references),
          remembered_branches(branches),
          selection_options(options),
          connection(connection_request) {
    }
};

// 六条腿全部求解成功后，才能形成的完整结果。
struct body_ik_solution {
    // 按物理舵机 ID-1 排列的目标容器。
    joint_targets targets;

    // 六腿整数位置解码后的名义关节角。
    // 按 leg_order 排列，供上层接受本帧后作为下一帧参考。
    std::array<leg_joint_angles, leg_count> nominal_angles;

    // 各腿本次选择得到的分支记忆。
    std::array<leg_branch_state, leg_count> branches;
};

enum class body_ik_status {
    success,
    leg_selection_failed
};

struct body_ik_result {
    body_ik_status status = body_ik_status::leg_selection_failed;

    // 单腿求解失败时，指出首先失败的腿。
    std::optional<leg_id> failed_leg;

    // 复用已有单腿状态，说明具体失败原因。
    std::optional<leg_selection_status> failure_reason;

    // 只有六腿全部成功时才有值。
    std::optional<body_ik_solution> solution;
};

class body_ik_solver {
 public:
    explicit body_ik_solver(robot_model model);

    // 求解六腿目标。
    // 任意一腿失败，不返回部分执行目标。
    // 不发送命令，也不修改调用方的参考姿态或分支记忆。
    body_ik_result solve(const body_ik_request& request) const;

 private:
    robot_model model_;
    leg_kinematics kinematics_;
    leg_candidate_selector selector_;
};


} // namespace hexapod_core
