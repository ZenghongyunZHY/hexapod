#pragma once

#include "hexapod_core/body_ik_solver.hpp"
#include "hexapod_core/leg_path_checker.hpp"

#include <array>
#include <cstddef>
#include <optional>

namespace hexapod_core {

// 固定机身参考下的一次六腿运动段规划请求。
struct body_segment_request {
    // 六腿目标、参考姿态、分支记忆、选择策略和可选速度约束。
    body_ik_request target_request;

    // 六腿共同使用的机身坐标参考平面。
    ground_plane_in_body ground;

    // 每条腿在整个段内需要满足的最小离面距离。
    // 按 leg_order 排列，单位 m。
    std::array<double, leg_count> minimum_clearance_m;

    // 各腿路径检查使用的共同区间数。
    std::size_t interval_count;

    body_segment_request(
        const body_ik_request& target,
        const ground_plane_in_body& plane,
        const std::array<double, leg_count>& minimum_clearances,
        std::size_t intervals
    )
        : target_request(target),
          ground(plane),
          minimum_clearance_m(minimum_clearances),
          interval_count(intervals) {
    }
};

enum class body_segment_status {
    checks_passed,              	//六腿目标选择和本次路径采样检查全部通过
    target_selection_failed,        //目标求解或候选选择阶段失败
    path_check_failed               //目标已选出，但某条腿的路径检查失败
};

struct body_segment_result {
    body_segment_status status;

    // 目标选择或路径检查失败时，指出失败腿。
    std::optional<leg_id> failed_leg;

    // 仅目标选择失败时有值。
    std::optional<leg_selection_status> selection_failure;

    // 仅路径检查失败时有值。
    std::optional<leg_path_check_result> path_failure;

    // 只有目标选择与六腿路径检查全部通过时才有值。
    std::optional<body_ik_solution> solution;

    explicit body_segment_result(
        body_segment_status checked_status
    )
        : status(checked_status) {
    }
};

class body_segment_planner {
 public:
    explicit body_segment_planner(robot_model model);

    // 先选择六腿目标，再检查参考姿态到名义目标的路径。
    // 全部通过才返回完整 solution。
    // 不发送命令，不修改调用方的参考姿态和分支记忆。
    body_segment_result plan(const body_segment_request& request) const;

 private:
    body_ik_solver solver_;
    leg_path_checker path_checker_;
};

}  // namespace hexapod_core
