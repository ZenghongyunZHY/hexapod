#pragma once

#include "hexapod_core/body_ik_solver.hpp"
#include "hexapod_core/leg_path_checker.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

namespace hexapod_core {

// 六腿各自的路径采样约束。
// 外层按 leg_order 排列，内层按路径采样顺序排列。
using body_path_constraint_samples = std::array<std::vector<leg_path_constraints>, leg_count>;

struct body_segment_request {
    body_ik_request target_request;

    ground_plane_in_body ground;

    // 各腿在整个段内的最小有符号离面距离。
    // 必须有限，可以为负。
    std::array<double, leg_count> minimum_clearance_m;

    std::size_t interval_count;

    // 各腿可选的最大有符号离面距离。
    // 无值表示该腿本段不检查上界。
    std::array<std::optional<double>, leg_count> maximum_clearance_m;

    // 无值：使用上面的固定约束。
    //
    // 有值：六条腿全部使用这里的逐点约束，
    // 每条腿都必须提供 interval_count + 1 个元素。
    //
    // 逐点模式的每个元素已经包含参考平面和上下界，
    // 因此该模式不使用上面的固定平面与固定上下界。
    std::optional<body_path_constraint_samples> sample_constraints;

    body_segment_request(
        const body_ik_request& target,
        const ground_plane_in_body& plane,
        const std::array<double, leg_count>& minimum_clearances,
        std::size_t intervals,
        const std::array<std::optional<double>, leg_count>& maximum_clearances = {},
        std::optional<body_path_constraint_samples> sampled_constraints = std::nullopt
    )
        : target_request(target),
          ground(plane),
          minimum_clearance_m(minimum_clearances),
          interval_count(intervals),
          maximum_clearance_m(maximum_clearances),
          sample_constraints(std::move(sampled_constraints)) {
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

    // 六腿目标选择和路径检查全部通过才返回完整结果。
    // 不修改调用方的参考姿态或分支记忆。
    body_segment_result plan(const body_segment_request& request) const;

 private:
    body_ik_solver solver_;
    leg_path_checker path_checker_;
};

}  // namespace hexapod_core
