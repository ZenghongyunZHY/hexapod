#include "hexapod_core/body_ik_solver.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace hexapod_core {

namespace {

void require_body_connection(const body_connection_request& request) {
    if (!std::isfinite(request.duration_s) || request.duration_s <= 0.0) {
        throw std::invalid_argument("Body connection duration must be finite and positive");
    }

    for (const auto& leg_speed_limits : request.max_joint_speed_rad_s) {
        for (const double speed : leg_speed_limits) {
            if (!std::isfinite(speed) || speed <= 0.0) {
                throw std::invalid_argument("Body joint speed limits must be finite and positive");
            }
        }
    }
}

}  // namespace

body_ik_solver::body_ik_solver(robot_model model) : model_(std::move(model)), kinematics_(model_), selector_(model_) {
}

// ik求解器，如果request，返回result
body_ik_result body_ik_solver::solve(const body_ik_request& request) const {
    // 先检查整份可选速度请求。
    if (request.connection) {
        require_body_connection(*request.connection);
    }

    body_ik_result result;

    // 所有单腿结果先写在局部临时对象中。
    // 只有六腿全部成功，才将它交给调用方。
    body_ik_solution pending{};

    for (const auto leg : leg_order) {

        const auto index = leg_index(leg);

        // 将六腿共同约束转换为当前腿的连接请求。
        std::optional<leg_connection_request> connection_request;

        if (request.connection) {
            connection_request.emplace(request.connection->duration_s, request.connection->max_joint_speed_rad_s[index]);
        }

        // 机身脚点 → 当前腿的几何候选。
        // 先将机身脚点转换到腿根局部坐标，再返回局部 IK 的关节角候选。
        const auto ik_result = kinematics_.inverse_candidates_in_body_frame(leg, request.points_in_body[index]);

        // 候选过滤、策略检查、可选速度检查与评分。
        const auto selected = selector_.select_candidate(
            leg,
            ik_result,
            request.reference_angles[index],
            request.remembered_branches[index],
            request.selection_options,
            connection_request);

        if (selected.status != leg_selection_status::success) {
            result.failed_leg = leg;
            result.failure_reason = selected.status;

            return result;
        }

        // success 必须同时带有实际选中的候选。
        if (!selected.selected) {
            throw std::logic_error("Successful leg selection has no candidate");
        }

        const auto& candidate = *selected.selected;

        // 暂存该腿的名义姿态与分支。
        pending.nominal_angles[index] = candidate.nominal_angles;
        pending.branches[index] = selected.branch;

        const auto& ideal = candidate.ik_candidate.angles;

        const std::array<double, joints_per_leg> desired_angles{
            ideal.yaw_rad,
            ideal.upper_leg_rad,
            ideal.lower_leg_rad
        };

        // 将腿内三个关节写入按物理 ID 排列的目标数组。
        for (std::size_t joint_offset = 0; joint_offset < joints_per_leg; ++joint_offset) {
            const auto id = model_.joint(leg, joint_order[joint_offset]).binding.id;

            pending.targets.desired_angles_rad[id.index()] = desired_angles[joint_offset];

            pending.targets.nominal_positions[id.index()] = candidate.positions[joint_offset];
        }
    }

    // 循环完整结束，说明六条腿全部成功。
    result.solution = std::move(pending);
    result.status = body_ik_status::success;

    return result;
}


}  // namespace hexapod_core
