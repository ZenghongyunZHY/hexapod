#include "hexapod_core/body_segment_planner.hpp"

#include <stdexcept>
#include <utility>
#include <vector>

namespace hexapod_core {

namespace {

void require_segment_request(const body_segment_request& request) {
    if (request.interval_count == 0) {
        throw std::invalid_argument("Body segment requires at least one path interval");
    }

    const auto maximum_sample_count = std::vector<leg_path_sample>{}.max_size();

    if (request.interval_count >= maximum_sample_count) {
        throw std::length_error("Body segment path interval count is too large");
    }

    for (const double minimum_clearance : request.minimum_clearance_m) {
        require_path_constraints(leg_path_constraints{request.ground, minimum_clearance});
    }
}

}  // namespace

body_segment_planner::body_segment_planner(robot_model model) : solver_(model), path_checker_(std::move(model)) {}

//上层已经利用sample_swing_trajectory将运动轨迹分成了若干段，plan用于检查当前段的链路
body_segment_result body_segment_planner::plan(const body_segment_request& request) const {

    require_segment_request(request);

    auto selected = solver_.solve(request.target_request);

    // 第一阶段：六腿目标选择失败。
    if (selected.status == body_ik_status::leg_selection_failed) {

        if (!selected.failed_leg || !selected.failure_reason || selected.solution) {
            throw std::logic_error("Inconsistent failed body IK result");
        }

        body_segment_result result{body_segment_status::target_selection_failed};

        result.failed_leg = selected.failed_leg;
        result.selection_failure = selected.failure_reason;

        return result;
    }

    // 成功结果必须带有完整解，且没有失败信息。
    if (selected.status != body_ik_status::success || !selected.solution || selected.failed_leg || selected.failure_reason) {
        throw std::logic_error("Inconsistent successful body IK result");
    }

    const auto& solution = *selected.solution;

    // 第二阶段：逐腿检查到名义目标的连接路径。
    for (const auto leg : leg_order) {

        const auto index = leg_index(leg);

        const leg_path_constraints constraints{request.ground, request.minimum_clearance_m[index]};

        const auto checked = path_checker_.check_linear_segment(leg, request.target_request.reference_angles[index], solution.nominal_angles[index], request.interval_count, constraints);

        switch (checked.status) {
            case leg_path_check_status::samples_passed:
                break;

            case leg_path_check_status::joint_limit_exceeded:


            case leg_path_check_status::foot_clearance_below_minimum: {

                body_segment_result result{body_segment_status::path_check_failed};

                result.failed_leg = leg;
                result.path_failure = checked;

                return result;
            }

            default:
                throw std::logic_error("Unexpected leg path check status");
        }
    }

    // 六条腿的目标选择与路径检查全部完成。
    body_segment_result result{body_segment_status::checks_passed};

    result.solution = std::move(selected.solution);

    return result;
}

}  // namespace hexapod_core
