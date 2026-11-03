#pragma once

#include "hexapod_core/leg_path_sampler.hpp"

#include <cstddef>
#include <optional>

namespace hexapod_core {

// 在机身坐标系中表达的参考平面：
// unit_normal · point + offset_m = 0。
struct ground_plane_in_body {
    // 单位法向量，指向约定的平面上方。
    vector3 unit_normal;

    // 平面偏移，单位 m。
    double offset_m;

    ground_plane_in_body(const vector3& normal, double offset) : unit_normal(normal), offset_m(offset) {}
};


struct leg_path_constraints {
    ground_plane_in_body ground;

    // 全部采样点需要满足的最小有符号离面距离，单位 m。
    // 必须有限且不小于 0。
    double minimum_clearance_m;

    leg_path_constraints(const ground_plane_in_body& plane, double minimum_clearance) : ground(plane), minimum_clearance_m(minimum_clearance) {}
};

// 检查参考平面与最小离面距离是否合法。
// 非法输入抛出 std::invalid_argument。
void require_path_constraints(const leg_path_constraints& constraints);

enum class leg_path_check_status {
    samples_passed,
    joint_limit_exceeded,
    foot_clearance_below_minimum
};

struct leg_path_check_result {
    leg_path_check_status status;

    // 已检查的采样点数，包含导致失败的那个点。
    std::size_t checked_sample_count;

    // 失败点在采样数组中的下标。
    std::optional<std::size_t> failed_sample_index;

    // 失败点的比例、关节角和机身脚点。
    std::optional<leg_path_sample> failed_sample;

    // 仅关节限位失败时有值。
    std::optional<joint_type> failed_joint;

    // 仅离面距离失败时有值，保存该点实际计算的距离。
    std::optional<double> failed_clearance_m;

    leg_path_check_result(leg_path_check_status checked_status, std::size_t sample_count) : status(checked_status), checked_sample_count(sample_count) {}
};

class leg_path_checker {
 public:
    explicit leg_path_checker(robot_model model);

    // 在线性关节插值模型下进行离散路径检查。
    // 返回首次失败的位置和原因。
    // 当前参考平面在整个检查段内保持不变。
    leg_path_check_result check_linear_segment(
        leg_id leg,
        const leg_joint_angles& start_nominal,
        const leg_joint_angles& end_nominal,
        std::size_t interval_count,
        const leg_path_constraints& constraints
    ) const;

 private:
    robot_model model_;
    leg_path_sampler sampler_;
};

}  // namespace hexapod_core
