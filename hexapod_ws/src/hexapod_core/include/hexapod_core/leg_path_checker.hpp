#pragma once

#include "hexapod_core/leg_path_sampler.hpp"

#include <cstddef>
#include <optional>
#include <vector>

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

    // 整个运动段内的最小有符号离面距离，单位 m。
    // 必须有限。
    // 负值表示本次检查显式允许的名义几何偏差。
    double minimum_clearance_m;

    // 整个运动段内的最大有符号离面距离，单位 m。
    // 无值表示不检查上界。
    // 有值时必须有限，且不小于下界。
    std::optional<double> maximum_clearance_m;

    leg_path_constraints(const ground_plane_in_body& plane, double minimum_clearance, std::optional<double> maximum_clearance = std::nullopt)
        : ground(plane),
          minimum_clearance_m(minimum_clearance),
          maximum_clearance_m(maximum_clearance) {
    }
};

// 检查参考平面与最小离面距离是否合法。
// 非法输入抛出 std::invalid_argument。
void require_path_constraints(const leg_path_constraints& constraints);

enum class leg_path_check_status {
    samples_passed,
    joint_limit_exceeded,
    foot_clearance_below_minimum,
    foot_clearance_above_maximum
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

    // 整条运动段使用同一份约束。
    // 保留已有调用方式。
    leg_path_check_result check_linear_segment(
        leg_id leg,
        const leg_joint_angles& start_nominal,
        const leg_joint_angles& end_nominal,
        std::size_t interval_count,
        const leg_path_constraints& constraints
    ) const;

    // 每个路径采样点使用各自的约束。
    //
    // interval_count 个区间对应 interval_count + 1 个采样点，
    // sample_constraints 必须具有相同数量的元素。
    //
    // 第 i 份约束对应第 i 个均匀路径采样点。
    // 起点和终点也分别具有一份约束。
    //
    // 检查全部约束是否合法后，再开始路径检查。
    // 约束数量不匹配或约束非法时抛出异常。
    leg_path_check_result check_linear_segment_with_sample_constraints(
        leg_id leg,
        const leg_joint_angles& start_nominal,
        const leg_joint_angles& end_nominal,
        std::size_t interval_count,
        const std::vector<leg_path_constraints>& sample_constraints
    ) const;

 private:
    // 两种公开入口共享实际的逐点检查过程。
    // 调用前保证采样点与约束数量一致，且约束已经校验。
    leg_path_check_result check_samples(
        leg_id leg,
        const std::vector<leg_path_sample>& samples,
        const std::vector<leg_path_constraints>& sample_constraints
    ) const;

    robot_model model_;
    leg_path_sampler sampler_;
};

}  // namespace hexapod_core
