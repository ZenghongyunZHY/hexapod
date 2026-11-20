#pragma once

#include "hexapod_core/tripod_timing.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace hexapod_core {

// 一个周期采样时刻。
// 普通采样点不带边界事件，关键边界带完整事件。
struct tripod_cycle_sample {
    double cycle_ratio;
    double time_s;

    std::optional<tripod_boundary> boundary;

    tripod_cycle_sample(
        double ratio,
        double time,
        std::optional<tripod_boundary> boundary_event = std::nullopt
    )
        : cycle_ratio(ratio),
          time_s(time),
          boundary(boundary_event) {
    }
};

// 在相邻边界之间均匀分段，生成完整周期采样时间轴。
//
// intervals_per_stage 必须大于 0。
// 每个边界区间划分为指定数量的运动段。
// 相邻区间共用的边界只保存一次。
//
// 初始时刻和周期终点均保留，边界直接使用原始数值。
// 不采样脚点，不推进脚点状态，不执行动作。
//
// 非法分段数抛 std::invalid_argument。
// 点数超过容器容量抛 std::length_error。
// 相邻采样点无法区分时抛 std::range_error。
// 周期配置的异常由边界生成函数继续抛出。
std::vector<tripod_cycle_sample> make_tripod_cycle_samples(const tripod_timing_request& request, std::size_t intervals_per_stage);

}  // namespace hexapod_core
