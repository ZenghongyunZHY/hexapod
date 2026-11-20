#pragma once

#include "hexapod_core/types.hpp"

#include <array>
#include <vector>

namespace hexapod_core {

// 一个交替三脚周期的时间配置。
struct tripod_timing_request {
    // 完整周期时长，单位 s，必须有限且为正。
    double period_s;

    // 支撑占空比，必须有限且位于 [0.5, 1)。
    double stance_ratio;

    tripod_timing_request(double period, double stance_fraction) : period_s(period), stance_ratio(stance_fraction) {}
};

// 一个周期内部的计划边界。
// 腿数组按 leg_order 排列。
struct tripod_boundary {
    // 周期进度，包含起点 0 和终点 1。
    double cycle_ratio;

    // 相对本周期起点的时间，单位 s。
    double time_s;

    // 本边界需要开始计划摆动的腿。
    std::array<bool, leg_count> begin_swing{};

    // 本边界需要完成计划摆动的腿。
    std::array<bool, leg_count> finish_swing{};

    // 仅最后一个边界为 true。
    // 上层接受最后运动段后，才能据此增加已完成周期数。
    bool cycle_complete = false;

    tripod_boundary(double ratio, double time) : cycle_ratio(ratio), time_s(time) {}
};

// 生成一个周期的有序边界，包含时间 0 和周期终点。
//
// stance_ratio > 0.5 时返回 5 个边界。
// stance_ratio = 0.5 时返回 3 个边界，同刻事件合并。
//
// 周期终点只安排本周期的落脚，不自动开始下一周期。
// 下一周期的起脚由下一周期时间 0 的边界处理。
//
// 不采样脚点，不修改脚点状态，不执行动作。
// 非法配置抛 std::invalid_argument。
// 无法用 double 区分相邻边界时抛 std::range_error。
std::vector<tripod_boundary> make_tripod_cycle_boundaries(const tripod_timing_request& request);

}  // namespace hexapod_core
