#pragma once

#include "hexapod_core/types.hpp"

#include <array>

namespace hexapod_core {

// 计划中的腿部阶段，不表示已经观测到真实触地。
enum class leg_phase {
    stance,
    swing
};

// 一条腿在当前时刻的计划阶段。
struct leg_phase_state {
    leg_phase phase;

    // 当前阶段内部的归一化进度。
    // 支撑期表示支撑进度，摆动期表示摆动进度。
    double phase_ratio;
};

// 一次交替三脚步态的相位采样输入。
struct tripod_phase_request {
    // 整个步态的归一化周期进度，必须有限且位于 [0, 1)。
    // 以 A 组为相位基准：A 组使用此进度，B 组偏移半个周期。
    double cycle_ratio;

    // 每条腿在一个周期中处于支撑期的时间比例。
    // 必须有限且位于 [0.5, 1)。
    double stance_ratio;

    tripod_phase_request(double cycle_progress, double stance_fraction) : cycle_ratio(cycle_progress), stance_ratio(stance_fraction) {}
};

// 返回六条腿的计划阶段，数组按 leg_order 排列。
//
// A 组：右前、左中、右后。
// B 组：左前、右中、左后，周期进度相对 A 组偏移 0.5。
//
// 每组自身的周期进度：
// [0, stance_ratio) 为支撑期；
// [stance_ratio, 1) 为摆动期。
//
// 只计算当前相位，不保存历史、不统计周期数，
// 不生成脚点，也不检测两次调用之间跨过的阶段边界。
// 非法输入抛出 std::invalid_argument。
std::array<leg_phase_state, leg_count>  sample_tripod_phase(const tripod_phase_request& request);

}  // namespace hexapod_core
