#include "hexapod_core/tripod_phase.hpp"

#include <cmath>
#include <stdexcept>

namespace hexapod_core {

namespace {

// 校验整份请求。
// 当前函数只接受一个周期内部的归一化进度，
// 不替调用方处理负时间、多周期累计或周期计数。
void require_tripod_phase_request(const tripod_phase_request& request) {
    if (!std::isfinite(request.cycle_ratio) || request.cycle_ratio < 0.0 || request.cycle_ratio >= 1.0) {
        throw std::invalid_argument("Cycle ratio must be finite and within [0, 1)");
    }

    if (!std::isfinite(request.stance_ratio) || request.stance_ratio < 0.5 || request.stance_ratio >= 1.0) {
        throw std::invalid_argument("Stance ratio must be finite and within [0.5, 1)");
    }
}

// 根据一组自身的周期进度，计算阶段与阶段内部进度。
// 调用前已经完成输入校验。
leg_phase_state sample_group_phase(double cycle_ratio, double stance_ratio) {

    if (cycle_ratio < stance_ratio) {
        return leg_phase_state{leg_phase::stance, cycle_ratio / stance_ratio};
    }

    const double swing_ratio = (cycle_ratio - stance_ratio) / (1.0 - stance_ratio);

    return leg_phase_state{leg_phase::swing, swing_ratio};
}

// 计算相对输入进度偏移半个周期的 B 组。
//
// 数学上等价于：
// group_cycle = wrap(cycle_ratio + 0.5);
//
// 实现时直接在原始进度中判断阶段边界，
// 避免 cycle_ratio + 0.5 的舍入导致提前跨过边界。
leg_phase_state sample_shifted_group_phase(double cycle_ratio, double stance_ratio) {

    // 输入处于后半周期时，B 组已经回绕。
    // B 组自身进度位于 [0, 0.5)，必定属于支撑期，
    // 因为 stance_ratio >= 0.5。
    if (cycle_ratio >= 0.5) {
        const double group_cycle_ratio = cycle_ratio - 0.5;

        return leg_phase_state{leg_phase::stance, group_cycle_ratio / stance_ratio};
    }

    // 输入处于前半周期时，B 组自身进度为：
    // cycle_ratio + 0.5。
    //
    // B 组进入摆动期的条件：
    // cycle_ratio + 0.5 >= stance_ratio，
    // 即 cycle_ratio >= stance_ratio - 0.5。
    const double swing_begin_ratio = stance_ratio - 0.5;

    if (cycle_ratio < swing_begin_ratio) {

        const double group_cycle_ratio = cycle_ratio + 0.5;

        return leg_phase_state{leg_phase::stance, group_cycle_ratio / stance_ratio};
    }

    // 与以下公式数学等价：
    // (cycle_ratio + 0.5 - stance_ratio)
    // / (1.0 - stance_ratio)。
    //
    // 先减去原始周期中的摆动起点，
    // 可以减少边界附近相加再相减的舍入影响。
    const double swing_ratio = (cycle_ratio - swing_begin_ratio) / (1.0 - stance_ratio);

    return leg_phase_state{leg_phase::swing, swing_ratio};

}

}  // namespace

std::array<leg_phase_state, leg_count> sample_tripod_phase(const tripod_phase_request& request) {

    require_tripod_phase_request(request);

    const auto group_a = sample_group_phase(request.cycle_ratio, request.stance_ratio);

    const auto group_b = sample_shifted_group_phase(request.cycle_ratio, request.stance_ratio);

    std::array<leg_phase_state, leg_count> result{};

    // A 组：右前、左中、右后。
    result[leg_index(leg_id::right_front)] = group_a;
    result[leg_index(leg_id::left_middle)] = group_a;
    result[leg_index(leg_id::right_rear)] = group_a;

    // B 组：左前、右中、左后。
    result[leg_index(leg_id::left_front)] = group_b;
    result[leg_index(leg_id::right_middle)] = group_b;
    result[leg_index(leg_id::left_rear)] = group_b;

    return result;
}

}  // namespace hexapod_core
