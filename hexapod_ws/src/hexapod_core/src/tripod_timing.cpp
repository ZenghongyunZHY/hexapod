#include "hexapod_core/tripod_timing.hpp"

#include <cmath>
#include <stdexcept>

namespace hexapod_core {

namespace {

void require_timing_request(const tripod_timing_request& request) {
    if (!std::isfinite(request.period_s) || request.period_s <= 0.0) {
        throw std::invalid_argument("Tripod period must be finite and positive");
    }

    if (!std::isfinite(request.stance_ratio) || request.stance_ratio < 0.5 || request.stance_ratio >= 1.0) {
        throw std::invalid_argument("Stance ratio must be finite and within [0.5, 1)");
    }
}

void mark_group_a(std::array<bool, leg_count>& marked) {
    marked[leg_index(leg_id::right_front)] = true;
    marked[leg_index(leg_id::left_middle)] = true;
    marked[leg_index(leg_id::right_rear)] = true;
}

void mark_group_b(std::array<bool, leg_count>& marked) {
    marked[leg_index(leg_id::left_front)] = true;
    marked[leg_index(leg_id::right_middle)] = true;
    marked[leg_index(leg_id::left_rear)] = true;
}

// 不通过容差合并不同边界。
// 如果时间精度不足以区分必要事件，明确拒绝这份配置。
void require_distinct_boundary_times(const std::vector<tripod_boundary>& boundaries) {
    for (std::size_t index = 0; index < boundaries.size(); ++index) {
        const auto& current = boundaries[index];

        if (!std::isfinite(current.time_s)) {
            throw std::range_error("Tripod boundary time is not representable");
        }

        if (index == 0) {
            continue;
        }

        const auto& previous = boundaries[index - 1];

        if (current.cycle_ratio <= previous.cycle_ratio || current.time_s <= previous.time_s) {
            throw std::range_error("Adjacent tripod boundaries cannot be distinguished");
        }
    }
}

}  // namespace

std::vector<tripod_boundary> make_tripod_cycle_boundaries(const tripod_timing_request& request) {
    require_timing_request(request);

    const double period = request.period_s;
    const double stance = request.stance_ratio;

    std::vector<tripod_boundary> boundaries;
    boundaries.reserve(5);

    // 周期起点。
    boundaries.emplace_back(0.0, 0.0);

    if (stance == 0.5) {
        // 没有双支撑重叠时，B 组在周期起点直接起脚。
        mark_group_b(boundaries.back().begin_swing);
    }
    else {
        // B 组自身进度相对 A 组偏移半周期。
        // B 组摆动起点在整个周期中的位置为 stance - 0.5。
        const double begin_b_ratio = stance - 0.5;

        boundaries.emplace_back(begin_b_ratio, begin_b_ratio * period);
        mark_group_b(boundaries.back().begin_swing);
    }

    // 周期一半处，B 组完成本次摆动。
    boundaries.emplace_back(0.5, 0.5 * period);
    mark_group_b(boundaries.back().finish_swing);

    if (stance == 0.5) {
        // 同一个时刻，A 组开始摆动。
        mark_group_a(boundaries.back().begin_swing);
    } else {
        // 有双支撑重叠时，A 组稍后起脚。
        boundaries.emplace_back(stance, stance * period);
        mark_group_a(boundaries.back().begin_swing);
    }

    // 周期终点，A 组完成摆动。
    // 此处不提前安排下一周期的 B 组起脚。
    boundaries.emplace_back(1.0, period);
    mark_group_a(boundaries.back().finish_swing);
    boundaries.back().cycle_complete = true;

    require_distinct_boundary_times(boundaries);

    return boundaries;
}

}  // namespace hexapod_core
