#include "hexapod_core/tripod_sampling.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hexapod_core {

namespace {

void require_ordered_samples(const std::vector<tripod_cycle_sample>& samples) {
    for (std::size_t index = 0; index < samples.size(); ++index) {
        const auto& current = samples[index];

        if (!std::isfinite(current.cycle_ratio) || !std::isfinite(current.time_s)) {
            throw std::range_error("Tripod sample is not finite");
        }

        if (index == 0) {
            continue;
        }

        const auto& previous = samples[index - 1];

        if (current.cycle_ratio <= previous.cycle_ratio || current.time_s <= previous.time_s) {
            throw std::range_error("Adjacent tripod samples cannot be distinguished");
        }
    }
}

}  // namespace

std::vector<tripod_cycle_sample> make_tripod_cycle_samples(const tripod_timing_request& request, std::size_t intervals_per_stage) {
    return make_tripod_cycle_samples(request, intervals_per_stage, intervals_per_stage);
}

std::vector<tripod_cycle_sample> make_tripod_cycle_samples(
    const tripod_timing_request& request,
    std::size_t double_support_intervals,
    std::size_t swing_intervals) {
    if (double_support_intervals == 0 || swing_intervals == 0) {
        throw std::invalid_argument("Each tripod stage requires at least one interval");
    }

    const auto boundaries = make_tripod_cycle_boundaries(request);
    const std::size_t stage_count = boundaries.size() - 1;

    std::vector<tripod_cycle_sample> samples;
    const auto intervals_for = [&](const tripod_boundary& start) {
        const auto& starts_swing = start.begin_swing;
        return std::find(starts_swing.begin(), starts_swing.end(), true) != starts_swing.end()
            ? swing_intervals
            : double_support_intervals;
    };

    std::size_t sample_count = 1;
    for (std::size_t stage = 0; stage < stage_count; ++stage) {
        const auto intervals = intervals_for(boundaries[stage]);
        if (intervals > samples.max_size() - sample_count) {
            throw std::length_error("Tripod sample count exceeds container capacity");
        }
        sample_count += intervals;
    }
    samples.reserve(sample_count);

    // 初始点只保存一次，并保留可能存在的起脚事件。
    const auto& initial = boundaries.front();

    samples.emplace_back(initial.cycle_ratio, initial.time_s, initial);

    for (std::size_t stage = 0; stage < stage_count; ++stage) {
        const auto& start = boundaries[stage];
        const auto& end = boundaries[stage + 1];
        const auto intervals_per_stage = intervals_for(start);

        // 起点已经由前一个区间保存，从第一个内部点开始。
        for (std::size_t step = 1; step <= intervals_per_stage; ++step) {
            if (step == intervals_per_stage) {
                // 区间终点直接复制原边界。
                // 不通过插值重新计算关键边界时间。
                samples.emplace_back(end.cycle_ratio, end.time_s, end);

                continue;
            }

            const double ratio = static_cast<double>(step) / static_cast<double>(intervals_per_stage);

            const double start_weight = 1.0 - ratio;

            const double cycle_ratio = start_weight * start.cycle_ratio + ratio * end.cycle_ratio;

            const double time_s = start_weight * start.time_s + ratio * end.time_s;

            // 普通内部采样点不携带边界事件。
            samples.emplace_back(cycle_ratio, time_s);
        }
    }

    require_ordered_samples(samples);

    return samples;
}

}  // namespace hexapod_core
