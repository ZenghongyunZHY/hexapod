#include "servo_plan_builder.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace hexapod_control {
namespace {

std::uint32_t scaled_boundary_ms(double seconds, double scale) {
    const double rounded = std::round(seconds * scale * 1000.0);
    if (!std::isfinite(rounded) || rounded < 0.0 || rounded > std::numeric_limits<std::uint32_t>::max()) {
        throw std::out_of_range("Scaled time does not fit a millisecond offset");
    }
    return static_cast<std::uint32_t>(rounded);
}

}  // namespace

hexapod_interfaces::msg::ServoPlan make_servo_plan(
    const std::vector<hexapod_core::tripod_timed_segment>& timeline,
    double scale,
    std::uint16_t min_move_duration_ms,
    std::uint16_t max_move_duration_ms,
    std::uint64_t plan_id,
    const std::array<std::uint16_t, hexapod_core::joint_count>& initial_positions,
    std::size_t walking_cycle_count
) {
    if (timeline.empty() || timeline.size() > std::numeric_limits<std::uint32_t>::max()) {
        throw std::invalid_argument("Servo plan requires a nonempty, addressable timeline");
    }
    if (!std::isfinite(scale) || scale < 1.0) {
        throw std::invalid_argument("Timing scale must be finite and at least one");
    }
    if (min_move_duration_ms == 0 || min_move_duration_ms > max_move_duration_ms) {
        throw std::invalid_argument("Invalid move duration bounds");
    }
    if (walking_cycle_count > 0 && timeline.back().cycle_index < walking_cycle_count - 1) {
        throw std::invalid_argument("Timeline is missing a walking cycle");
    }

    for (const auto value : initial_positions) {
        (void)hexapod_core::servo_position(value);
    }

    hexapod_interfaces::msg::ServoPlan plan;
    plan.plan_id = plan_id;
    plan.initial_positions = initial_positions;
    plan.segments.reserve(timeline.size());

    double previous_end_s = 0.0;
    std::uint32_t start_ms = 0;
    std::size_t expected_cycle = 0;
    std::size_t expected_end_frame = 1;
    for (std::size_t index = 0; index < timeline.size(); ++index) {
        const auto& segment = timeline[index];
        if (segment.cycle_index != expected_cycle || segment.end_frame_index != expected_end_frame ||
            segment.start_time_s != previous_end_s || !(segment.end_time_s > segment.start_time_s)) {
            throw std::invalid_argument("Discontinuous timeline at segment " + std::to_string(index));
        }

        // 共享相邻段的整数毫秒边界，避免逐段取整造成累计误差。
        const auto end_ms = scaled_boundary_ms(segment.end_time_s, scale);
        if (end_ms <= start_ms) {
            throw std::out_of_range("Rounding collapsed segment " + std::to_string(index));
        }
        const auto duration_ms = end_ms - start_ms;
        if (duration_ms < min_move_duration_ms || duration_ms > max_move_duration_ms) {
            throw std::out_of_range("Segment " + std::to_string(index) + " does not fit the move duration bounds");
        }
        constexpr double roundoff_ms = 1e-6;
        if (duration_ms + roundoff_ms < (segment.end_time_s - segment.start_time_s) * 1000.0) {
            throw std::out_of_range("Rounded duration would shorten segment " + std::to_string(index));
        }

        const bool cycle_end = index == timeline.size() - 1 || timeline[index + 1].cycle_index != segment.cycle_index;
        auto& command = plan.segments.emplace_back();
        command.start_offset_ms = start_ms;
        command.move_duration_ms = static_cast<std::uint16_t>(duration_ms);
        command.walking_cycle_end = cycle_end && segment.cycle_index < walking_cycle_count;
        for (std::size_t axis = 0; axis < segment.nominal_positions.size(); ++axis) {
            const auto& position = segment.nominal_positions[axis];
            if (!position) {
                throw std::invalid_argument("Missing nominal position for ID " +
                                            std::to_string(axis + 1) + " in segment " +
                                            std::to_string(index));
            }
            command.positions[axis] = static_cast<std::uint16_t>(position->value());
        }

        previous_end_s = segment.end_time_s;
        start_ms = end_ms;
        if (cycle_end) {
            ++expected_cycle;
            expected_end_frame = 1;
        } else {
            ++expected_end_frame;
        }
    }
    return plan;
}

}  // namespace hexapod_control
