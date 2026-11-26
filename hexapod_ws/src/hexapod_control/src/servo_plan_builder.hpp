#pragma once

#include "hexapod_core/tripod_sequence_timeline.hpp"
#include "hexapod_interfaces/msg/servo_plan.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace hexapod_control {

// 从完整的运动学时间轴生成一批消息。发布时由调用方填写 motion_start。
hexapod_interfaces::msg::ServoPlan make_servo_plan(
    const std::vector<hexapod_core::tripod_timed_segment>& timeline,
    std::uint16_t min_move_duration_ms,
    std::uint16_t max_move_duration_ms,
    std::uint64_t plan_id,
    const std::array<std::uint16_t, hexapod_core::joint_count>& initial_positions,
    std::size_t walking_cycle_count
);

}  // namespace hexapod_control
