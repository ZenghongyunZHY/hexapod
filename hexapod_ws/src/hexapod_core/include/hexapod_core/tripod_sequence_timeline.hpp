#pragma once

#include "hexapod_core/tripod_sequence_planner.hpp"

#include <cstddef>
#include <vector>

namespace hexapod_core {

struct tripod_timed_segment {
    // 周期和帧下标从 0 开始。起点帧为 end_frame_index - 1。
    std::size_t cycle_index;
    std::size_t end_frame_index;

    // 时间相对整个序列的起点，单位 s。
    double start_time_s;
    double end_time_s;

    // 段终点的完整名义位置，按物理舵机 ID - 1 排列。
    servo_positions nominal_positions;
};

std::vector<tripod_timed_segment> make_tripod_sequence_timeline(const tripod_sequence_solution& solution);

}  // namespace hexapod_core
