#include "hexapod_core/tripod_sequence_timeline.hpp"

#include <cmath>
#include <stdexcept>

namespace hexapod_core {

std::vector<tripod_timed_segment> make_tripod_sequence_timeline(const tripod_sequence_solution& solution) {
    if (solution.cycles.empty()) {
        throw std::invalid_argument("Tripod timeline requires a complete sequence");
    }

    std::vector<tripod_timed_segment> timeline;
    double cycle_start_s = 0.0;

    for (std::size_t cycle_index = 0; cycle_index < solution.cycles.size(); ++cycle_index) {
        const auto& cycle = solution.cycles[cycle_index];
        const auto& frames = cycle.foot_cycle.frames;
        const auto& segments = cycle.segments;

        if (frames.size() < 2 || frames.size() - 1 != segments.size()) {
            throw std::invalid_argument("Tripod cycle frame and segment counts do not match");
        }
        if (frames.front().sample.time_s != 0.0) {
            throw std::invalid_argument("Tripod cycle must start at local time zero");
        }

        double previous_local_s = 0.0;
        double previous_end_s = cycle_start_s;

        for (std::size_t segment_index = 0; segment_index < segments.size(); ++segment_index) {
            const std::size_t end_frame_index = segment_index + 1;
            const double local_end_s = frames[end_frame_index].sample.time_s;

            if (!std::isfinite(local_end_s) || local_end_s <= previous_local_s) {
                throw std::invalid_argument("Tripod cycle times must be finite and increasing");
            }

            const double end_time_s = cycle_start_s + local_end_s;
            if (!std::isfinite(end_time_s) || end_time_s <= previous_end_s) {
                throw std::range_error("Tripod segment times cannot be distinguished on the timeline");
            }

            const auto& positions = segments[segment_index].targets.nominal_positions;
            for (const auto& position : positions) {
                if (!position) {
                    throw std::invalid_argument("Tripod segment is missing a nominal servo position");
                }
            }

            timeline.push_back({
                cycle_index,
                end_frame_index,
                previous_end_s,
                end_time_s,
                positions
            });

            previous_local_s = local_end_s;
            previous_end_s = end_time_s;
        }

        // 下一周期的局部 0 s 与本周期最后一段的终点共用时刻。
        cycle_start_s = previous_end_s;
    }

    return timeline;
}

}  // namespace hexapod_core
