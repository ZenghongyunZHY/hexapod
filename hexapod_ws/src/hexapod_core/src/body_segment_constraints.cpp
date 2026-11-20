#include "hexapod_core/tripod_segment_constraints.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace hexapod_core {

namespace {

void require_phase(const leg_phase_state& phase) {
    if (phase.phase != leg_phase::stance && phase.phase != leg_phase::swing) {
        throw std::invalid_argument("Unknown leg phase");
    }

    if (!std::isfinite(phase.phase_ratio) || phase.phase_ratio < 0.0 || phase.phase_ratio > 1.0) {
        throw std::invalid_argument(
            "Leg phase ratio must be finite and within [0, 1]"
        );
    }
}


// 将规划参考系中的高度转换为相对地面的高度。
// 不把负的理想高度自动夹紧为零。
double relative_height(double planning_height_m, double ground_height_m) {
    if (!std::isfinite(planning_height_m)) {
        throw std::invalid_argument("Planning foot height must be finite");
    }

    const double height_m = planning_height_m - ground_height_m;

    if (!std::isfinite(height_m)) {
        throw std::overflow_error("Relative foot height overflowed");
    }

    if (height_m < 0.0) {
        throw std::invalid_argument("Ideal foot height must not be below planning ground");
    }

    return height_m;
}

}  // namespace


body_path_constraint_samples make_tripod_segment_constraints(const tripod_foot_frame& previous,const tripod_foot_frame& current,double ground_height_m,std::size_t interval_count,const foot_clearance_policy& policy) {
    require_foot_clearance_policy(policy);

    if (interval_count == 0) {
        throw std::invalid_argument("Segment constraints require at least one interval");
    }

    const auto maximum_constraint_count = std::vector<leg_path_constraints>{}.max_size();

    const auto maximum_path_count = std::vector<leg_path_sample>{}.max_size();

    if (interval_count >= maximum_constraint_count || interval_count >= maximum_path_count) {
        throw std::length_error( "Segment constraint count is too large" );
    }

    if (!std::isfinite(ground_height_m)) {
        throw std::invalid_argument("Planning ground height must be finite");
    }

    if (!std::isfinite(previous.sample.time_s) || !std::isfinite(current.sample.time_s) || previous.sample.time_s < 0.0 || current.sample.time_s <= previous.sample.time_s) {
        throw std::invalid_argument("Segment frame times must be finite and increasing");
    }

    const double body_height_m = previous.body_pose.body_origin_in_planning_frame.z;

    const double next_body_height_m = current.body_pose.body_origin_in_planning_frame.z;

    if (!std::isfinite(body_height_m) || !std::isfinite(next_body_height_m) || body_height_m != next_body_height_m) {
        throw std::invalid_argument("Segment requires a finite constant body height" );
    }

    const double ground_offset_m = body_height_m - ground_height_m;

    if (!std::isfinite(ground_offset_m)) {
        throw std::overflow_error("Body-frame ground offset overflowed");
    }

    const ground_plane_in_body ground{
        {0.0, 0.0, 1.0},
        ground_offset_m
    };

    // 只修改这些局部曲线副本的 Z 坐标，
    // 将其转换为相对地面的高度。
    auto relative_swings = previous.outgoing_swings;

    std::array<double, leg_count> stance_heights{};

    // 先核对全部六腿的数据，再生成约束数组。
    for (const auto leg : leg_order) {
        const auto index = leg_index(leg);

        const auto& start_phase = previous.phases[index];
        const auto& end_phase = current.phases[index];

        require_phase(start_phase);
        require_phase(end_phase);

        auto& swing = relative_swings[index];

        if (swing) {
            if (start_phase.phase != leg_phase::swing || end_phase.phase != leg_phase::swing || end_phase.phase_ratio <= start_phase.phase_ratio) {
                throw std::invalid_argument("Active swing requires increasing swing phases");
            }

            if (end_phase.phase_ratio == 1.0 && (!current.sample.boundary || !current.sample.boundary->finish_swing[index])) {
                throw std::invalid_argument("Swing endpoint requires a finish boundary");
            }

            // 先平移原曲线的起终点高度，
            // 再复用原来的摆动采样函数。
            swing->start_in_planning_frame.z = relative_height(
                swing->start_in_planning_frame.z,
                ground_height_m
            );

            swing->end_in_planning_frame.z = relative_height(
                swing->end_in_planning_frame.z,
                ground_height_m
            );

            // 复用原曲线的完整几何输入校验。
            (void)sample_swing_trajectory(*swing, 0.0);
            (void)sample_swing_trajectory(*swing, 1.0);
        } else {
            // 没有后续摆动：
            // 起点可以是支撑，或刚完成旧摆动；
            // 终点可以是支撑，或刚开始新摆动。
            const bool valid_start = start_phase.phase == leg_phase::stance || (start_phase.phase == leg_phase::swing && start_phase.phase_ratio == 1.0);

            const bool valid_end = end_phase.phase == leg_phase::stance || (end_phase.phase == leg_phase::swing && end_phase.phase_ratio == 0.0);

            if (!valid_start || !valid_end) {
                throw std::invalid_argument("Support segment has inconsistent endpoint phases");
            }

            stance_heights[index] = relative_height(previous.targets.points_in_planning_frame[index].z, ground_height_m);

            const double end_height = relative_height(current.targets.points_in_planning_frame[index].z, ground_height_m);

            if (end_height != stance_heights[index]) {
                throw std::invalid_argument("Support segment changes ideal foot height");
            }
        }
    }

    body_path_constraint_samples result{};

    for (const auto leg : leg_order) {
        const auto index = leg_index(leg);

        auto& constraints = result[index];
        constraints.reserve(interval_count + 1);

        const auto& swing = relative_swings[index];

        for (std::size_t sample_index = 0; sample_index <= interval_count; ++sample_index) {
            const double path_ratio = sample_index == interval_count ? 1.0 : static_cast<double>(sample_index) / static_cast<double>(interval_count);

            // 支撑策略不依赖支撑内部进度。
            // 这里的 0 只用于计算接触范围，
            // 不写回几何帧，也不表示重新开始支撑。
            leg_phase_state phase{leg_phase::stance, 0.0};

            double ideal_clearance_m = stance_heights[index];

            if (swing) {
                const double start_ratio = previous.phases[index].phase_ratio;

                const double end_ratio = current.phases[index].phase_ratio;

                double swing_ratio;

                if (sample_index == 0) {
                    swing_ratio = start_ratio;
                }
                else if (sample_index == interval_count) {
                    swing_ratio = end_ratio;
                }
                else {
                    swing_ratio =
                        (1.0 - path_ratio) * start_ratio
                        + path_ratio * end_ratio;

                    if (swing_ratio <= start_ratio
                        || swing_ratio >= end_ratio) {
                        throw std::range_error(
                            "Interior swing sample cannot be distinguished"
                        );
                    }
                }

                phase = {leg_phase::swing, swing_ratio};

                ideal_clearance_m = sample_swing_trajectory(*swing, swing_ratio).z;
            }

            const auto bounds = make_foot_clearance_bounds(phase, ideal_clearance_m, policy);

            constraints.emplace_back(ground, bounds.minimum_clearance_m, bounds.maximum_clearance_m);
        }
    }

    return result;
}

}  // namespace hexapod_core
