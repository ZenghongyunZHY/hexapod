#include "hexapod_core/tripod_foot_cycle.hpp"

#include <cmath>
#include <stdexcept>
#include <array>
#include <optional>

namespace hexapod_core {

namespace {

void require_foot_cycle_request(const tripod_foot_cycle_request& request) {
    if (!std::isfinite(request.ground_height_m)) {
        throw std::invalid_argument("Planning ground height must be finite");
    }

    if (!std::isfinite(request.lift_height_m) || request.lift_height_m < 0.0) {
        throw std::invalid_argument("Lift height must be finite and nonnegative");
    }

    for (const auto leg : leg_order) {
        const auto index = leg_index(leg);
        const auto& reference = request.midpoint_references_in_body[index];

        if (!std::isfinite(reference.x_m) || !std::isfinite(reference.y_m)) {
            throw std::invalid_argument("All stance midpoint references must be finite");
        }

        if (request.initial_feet.swings()[index]) {
            throw std::invalid_argument("Cycle must begin without an active swing");
        }
    }

    // 零时长积分也会校验初始位姿和速度。
    (void)advance_body_pose(request.initial_pose, request.velocity, 0.0);
}

}  // namespace

tripod_foot_cycle plan_tripod_foot_cycle(const tripod_foot_cycle_request& request) {
    require_foot_cycle_request(request);

    //返回每个time_s对应的ratio和周期阶段
    const auto samples = make_tripod_cycle_samples(request.timing, request.intervals_per_stage);

    const double stance_duration_s = request.timing.period_s * request.timing.stance_ratio;
    const double swing_duration_s = request.timing.period_s * (1.0 - request.timing.stance_ratio);

    auto feet = request.initial_feet;

    std::vector<tripod_foot_frame> frames;

    frames.reserve(samples.size());

    for (const auto& sample : samples) {
        // 每次从相同周期起点预测，避免逐小段累积时间误差。
        const auto pose = advance_body_pose(request.initial_pose, request.velocity, sample.time_s);

        // 周期终点不能直接传给只接受 [0,1) 的相位函数。
        const double phase_ratio = sample.cycle_ratio == 1.0 ? 0.0 : sample.cycle_ratio;

        //返回改时刻所处于的阶段和阶段内进度(swing or stance)
        auto phases = sample_tripod_phase({phase_ratio, request.timing.stance_ratio});

        if (sample.boundary) {
            const auto& event = *sample.boundary;

            for (const auto leg : leg_order) {
                const auto index = leg_index(leg);

                if (event.begin_swing[index]) {
                    const swing_landing_request landing_request{
                        pose,
                        request.velocity,
                        request.midpoint_references_in_body[index],
                        swing_duration_s,
                        stance_duration_s,
                        request.ground_height_m
                    };

                    //landing为该腿在planning frame中的下一支撑期中点
                    //在支撑期内，支撑点在planning frame中的坐标是不变的
                    //所以可以使用它作为摆动曲线参数的落点
                    const auto landing = plan_swing_landing_point(landing_request);

                    //不断更新swings_[index]成员
                    feet = feet.begin_swing(leg, landing, {0.0, 0.0, 1.0}, request.lift_height_m);

                    phases[index] = {leg_phase::swing, 0.0};
                }

                if (event.finish_swing[index]) {
                    // 先采样旧摆动终点，稍后才保存新支撑点。
                    phases[index] = {leg_phase::swing, 1.0};
                }
            }

            if (event.cycle_complete && request.timing.stance_ratio == 0.5) {
                // 半占空比下，B 组在周期末完成支撑。
                // 此处不能用回绕相位提前开始下一次 B 组摆动。
                for (const auto leg : leg_order) {
                    const auto index = leg_index(leg);

                    if (!event.finish_swing[index]) {
                        phases[index] = {leg_phase::stance, 1.0};
                    }
                }
            }
        }

        // 先计算当前时刻的脚点。
        // 落脚腿此时仍使用旧摆动曲线的进度 1。
        const auto targets = sample_body_foot_targets({pose, phases, feet.stance_points(), feet.swings()});

        if (sample.boundary) {
            // 当前时刻的终点已经计算出来，
            // 再完成计划落脚，更新局部候选状态。
            for (const auto leg : leg_order) {
                const auto index = leg_index(leg);

                if (sample.boundary->finish_swing[index]) {
                    feet = feet.finish_swing(leg);
                }
            }
        }

        // phases/targets 描述当前时刻；
        // feet.swings() 描述离开本帧后的摆动状态。
        frames.emplace_back(sample, pose, phases, targets, feet.swings());
    }

    for (const auto& swing : feet.swings()) {
        if (swing) {
            throw std::logic_error("Completed foot cycle still contains an active swing");
        }
    }

    const auto final_pose = frames.back().body_pose;

    return tripod_foot_cycle{std::move(frames), feet, final_pose};
}

}  // namespace hexapod_core
