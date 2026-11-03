#include "hexapod_core/leg_path_sampler.hpp"

#include <stdexcept>
#include <utility>

namespace hexapod_core {

leg_path_sampler::leg_path_sampler(robot_model model)
    : kinematics_(std::move(model)) {
}

std::vector<leg_path_sample> leg_path_sampler::sample_linear_segment(leg_id leg, const leg_joint_angles& start_nominal, const leg_joint_angles& end_nominal, std::size_t interval_count) const {
    if (interval_count == 0) {
        throw std::invalid_argument("Path sampling requires at least one interval");
    }

    std::vector<leg_path_sample> samples;

    // 后面需要保存 interval_count + 1 个点。
    // 先检查容量边界，也避免加一溢出。
    if (interval_count >= samples.max_size()) {
        throw std::length_error("Path sampling interval count is too large");
    }

    // 插值函数会检查两个端点的全部角度。
    const auto first_angles = interpolate_leg_joint_angles(start_nominal, end_nominal, 0.0);

    // FK 同时检查腿编号和该腿所需的安装信息。
    const auto first_point = kinematics_.forward_in_body_frame(leg, first_angles);


    //先计算起点的原因是，通过检查起终点，使得 如果输入本身存在问题，能在申请大量存储空间前报告
    samples.reserve(interval_count + 1);

    samples.push_back(leg_path_sample{
        0.0,
        first_angles,
        first_point
    });

    for (std::size_t index = 1; index <= interval_count; ++index) {
        const double ratio = index == interval_count ? 1.0 : static_cast<double>(index) / static_cast<double>(interval_count);

        const auto angles = interpolate_leg_joint_angles(start_nominal, end_nominal, ratio);

        const auto point = kinematics_.forward_in_body_frame(leg, angles);

        samples.push_back(leg_path_sample{
            ratio,
            angles,
            point
        });
    }

    return samples;
}

}  // namespace hexapod_core
