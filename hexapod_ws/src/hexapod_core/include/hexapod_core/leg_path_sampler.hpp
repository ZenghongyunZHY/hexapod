#pragma once

#include "hexapod_core/joint_interpolation.hpp"

#include <cstddef>
#include <vector>

namespace hexapod_core {

// 一条线性关节运动段上的离线采样记录。
struct leg_path_sample {
    // 该点在运动段中的比例，范围 [0, 1]。
    double ratio;

    // 由起终点名义角插值得到的连续关节角，单位 rad。
    // 不是重新取整后的命令，也不是舵机回读。
    leg_joint_angles angles;

    // 对应脚端在机身坐标系中的位置，单位 m。
    vector3 point_in_body;
};


class leg_path_sampler {
 public:
    explicit leg_path_sampler(robot_model model);

    // 对一个线性关节段均匀采样。
    // interval_count 为区间数，返回 interval_count + 1 个点。
    // 起终点应由调用方提供有效的名义关节角。
    // 本函数只生成采样记录，不进行执行放行。
    std::vector<leg_path_sample> sample_linear_segment(leg_id leg, const leg_joint_angles& start_nominal, const leg_joint_angles& end_nominal, std::size_t interval_count) const;

 private:
    leg_kinematics kinematics_;
};

}  // namespace hexapod_core

