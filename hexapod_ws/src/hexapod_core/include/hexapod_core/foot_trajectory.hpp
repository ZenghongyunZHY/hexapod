#pragma once

#include "hexapod_core/types.hpp"

namespace hexapod_core {

// 一次脚端摆动曲线的几何输入。
// 三个向量必须在同一个、由调用方明确选定的规划坐标系中表达。
struct swing_trajectory_request {
    // 起终点位置，单位 m。
    vector3 start_in_planning_frame;
    vector3 end_in_planning_frame;

    // 抬升方向：有限单位向量。
    vector3 lift_direction_in_planning_frame;

    // 相对平滑基线的最大额外抬升量，单位 m。
    // 必须有限且不小于 0。
    double lift_height_m;

    swing_trajectory_request(
        const vector3& start,
        const vector3& end,
        const vector3& lift_direction,
        double lift_height
    )
        : start_in_planning_frame(start),
          end_in_planning_frame(end),
          lift_direction_in_planning_frame(lift_direction),
          lift_height_m(lift_height) {
    }
};

// 根据 [0,1] 内的进度，计算规划坐标中的脚端目标。
// 只进行几何计算，不检查 IK、关节约束或执行能力。
vector3 sample_swing_trajectory(
    const swing_trajectory_request& request,
    double ratio
);

}  // namespace hexapod_core
