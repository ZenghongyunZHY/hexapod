#pragma once

#include "hexapod_core/body_motion.hpp"

namespace hexapod_core {

// 本次摆动结束后的下一支撑期中点，
// 脚在机身坐标中的水平参考位置，单位 m。
// 中点指支撑时间中点；直线匀速时也是支撑行程中点。
struct stance_midpoint_reference {
    double x_m;
    double y_m;
};

// 起脚时，用于确定本次固定摆动终点的请求。
struct swing_landing_request {
    // 起脚时刻的规划机身位姿。
    planar_body_pose lift_off_pose;

    // 预测到下一支撑期中点时，假设保持恒定的机身坐标速度。
    planar_body_velocity velocity;

    // 下一支撑期中点，期望脚在机身坐标中的水平位置。
    stance_midpoint_reference midpoint_reference_in_body;

    // 本次完整摆动时长，必须有限且为正。
    double swing_duration_s;

    // 本次摆动结束后的完整支撑时长，必须有限且为正。
    double next_stance_duration_s;

    // 规划参考系中的水平地面高度。
    // 落脚点满足 z = ground_height_m。
    double ground_height_m;

    swing_landing_request(
        const planar_body_pose& pose,
        const planar_body_velocity& body_velocity,
        const stance_midpoint_reference& midpoint_reference,
        double swing_duration,
        double stance_duration,
        double ground_height
    )
        : lift_off_pose(pose),
          velocity(body_velocity),
          midpoint_reference_in_body(midpoint_reference),
          swing_duration_s(swing_duration),
          next_stance_duration_s(stance_duration),
          ground_height_m(ground_height) {
    }
};

// 返回规划参考系中的候选落脚点。
//
// 预测 T_swing + T_stance/2 后的机身位姿，
// 将下一支撑期中点的水平参考脚位转换到规划参考系，
// 再使用显式地面高度设置 Z。
//
// 上层起脚时调用，并保存结果作为固定摆动终点。
// 不修改请求，不进行 IK、路径检查或命令发送。
//
// 非法输入抛 std::invalid_argument。
// 积分或坐标变换溢出时抛 std::overflow_error。
vector3 plan_swing_landing_point(
    const swing_landing_request& request
);

}  // namespace hexapod_core
