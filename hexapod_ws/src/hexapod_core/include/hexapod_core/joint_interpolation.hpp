#pragma once

#include "hexapod_core/leg_kinematics.hpp"

namespace hexapod_core {

// 在两个单腿关节姿态之间进行线性插值。
// 角度单位为 rad，ratio 必须在 [0, 1] 内。
// 使用普通角度插值，不进行整圈归一化。
// 本函数不检查安装限位、脚端路径或实际执行能力。
leg_joint_angles interpolate_leg_joint_angles(
    const leg_joint_angles& start,
    const leg_joint_angles& end,
    double ratio
);

}  // namespace hexapod_core
