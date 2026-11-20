#pragma once

#include "hexapod_core/body_pose.hpp"

namespace hexapod_core {

// 在运动段内保持恒定的机身坐标速度。
// 这些是规划输入，不表示实测速度或设备能力上限。
struct planar_body_velocity {
    // 沿机身 +X 向前的速度，单位 m/s。
    double vx_m_s;

    // 沿机身 +Y 向左的速度，单位 m/s。
    double vy_m_s;

    // 绕共同 +Z 轴的偏航角速度，单位 rad/s。
    double wz_rad_s;

    planar_body_velocity(double vx, double vy, double wz) : vx_m_s(vx), vy_m_s(vy), wz_rad_s(wz) {}
};

// 在恒定机身坐标速度下，推进规划机身位姿。
//
// duration_s 必须有限且不小于 0。
// 初始位姿和三个速度必须有限。
// duration_s 为 0 时，校验全部输入后返回原位姿。
//
// 保持机身原点高度不变，仅推进 X/Y 和偏航。
// 偏航角连续累积，不强制归一化到 [-pi, pi]。
// 不修改输入，不限速，不检查 IK 或执行能力。
//
// 非法输入抛 std::invalid_argument。
// 中间运算或结果无法表示时抛 std::overflow_error。
planar_body_pose advance_body_pose(const planar_body_pose& initial_pose, const planar_body_velocity& velocity, double duration_s);

}  // namespace hexapod_core
