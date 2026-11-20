#pragma once

#include "hexapod_core/types.hpp"

namespace hexapod_core {

// 平地步态使用的规划机身位姿。
//
// 机身 Z 轴与规划参考系 Z 轴同向，
// 姿态只包含绕共同 +Z 轴的偏航。
//
// 这是规划输入，不表示来自定位或 IMU 的实测位姿。
struct planar_body_pose {
    // 机身原点在规划参考系中的位置，单位 m。
    // z 用于显式表达机身原点的高度。
    vector3 body_origin_in_planning_frame;

    // 从规划参考系到机身朝向的偏航角，单位 rad。
    // 从 +Z 方向看，逆时针为正。
    // yaw 为 0 时，机身轴与规划参考系轴同向。
    double yaw_rad;

    planar_body_pose(const vector3& body_origin,double yaw) : body_origin_in_planning_frame(body_origin), yaw_rad(yaw) {}
};

// 机身坐标脚点 → 规划参考系脚点。
// 输入与输出位置单位均为 m。
vector3 body_to_planning_frame(const planar_body_pose& pose, const vector3& point_in_body);

// 规划参考系脚点 → 机身坐标脚点。
// 输入与输出位置单位均为 m。
vector3 planning_to_body_frame(const planar_body_pose& pose, const vector3& point_in_planning_frame);

}  // namespace hexapod_core
