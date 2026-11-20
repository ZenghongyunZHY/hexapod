#include "hexapod_core/body_pose.hpp"

#include <cmath>
#include <stdexcept>

namespace hexapod_core {

namespace {

void require_finite_point(const vector3& point, const char* message) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        throw std::invalid_argument(message);
    }
}

void require_body_pose(const planar_body_pose& pose) {
    require_finite_point(pose.body_origin_in_planning_frame, "Body origin must contain finite coordinates");
    if (!std::isfinite(pose.yaw_rad)) {
        throw std::invalid_argument("Body yaw must be finite");
    }
}

// 有限输入也可能在减法、乘法或加法中溢出。
// 计算结果无法表示时明确拒绝，不返回无穷坐标。
void require_finite_result(const vector3& point, const char* message) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        throw std::overflow_error(message);
    }
}

}  // namespace

// 输入机身在 planning frame 中的位姿，
// 以及某个点在 body frame 中的坐标。
// 返回同一个点在 planning frame 中的坐标。
//
// 先按机身偏航角旋转，再加上机身原点的位置：
// p_planning = Rz(yaw) * p_body + body_origin。
vector3 body_to_planning_frame(const planar_body_pose& pose,const vector3& point_in_body) {

    require_body_pose(pose);

    require_finite_point(point_in_body, "Body-frame point must contain finite coordinates");

    const double cosine = std::cos(pose.yaw_rad);
    const double sine = std::sin(pose.yaw_rad);

    const auto& origin = pose.body_origin_in_planning_frame;

    // 先绕 +Z 旋转，再加上机身原点的位置：
    //
    // x_p = cos(yaw) * x_b - sin(yaw) * y_b + origin.x
    // y_p = sin(yaw) * x_b + cos(yaw) * y_b + origin.y
    // z_p = z_b + origin.z
    const vector3 result{
        cosine * point_in_body.x - sine * point_in_body.y + origin.x,

        sine * point_in_body.x + cosine * point_in_body.y + origin.y,

        point_in_body.z + origin.z
    };

    require_finite_result(result, "Body-to-planning transform overflowed");

    return result;
}

vector3 planning_to_body_frame(const planar_body_pose& pose, const vector3& point_in_planning_frame) {

    require_body_pose(pose);

    require_finite_point(point_in_planning_frame, "Planning-frame point must contain finite coordinates");

    const auto& origin = pose.body_origin_in_planning_frame;

    // 先减去机身原点，得到规划参考系中的相对向量。
    const vector3 relative{
        point_in_planning_frame.x - origin.x,
        point_in_planning_frame.y - origin.y,
        point_in_planning_frame.z - origin.z
    };

    require_finite_result(relative, "Planning-to-body relative coordinates overflowed");

    const double cosine = std::cos(pose.yaw_rad);
    const double sine = std::sin(pose.yaw_rad);

    // 再乘 Rz(yaw) 的转置：
    //
    // x_b =  cos(yaw) * relative.x + sin(yaw) * relative.y
    // y_b = -sin(yaw) * relative.x + cos(yaw) * relative.y
    // z_b = relative.z
    const vector3 result{
        cosine * relative.x + sine * relative.y,

        -sine * relative.x + cosine * relative.y,

        relative.z
    };

    require_finite_result(
        result,
        "Planning-to-body transform overflowed");

    return result;
}

}  // namespace hexapod_core
