#include "hexapod_core/foot_trajectory.hpp"

#include <cmath>
#include <stdexcept>

namespace hexapod_core {

namespace {

void require_finite_vector(const vector3& value, const char* message) {
    if (!std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z)) {
        throw std::invalid_argument(message);
    }
}

//p(s)=(1-u(s))p_start+u(s)p_end+ H b(s)n

//u(s)=10s^3-15s^4+6s^5

//b(s)=64s^3(1-s)^3


// 五次平滑进度。
// 调用前已经保证 ratio 位于 [0, 1]。
double quintic_progress(double ratio) {
    if (ratio <= 0.5) {
        const double ratio_squared = ratio * ratio;
        const double ratio_cubed = ratio_squared * ratio;

        return ratio_cubed * (10.0 + ratio * (-15.0 + 6.0 * ratio));
    }

    // 利用 u(s) = 1 - u(1-s)，改善接近终点时的数值计算。
    // double 精度有限，这样可减少接近终点时多项式相加减带来的舍入影响。
    const double remaining = 1.0 - ratio;
    const double remaining_squared = remaining * remaining;
    const double remaining_cubed = remaining_squared * remaining;

    return 1.0 - remaining_cubed * (10.0 + remaining * (-15.0 + 6.0 * remaining));
}

}  // namespace

vector3 sample_swing_trajectory(const swing_trajectory_request& request,double ratio) {

    if (!std::isfinite(ratio) || ratio < 0.0 || ratio > 1.0) {
        throw std::invalid_argument("Swing ratio must be finite and within [0, 1]");
    }

    require_finite_vector(request.start_in_planning_frame, "Swing start point must be finite");

    require_finite_vector(request.end_in_planning_frame, "Swing end point must be finite");

    require_finite_vector(request.lift_direction_in_planning_frame, "Swing lift direction must be finite");

    const auto& direction = request.lift_direction_in_planning_frame;

    const double direction_length = std::hypot(direction.x, direction.y, direction.z);

    // 数值容差，不是方向安装误差或机械余量。
    constexpr double unit_direction_tolerance = 1e-9;

    if (!std::isfinite(direction_length) || std::abs(direction_length - 1.0) > unit_direction_tolerance) {
        throw std::invalid_argument("Swing lift direction must have unit length");
    }

    if (!std::isfinite(request.lift_height_m) || request.lift_height_m < 0.0) {
        throw std::invalid_argument("Swing lift height must be finite and nonnegative");
    }

    // 校验完整请求后，再精确返回端点。
    if (ratio == 0.0) {
        return request.start_in_planning_frame;
    }

    if (ratio == 1.0) {
        return request.end_in_planning_frame;
    }

    const double progress = quintic_progress(ratio);//u(s)
    const double start_weight = 1.0 - progress;//1-u(s)

    const double arch = ratio * (1.0 - ratio);
    const double lift_fraction = 64.0 * arch * arch * arch;//64* s^3 (1-s)^3

    const double lift_distance_m = request.lift_height_m * lift_fraction;

    const auto& start = request.start_in_planning_frame;
    const auto& end = request.end_in_planning_frame;

    const vector3 result{
        start_weight * start.x + progress * end.x + lift_distance_m * direction.x,

        start_weight * start.y + progress * end.y + lift_distance_m * direction.y,

        start_weight * start.z + progress * end.z + lift_distance_m * direction.z
    };

    if (!std::isfinite(result.x) || !std::isfinite(result.y) || !std::isfinite(result.z)) {
        throw std::overflow_error("Computed swing target is not finite");
    }

    return result;
}

}  // namespace hexapod_core
