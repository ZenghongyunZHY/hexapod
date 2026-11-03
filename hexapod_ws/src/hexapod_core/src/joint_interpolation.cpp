#include "hexapod_core/joint_interpolation.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

namespace hexapod_core {

leg_joint_angles interpolate_leg_joint_angles(const leg_joint_angles& start, const leg_joint_angles& end, double ratio) {
    if (!std::isfinite(ratio) || ratio < 0.0 || ratio > 1.0) {
        throw std::invalid_argument(
            "Interpolation ratio must be finite and within [0, 1]");
    }

    const std::array<double, 6> input_angles{
        start.yaw_rad,
        start.upper_leg_rad,
        start.lower_leg_rad,
        end.yaw_rad,
        end.upper_leg_rad,
        end.lower_leg_rad
    };

    for (const double angle : input_angles) {
        if (!std::isfinite(angle)) {
            throw std::invalid_argument("Interpolation angles must be finite radians");
        }
    }

    // 明确返回原始端点，避免端点受到额外计算影响。
    if (ratio == 0.0) {
        return start;
    }

    if (ratio == 1.0) {
        return end;
    }

    const double start_weight = 1.0 - ratio;

    const leg_joint_angles result{ start_weight * start.yaw_rad + ratio * end.yaw_rad,

        start_weight * start.upper_leg_rad + ratio * end.upper_leg_rad,

        start_weight * start.lower_leg_rad + ratio * end.lower_leg_rad
    };

    if (!std::isfinite(result.yaw_rad) || !std::isfinite(result.upper_leg_rad) || !std::isfinite(result.lower_leg_rad)) {
        throw std::overflow_error("Interpolated joint angles are not finite");
    }

    return result;
}

}  // namespace hexapod_core
