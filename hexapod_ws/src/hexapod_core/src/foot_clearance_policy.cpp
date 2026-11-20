#include "hexapod_core/foot_clearance_policy.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hexapod_core {

void require_foot_clearance_policy(const foot_clearance_policy& policy) {
    if (!std::isfinite(policy.contact_below_tolerance_m)
        || policy.contact_below_tolerance_m < 0.0) {
        throw std::invalid_argument("Contact below tolerance must be finite and nonnegative");
    }

    if (!std::isfinite(policy.contact_above_tolerance_m)
        || policy.contact_above_tolerance_m < 0.0) {
        throw std::invalid_argument("Contact above tolerance must be finite and nonnegative");
    }

    if (!std::isfinite(policy.swing_height_error_m)
        || policy.swing_height_error_m < 0.0) {
        throw std::invalid_argument("Swing height error must be finite and nonnegative");
    }
}

//double ideal_clearance_m 表示：当前时刻，规划的理想脚点到参考地面的有符号距离，单位是米。
//输入当前腿的阶段、理想脚点离地高度、误差预算，返回这个时刻允许的脚点离地距离上下界。
foot_clearance_bounds make_foot_clearance_bounds(const leg_phase_state& phase, double ideal_clearance_m, const foot_clearance_policy& policy) {
    require_foot_clearance_policy(policy);

    if (!std::isfinite(phase.phase_ratio)
        || phase.phase_ratio < 0.0
        || phase.phase_ratio > 1.0) {
        throw std::invalid_argument("Leg phase ratio must be finite and within [0, 1]");
    }

    if (!std::isfinite(ideal_clearance_m)
        || ideal_clearance_m < 0.0) {
        throw std::invalid_argument("Ideal clearance must be finite and nonnegative");
    }

    bool uses_contact_bounds = false;

    switch (phase.phase) {
        case leg_phase::stance:
            uses_contact_bounds = true;
            break;

        case leg_phase::swing:
            uses_contact_bounds =
                phase.phase_ratio == 0.0
                || phase.phase_ratio == 1.0;
            break;

        default:
            throw std::invalid_argument("Unknown leg phase");
    }

    const double contact_minimum = -policy.contact_below_tolerance_m;

    if (uses_contact_bounds) {
        return foot_clearance_bounds{
            contact_minimum,
            policy.contact_above_tolerance_m
        };
    }

    const double swing_minimum = std::max(
        contact_minimum,
        ideal_clearance_m - policy.swing_height_error_m
    );

    return foot_clearance_bounds{
        swing_minimum
    };
}

}  // namespace hexapod_core
