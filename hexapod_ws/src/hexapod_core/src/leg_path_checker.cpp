#include "hexapod_core/leg_path_checker.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace hexapod_core {

void require_path_constraints(const leg_path_constraints& constraints) {
    //法向量
    const auto& normal = constraints.ground.unit_normal;

    if (!std::isfinite(normal.x) || !std::isfinite(normal.y) || !std::isfinite(normal.z)) {
        throw std::invalid_argument("Ground plane normal must be finite");
    }

    const double normal_length = std::hypot(normal.x, normal.y, normal.z);

    // 仅用于判断单位向量的数值误差，不是机械误差余量。
    constexpr double unit_normal_tolerance = 1e-9;

    if (!std::isfinite(normal_length) || std::abs(normal_length - 1.0) > unit_normal_tolerance) {
        throw std::invalid_argument("Ground plane normal must have unit length");
    }

    if (!std::isfinite(constraints.ground.offset_m)) {
        throw std::invalid_argument("Ground plane offset must be finite");
    }

    if (!std::isfinite(constraints.minimum_clearance_m) || constraints.minimum_clearance_m < 0.0) {
        throw std::invalid_argument("Minimum clearance must be finite and nonnegative");
    }
}

namespace {

//计算有符号离面距离
double signed_clearance(const ground_plane_in_body& plane, const vector3& point) {

    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        throw std::overflow_error("Sample foot coordinates are not finite");
    }

    const auto& normal = plane.unit_normal;

    const double clearance_m = normal.x * point.x + normal.y * point.y + normal.z * point.z + plane.offset_m;

    if (!std::isfinite(clearance_m)) {
        throw std::overflow_error("Computed foot clearance is not finite");
    }

    return clearance_m;
}

}  // namespace

leg_path_checker::leg_path_checker(robot_model model) : model_(std::move(model)), sampler_(model_) {}

leg_path_check_result leg_path_checker::check_linear_segment(
    leg_id leg,
    const leg_joint_angles& start_nominal,
    const leg_joint_angles& end_nominal,
    std::size_t interval_count,
    const leg_path_constraints& constraints //约束
) const {

    require_path_constraints(constraints);

    // 先取得三个关节的限位，配置不完整时明确报错。
    std::array<angle_limits, joints_per_leg> limits{};

    for (std::size_t joint_offset = 0; joint_offset < joints_per_leg; ++joint_offset) {

        const auto& joint = model_.joint(leg, joint_order[joint_offset]);

        if (!model_.is_command_ready(joint.binding.id)) {
            throw std::logic_error("Path checking requires complete joint calibration");
        }

        limits[joint_offset] = *joint.calibration.installed_limits;
    }

    const auto samples = sampler_.sample_linear_segment(leg, start_nominal, end_nominal, interval_count);

    // 与当前 mapper、参考角检查采用的数值尺度保持一致。
    // 只吸收浮点误差，不表示机械限位余量。
    constexpr double angle_numerical_tolerance = 1e-12;

    for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {

        const auto& sample = samples[sample_index];

        const std::array<double, joints_per_leg> angles{sample.angles.yaw_rad, sample.angles.upper_leg_rad, sample.angles.lower_leg_rad};

        // 同一个采样点内，先检查三个关节。
        for (std::size_t joint_offset = 0; joint_offset < joints_per_leg; ++joint_offset) {
            const auto& joint_limits = limits[joint_offset];

            const bool below_minimum = angles[joint_offset] < joint_limits.minimum_rad - angle_numerical_tolerance;

            const bool above_maximum = angles[joint_offset] > joint_limits.maximum_rad + angle_numerical_tolerance;

            if (below_minimum || above_maximum) {
                leg_path_check_result result{leg_path_check_status::joint_limit_exceeded, sample_index + 1};

                result.failed_sample_index = sample_index;
                result.failed_sample = sample;
                result.failed_joint = joint_order[joint_offset];

                return result;
            }
        }

        // 关节角通过后，再检查脚端离面距离。
        const double clearance_m = signed_clearance(constraints.ground, sample.point_in_body);

        if (clearance_m < constraints.minimum_clearance_m) {
            leg_path_check_result result{leg_path_check_status::foot_clearance_below_minimum, sample_index + 1};

            result.failed_sample_index = sample_index;
            result.failed_sample = sample;
            result.failed_clearance_m = clearance_m;

            return result;
        }
    }

    return leg_path_check_result{leg_path_check_status::samples_passed, samples.size()};
}

}  // namespace hexapod_core
