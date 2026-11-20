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

    if (!std::isfinite(constraints.minimum_clearance_m)) {
        throw std::invalid_argument("Minimum clearance must be finite");
    }

    if (constraints.maximum_clearance_m) {
        const double maximum = *constraints.maximum_clearance_m;

        if (!std::isfinite(maximum)) {
            throw std::invalid_argument("Maximum clearance must be finite");
        }

        if (maximum < constraints.minimum_clearance_m) {
            throw std::invalid_argument("Maximum clearance must not be below minimum clearance");
        }
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
    const leg_path_constraints& constraints
) const {
    require_path_constraints(constraints);

    const auto samples = sampler_.sample_linear_segment(
        leg,
        start_nominal,
        end_nominal,
        interval_count
    );

    // 固定约束入口：为每个采样点复制同一份约束。
    const std::vector<leg_path_constraints> sample_constraints(
        samples.size(),
        constraints
    );

    return check_samples(
        leg,
        samples,
        sample_constraints
    );
}


leg_path_check_result
leg_path_checker::check_linear_segment_with_sample_constraints(
    leg_id leg,
    const leg_joint_angles& start_nominal,
    const leg_joint_angles& end_nominal,
    std::size_t interval_count,
    const std::vector<leg_path_constraints>& sample_constraints
) const {
    if (interval_count == 0) {
        throw std::invalid_argument(
            "Path checking requires at least one interval"
        );
    }

    const auto maximum_sample_count =
        std::vector<leg_path_sample>{}.max_size();

    if (interval_count >= maximum_sample_count) {
        throw std::length_error(
            "Path checking interval count is too large"
        );
    }

    // 使用减法形式核对数量，避免直接计算 interval_count + 1。
    if (sample_constraints.empty()
        || sample_constraints.size() - 1 != interval_count) {
        throw std::invalid_argument(
            "Path constraint count must match path sample count"
        );
    }

    // 先检查整份输入。
    // 即使前面的路径点会失败，也不能忽略后面的非法约束。
    for (const auto& constraints : sample_constraints) {
        require_path_constraints(constraints);
    }

    const auto samples = sampler_.sample_linear_segment(
        leg,
        start_nominal,
        end_nominal,
        interval_count
    );

    return check_samples(
        leg,
        samples,
        sample_constraints
    );
}


leg_path_check_result leg_path_checker::check_samples(leg_id leg, const std::vector<leg_path_sample>& samples, const std::vector<leg_path_constraints>& sample_constraints) const {
    // 私有函数的调用关系应当保证这一条件。
    if (samples.empty() || samples.size() != sample_constraints.size()) {
        throw std::logic_error( "Path samples and constraints are inconsistent");
    }

    std::array<angle_limits, joints_per_leg> limits{};

    for (std::size_t joint_offset = 0; joint_offset < joints_per_leg; ++joint_offset) {
        const auto& joint = model_.joint(leg, joint_order[joint_offset]);

        if (!model_.is_command_ready(joint.binding.id)) {
            throw std::logic_error("Path checking requires complete joint calibration");
        }

        limits[joint_offset] = *joint.calibration.installed_limits;
    }

    // 仅吸收浮点计算误差，不表示机械限位余量。
    constexpr double angle_numerical_tolerance = 1e-12;

    for (std::size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
        const auto& sample = samples[sample_index];

        // 本次最关键的变化：
        // 使用与当前采样点一一对应的约束。
        const auto& constraints = sample_constraints[sample_index];

        const std::array<double, joints_per_leg> angles{
            sample.angles.yaw_rad,
            sample.angles.upper_leg_rad,
            sample.angles.lower_leg_rad
        };

        // 同一采样点内，先检查三个关节安装限位。
        for (std::size_t joint_offset = 0;
             joint_offset < joints_per_leg;
             ++joint_offset) {
            const auto& joint_limits = limits[joint_offset];

            const bool below_minimum = angles[joint_offset] < joint_limits.minimum_rad - angle_numerical_tolerance;

            const bool above_maximum = angles[joint_offset] > joint_limits.maximum_rad + angle_numerical_tolerance;

            if (below_minimum || above_maximum) {
                leg_path_check_result result{
                    leg_path_check_status::joint_limit_exceeded,
                    sample_index + 1
                };

                result.failed_sample_index = sample_index;
                result.failed_sample = sample;
                result.failed_joint = joint_order[joint_offset];

                return result;
            }
        }

        const double clearance_m = signed_clearance(
            constraints.ground,
            sample.point_in_body
        );

        if (clearance_m < constraints.minimum_clearance_m) {
            leg_path_check_result result{leg_path_check_status::foot_clearance_below_minimum, sample_index + 1};

            result.failed_sample_index = sample_index;
            result.failed_sample = sample;
            result.failed_clearance_m = clearance_m;

            return result;
        }

        if (constraints.maximum_clearance_m && clearance_m > *constraints.maximum_clearance_m) {
            
            leg_path_check_result result{leg_path_check_status::foot_clearance_above_maximum, sample_index + 1};

            result.failed_sample_index = sample_index;
            result.failed_sample = sample;
            result.failed_clearance_m = clearance_m;

            return result;
        }
    }

    return leg_path_check_result{
        leg_path_check_status::samples_passed,
        samples.size()
    };
}

}  // namespace hexapod_core
