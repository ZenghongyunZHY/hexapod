#include "hexapod_core/tripod_cycle_planner.hpp"
#include "hexapod_core/tripod_segment_constraints.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace hexapod_core {

namespace {

void require_cycle_request(const tripod_cycle_request& request) {
    if (!std::isfinite(request.initial_foot_tolerance_m) || request.initial_foot_tolerance_m < 0.0) {
        throw std::invalid_argument("Initial foot tolerance must be finite and nonnegative");
    }

    if (request.path_interval_count == 0) {
        throw std::invalid_argument("Cycle path checking requires at least one interval");
    }

    if (request.path_interval_count >= std::vector<leg_path_sample>{}.max_size()) {
        throw std::length_error("Cycle path interval count is too large");
    }

    if (request.clearance_policy) {
        require_foot_clearance_policy(*request.clearance_policy);

        const auto maximum_constraint_count = std::vector<leg_path_constraints>{}.max_size();

        if (request.path_interval_count >= maximum_constraint_count) {
            throw std::length_error("Cycle path constraint count is too large");
        }
    }

    for (const auto& limits : request.max_joint_speed_rad_s) {
        for (const double speed : limits) {
            if (!std::isfinite(speed) || speed <= 0.0) {
                throw std::invalid_argument("Cycle joint speed limits must be finite and positive");
            }
        }
    }

    for (const auto& angles : request.reference_angles) {
        if (!std::isfinite(angles.yaw_rad) || !std::isfinite(angles.upper_leg_rad) || !std::isfinite(angles.lower_leg_rad)) {
            throw std::invalid_argument("Initial reference angles must be finite");
        }
    }

    const auto& options = request.selection_options;

    if (options.knee_singularity != knee_singularity_policy::allow
        && options.knee_singularity != knee_singularity_policy::reject) {
        throw std::invalid_argument("Invalid knee singularity policy");
    }

    if (options.branch_continuity != branch_continuity_policy::nearest
        && options.branch_continuity != branch_continuity_policy::keep_reference_branch) {
        throw std::invalid_argument("Invalid branch continuity policy");
    }
}

tripod_foot_cycle plan_foot_cycle(const tripod_foot_cycle_request& request, const leg_kinematics& kinematics) {
    const auto& profile = request.walking_profile;
    if (!profile || !std::isfinite(profile->outward_offset_m) || profile->outward_offset_m <= 0.0) {
        return plan_tripod_foot_cycle(request);
    }

    std::array<vector3, leg_count> yaw_roots_in_body{};
    for (const auto leg : leg_order) {
        yaw_roots_in_body[leg_index(leg)] = kinematics.leg_to_body_frame(leg, {0.0, 0.0, 0.0});
    }
    return plan_tripod_foot_cycle(request, yaw_roots_in_body);
}

}  // namespace

tripod_cycle_planner::tripod_cycle_planner(robot_model model) : kinematics_(model), segment_planner_(std::move(model)) {}

tripod_cycle_result tripod_cycle_planner::plan(const tripod_cycle_request& request) const {
    require_cycle_request(request);

    // 先生成完整的候选脚点周期。
    auto geometry = plan_foot_cycle(request.foot_request, kinematics_);

    if (geometry.frames.size() < 2) {
        throw std::logic_error("Foot cycle must contain an initial frame and at least one target frame");
    }

    // 水平地面在机身坐标中的方程：
    // z_body + body_height - ground_height = 0。
    //
    // 当前机身高度不变，只有水平平移和偏航，
    // 所以该水平参考平面在整个周期中保持不变。
    const double ground_offset = request.foot_request.initial_pose.body_origin_in_planning_frame.z - request.foot_request.ground_height_m;

    if (!std::isfinite(ground_offset)) {
        throw std::overflow_error("Body-frame ground offset overflowed");
    }

    const ground_plane_in_body ground{{0.0, 0.0, 1.0}, ground_offset};

    // 当前整周期采用不低于规划地面的离散检查。
    require_path_constraints(leg_path_constraints{ground, 0.0});

    const auto& initial_points = geometry.frames.front().targets.points_in_body;

    // 先计算六腿初始误差，再判断是否接受。
    std::array<double, leg_count> initial_errors{};

    for (const auto leg : leg_order) {
        const auto index = leg_index(leg);

        const auto reference_point = kinematics_.forward_in_body_frame(leg, request.reference_angles[index]);

        const auto& target_point = initial_points[index];

        const double error = std::hypot(reference_point.x - target_point.x, reference_point.y - target_point.y, reference_point.z - target_point.z);

        if (!std::isfinite(error)) {
            throw std::overflow_error("Initial foot reference error is not finite");
        }

        initial_errors[index] = error;
    }

    for (const auto leg : leg_order) {
        const auto index = leg_index(leg);

        if (initial_errors[index] > request.initial_foot_tolerance_m) {
            tripod_cycle_result result{
                tripod_cycle_status::initial_reference_mismatch
            };

            result.initial_failed_leg = leg;
            result.initial_foot_error_m = initial_errors[index];

            return result;
        }
    }

    // 后续只推进这些局部规划参考。
    auto references = request.reference_angles;
    auto branches = request.remembered_branches;

    std::vector<body_ik_solution> segments;
    segments.reserve(geometry.frames.size() - 1);

    for (std::size_t index = 1; index < geometry.frames.size(); ++index) {
        const auto& previous = geometry.frames[index - 1];
        const auto& current = geometry.frames[index];

        const double duration_s = current.sample.time_s - previous.sample.time_s;

        if (!std::isfinite(duration_s) || duration_s <= 0.0) {
            throw std::logic_error("Foot cycle contains an invalid segment duration");
        }

        const body_ik_request target{
            current.targets.points_in_body,
            references,
            branches,
            request.selection_options,
            body_connection_request{
                duration_s,
                request.max_joint_speed_rad_s
            }
        };

        std::optional<body_path_constraint_samples> sample_constraints;

        if (request.clearance_policy) {
            sample_constraints.emplace(
                make_tripod_segment_constraints(
                    previous,
                    current,
                    request.foot_request.ground_height_m,
                    request.path_interval_count,
                    *request.clearance_policy
                )
            );
        }

        const body_segment_request segment_request{
            target,
            ground,
            {},  // 固定模式下，六腿下界均为 0。
            request.path_interval_count,
            {},  // 固定模式下，六腿均不设上界。
            std::move(sample_constraints)
        };

        auto planned = segment_planner_.plan(segment_request);


        switch (planned.status) {
            case body_segment_status::target_selection_failed:
            case body_segment_status::path_check_failed: {
                if (planned.solution) {
                    throw std::logic_error("Failed body segment unexpectedly contains a solution");
                }

                tripod_cycle_result result{tripod_cycle_status::segment_failed};

                result.failed_frame_index = index;
                result.segment_failure = std::move(planned);

                return result;
            }

            case body_segment_status::checks_passed:
                break;

            default:
                throw std::logic_error("Unexpected body segment status");
        }

        if (!planned.solution) {
            throw std::logic_error("Successful body segment has no solution");
        }

        segments.push_back(std::move(*planned.solution));

        // 本段通过后，才推进局部名义角和分支。
        references = segments.back().nominal_angles;
        branches = segments.back().branches;
    }

    // 所有运动段全部通过，才形成完整周期结果。
    tripod_cycle_result result{tripod_cycle_status::checks_passed};

    result.solution.emplace(std::move(geometry), std::move(segments));

    return result;
}

}  // namespace hexapod_core
