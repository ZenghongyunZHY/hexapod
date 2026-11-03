#include "hexapod_core/leg_candidate_selector.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace hexapod_core {

leg_candidate_selector::leg_candidate_selector(robot_model model) : model_(std::move(model)), mapper_(model_) {
}

double leg_candidate_selector::joint_distance_cost(const leg_joint_angles& target_angles, const leg_joint_angles& reference_angles) {

    // 目标和参考都必须由有限角度组成。
    if (!std::isfinite(target_angles.yaw_rad)
        || !std::isfinite(target_angles.upper_leg_rad)
        || !std::isfinite(target_angles.lower_leg_rad)
        || !std::isfinite(reference_angles.yaw_rad)
        || !std::isfinite(reference_angles.upper_leg_rad)
        || !std::isfinite(reference_angles.lower_leg_rad)) {
        throw std::invalid_argument(
            "Joint distance requires finite angles");
    }

    // 有限行程关节使用普通角差，不进行整圈归一化。
    const double yaw_delta_rad = target_angles.yaw_rad - reference_angles.yaw_rad;

    const double upper_delta_rad = target_angles.upper_leg_rad - reference_angles.upper_leg_rad;

    const double lower_delta_rad = target_angles.lower_leg_rad - reference_angles.lower_leg_rad;

    const double cost = yaw_delta_rad * yaw_delta_rad + upper_delta_rad * upper_delta_rad + lower_delta_rad * lower_delta_rad;

    // 有限输入在极端数值下也可能使减法或平方溢出。
    if (!std::isfinite(cost)) {
        throw std::invalid_argument("Joint distance cost is not finite");
    }

    return cost;
}

std::optional<leg_mapped_candidate> leg_candidate_selector::map_candidate(leg_id leg, const leg_ik_candidate& candidate) const {

    const auto& angles = candidate.angles;

    // 非法数值属于输入错误，不能当作普通超限淘汰。
    if (!std::isfinite(angles.yaw_rad)
        || !std::isfinite(angles.upper_leg_rad)
        || !std::isfinite(angles.lower_leg_rad)) {
        throw std::invalid_argument("IK candidate angles must be finite radians");
    }

    // 通过模型查询实际绑定，不根据逻辑下标猜舵机 ID。
    const std::array<servo_id, joints_per_leg> ids{
        model_.joint(leg, joint_type::body_yaw).binding.id,
        model_.joint(leg, joint_type::upper_leg_pitch).binding.id,
        model_.joint(leg, joint_type::lower_leg_pitch).binding.id
    };

    // 先检查整条腿的命令配置，防止缺失标定被超限结果掩盖。
    for (const auto id : ids) {
        if (!model_.is_command_ready(id)) {
            throw std::logic_error("Leg command mapping is not ready");
        }
    }

    try {
        // 三轴全部转换成功，才形成完整的位置数组。
        const std::array<servo_position, joints_per_leg> positions{
            mapper_.joint_angle_rad_to_servo_position(
                ids[0], angles.yaw_rad),

            mapper_.joint_angle_rad_to_servo_position(
                ids[1], angles.upper_leg_rad),

            mapper_.joint_angle_rad_to_servo_position(
                ids[2], angles.lower_leg_rad)
        };

        // 纯数学解码，不读取舵机反馈。
        const leg_joint_angles nominal_angles{
            mapper_.servo_position_to_joint_angle_rad(
                ids[0], positions[0]),

            mapper_.servo_position_to_joint_angle_rad(
                ids[1], positions[1]),

            mapper_.servo_position_to_joint_angle_rad(
                ids[2], positions[2])
        };

        return leg_mapped_candidate{
            candidate,
            positions,
            nominal_angles
        };
    }
    catch (const std::out_of_range&) {
        // 当前候选不能通过映射范围检查，由调用方继续检查其他候选。
        return std::nullopt;
    }
}

leg_filter_result leg_candidate_selector::filter_candidates(leg_id leg, const leg_ik_result& ik_result) const {

    // 即使没有几何候选，也不能接受非法腿编号。
    (void)model_.leg(leg);

    // 先检查上游几何结果，不能把不同失败原因混在一起。
    switch (ik_result.status) {
        case leg_ik_status::unreachable:
            if (!ik_result.solution.empty()) {
                throw std::invalid_argument(
                    "Unreachable IK result must have no candidates");
            }

            return {
                leg_filter_status::geometric_unreachable,
                {}
            };

        case leg_ik_status::singular:
            if (!ik_result.solution.empty()) {
                throw std::invalid_argument(
                    "Free-angle IK result must have no candidates");
            }

            return {
                leg_filter_status::geometric_singular,
                {}
            };

        case leg_ik_status::success:
            if (ik_result.solution.empty()) {
                throw std::invalid_argument(
                    "Successful IK result must contain candidates");
            }

            break;

        default:
            throw std::invalid_argument(
                "Invalid IK status");
    }

    leg_filter_result result;

    // 过滤后的数量不会超过输入数量。
    result.candidates.reserve(ik_result.solution.size());

    for (const auto& candidate : ik_result.solution) {
        const auto mapped = map_candidate(leg, candidate);

        if (mapped) {
            result.candidates.push_back(*mapped);
        }
    }

    if (!result.candidates.empty()) {
        result.status = leg_filter_status::success;
    }

    return result;
}

//参考角检查
void leg_candidate_selector::require_reference_angles(leg_id leg, const leg_joint_angles& reference_angles) const {

    const std::array<double, joints_per_leg> angles{
        reference_angles.yaw_rad,
        reference_angles.upper_leg_rad,
        reference_angles.lower_leg_rad
    };

    for (const double angle : angles) {
        if (!std::isfinite(angle)) {
            throw std::invalid_argument(
                "Reference joint angles must be finite");
        }
    }

    // 与当前 mapper 一样，只吸收浮点往返误差。
    // 不是机械限位余量。
    constexpr double numerical_tolerance = 1e-12;

    for (std::size_t index = 0; index < joints_per_leg; ++index) {

        const auto& joint =  model_.joint(leg, joint_order[index]);

        if (!model_.is_command_ready(joint.binding.id)) {
            throw std::logic_error("Reference joint calibration is incomplete");
        }

        const auto& limits = *joint.calibration.installed_limits;

        if (angles[index] < limits.minimum_rad - numerical_tolerance || angles[index] > limits.maximum_rad + numerical_tolerance) {
            throw std::out_of_range("Reference angle exceeds installed limits");
        }
    }
}

//获得 输入关节组的分支
leg_branch_state leg_candidate_selector::branch_of(const leg_joint_angles& angles, const leg_kinematics& kinematics) {

    // 与当前 IK 使用的数值判断尺度一致。
    // 不代表机械精度或实机安全余量。
    constexpr double radial_tolerance_m = 1e-12;
    constexpr double knee_sine_tolerance = 1e-6;

    const auto point = kinematics.forward_in_leg_frame(angles);

    // FK:
    // x = r * sin(q1)
    // y = -r * cos(q1)
    //
    // 因此：
    // r = x * sin(q1) - y * cos(q1)
    //
    // 不能用 hypot(x, y)，否则会丢失 r 的符号。
    const double radial_m = point.x * std::sin(angles.yaw_rad) - point.y * std::cos(angles.yaw_rad);

    const double knee_sine = std::sin(angles.lower_leg_rad);

    if (!std::isfinite(radial_m) || !std::isfinite(knee_sine)) {
        throw std::logic_error("Branch classification is not finite");
    }

    leg_branch_state result;

    if (std::abs(radial_m) > radial_tolerance_m) {
        result.radial = radial_m > 0.0 ? ik_branch_sign::positive : ik_branch_sign::negative;
    }

    if (std::abs(knee_sine) > knee_sine_tolerance) {
        result.knee = knee_sine > 0.0 ? ik_branch_sign::positive : ik_branch_sign::negative;
    }

    return result;
}

//对请求进行检查，查看connection request是否合法
void leg_candidate_selector::require_connection_request(const leg_connection_request& request) {

    if (!std::isfinite(request.duration_s) || request.duration_s <= 0.0) {
        throw std::invalid_argument("Connection duration must be finite and positive");
    }

    for (const double speed : request.max_joint_speed_rad_s) {
        if (!std::isfinite(speed) || speed <= 0.0) {
            throw std::invalid_argument("Known joint speed limits must be finite and positive");
        }
    }
}

leg_connection_result leg_candidate_selector::check_connection(
    leg_id leg,
    const leg_joint_angles& reference_angles,
    const leg_joint_angles& target_angles,
    const leg_connection_request& request
) const {

    //这里会有一点额外开销，不过开销很小，为了结构简单和方便调用，可以暂时不优化
    require_connection_request(request);

    // 两个端点都必须是有限且处于安装限位内的角度。
    require_reference_angles(leg, reference_angles);
    require_reference_angles(leg, target_angles);

    const std::array<double, joints_per_leg> delta_rad{
        target_angles.yaw_rad - reference_angles.yaw_rad,

        target_angles.upper_leg_rad - reference_angles.upper_leg_rad,

        target_angles.lower_leg_rad - reference_angles.lower_leg_rad
    };

    std::array<bool, joints_per_leg> joint_passed{};

    bool all_passed = true;

    for (std::size_t index = 0; index < joints_per_leg; ++index) {
        if (!std::isfinite(delta_rad[index])) {
            throw std::invalid_argument("Connection angle difference is not finite");
        }

        const double required_average_speed_rad_s = std::abs(delta_rad[index]) / request.duration_s;

        const bool passed = (required_average_speed_rad_s <= request.max_joint_speed_rad_s[index]);

        joint_passed[index] = passed;

        if (!passed) {
            all_passed = false;
        }
    }

   return leg_connection_result{all_passed ? leg_connection_status::within_average_speed_limits : leg_connection_status::average_speed_limit_exceeded, joint_passed};
}

//最终选择函数
leg_selection_result leg_candidate_selector::select_candidate(
    leg_id leg,
    const leg_ik_result& ik_result,
    const leg_joint_angles& reference_angles,
    const leg_branch_state& remembered_branch,
    const leg_selection_options& options,
    std::optional<leg_connection_request> connection_request
) const {

    // 提供了connection请求，就必须是一份完整、合法的请求。
    if (connection_request) {
        require_connection_request(*connection_request);
    }

    // 拒绝非法策略枚举值。
    if (options.knee_singularity != knee_singularity_policy::allow && options.knee_singularity != knee_singularity_policy::reject) {
        throw std::invalid_argument( "Invalid knee singularity policy");
    }

    if (options.branch_continuity != branch_continuity_policy::nearest && options.branch_continuity != branch_continuity_policy::keep_reference_branch) {
        throw std::invalid_argument("Invalid branch continuity policy");
    }

    const auto filtered = filter_candidates(leg, ik_result);

    leg_selection_result result;

    // 保留几何失败与映射失败的区别。
    switch (filtered.status) {
        case leg_filter_status::geometric_unreachable:
            result.status = leg_selection_status::geometric_unreachable;
            return result;

        case leg_filter_status::geometric_singular:
            result.status = leg_selection_status::geometric_singular;
            return result;

        case leg_filter_status::no_candidate_within_limits:
            result.status = leg_selection_status::no_candidate_within_limits;
            return result;

        case leg_filter_status::success:
            break;

        default:
            throw std::logic_error("Unexpected filter status");
    }

    require_reference_angles(leg, reference_angles);

    // 只在真正需要选解时，要求完整的运动学模型。
    // 原有 map_candidate() 的职责不变。
    const leg_kinematics kinematics(model_);

    const auto observed_branch = branch_of(reference_angles, kinematics);

    // 普通姿态采用角度本身能够确定的分支。
    // 奇异附近才使用历史记忆补充。
    // 历史标签若与明确的参考姿态冲突，拒绝继续。
    const auto resolve_branch = [](std::optional<ik_branch_sign> observed, std::optional<ik_branch_sign> remembered) -> std::optional<ik_branch_sign> {

        // 1. 历史标签有值时，必须是合法的正／负标签
        if (remembered && *remembered != ik_branch_sign::negative && *remembered != ik_branch_sign::positive) {
            throw std::invalid_argument("Invalid remembered branch sign");
        }
        // 2. 两者都有值，却互相矛盾，说明参考和历史不一致。
        if (observed && remembered && *observed != *remembered) {
            throw std::invalid_argument("Remembered branch conflicts with reference");
        }
        // 3. 当前能够判断时采用当前结果；
        //    当前无法判断时采用历史记忆。
        return observed ? observed : remembered;
    };

    const leg_branch_state reference_branch{
        resolve_branch(observed_branch.radial, remembered_branch.radial),

        resolve_branch(observed_branch.knee, remembered_branch.knee)
    };


    const bool keep_branch =  options.branch_continuity == branch_continuity_policy::keep_reference_branch;

    // 要求保持分支，但参考角和历史都无法确定分支。
    // 此时不能自行指定正分支或负分支。
    if (keep_branch && (!reference_branch.radial || !reference_branch.knee)) {
        result.status = leg_selection_status::reference_branch_undetermined;
        return result;
    }

    // 候选端点处于数值奇异附近时，
    // 允许它暂时没有确定标签，随后保留原有记忆。
    const auto matches_branch = [](const leg_branch_state& candidate, const leg_branch_state& expected) {

        const bool radial_matches = !candidate.radial || candidate.radial == expected.radial;

        const bool knee_matches = !candidate.knee || candidate.knee == expected.knee;

        return radial_matches && knee_matches;
    };

    bool has_speed_rejection = false;

    for (const auto& candidate : filtered.candidates) {
        const auto ideal_branch = branch_of(candidate.ik_candidate.angles, kinematics);

        const auto nominal_branch = branch_of(candidate.nominal_angles, kinematics);

        // 取整可能让原本非奇异的膝角变为零。
        // 所以同时检查几何候选与整数目标对应的姿态。
        const bool knee_near_singular = candidate.ik_candidate.knee_near_singular || !ideal_branch.knee || !nominal_branch.knee;

        ////////在后续这段逻辑不能使用默认的option配置，需要优化升级，不过对于v1够用//////////////

        //如果约定的奇异策略不允许的话，这段if会跳过接近奇异解的候选角组合
        if (options.knee_singularity  == knee_singularity_policy::reject && knee_near_singular) {
            continue;
        }

        //分支策略不允许的话
        if (keep_branch && (!matches_branch(ideal_branch, reference_branch) || !matches_branch(nominal_branch, reference_branch))) {
            continue;
        }

        ////////////////////////////////////////////////////////////////////////////////////////

        std::optional<leg_connection_result> candidate_connection;

        // 检查速度是否过快，在评分之前进行。
        if (connection_request) {
            const auto checked = check_connection(leg, reference_angles, candidate.nominal_angles, *connection_request);

            switch (checked.status) {
                case leg_connection_status::average_speed_limit_exceeded:
                    has_speed_rejection = true;
                    continue;

                case leg_connection_status::within_average_speed_limits:
                    candidate_connection = checked;
                    break;

                default:
                    throw std::logic_error("Unexpected connection status");
            }
        }

        const double cost = joint_distance_cost(candidate.nominal_angles, reference_angles);

        // 首套合格候选，或者发现更低评分时更新。
        // 完全同分时保留本次输入顺序中先出现的一套。
        if (!result.cost || cost < *result.cost) {
            result.selected = candidate;
            result.cost = cost;

            // 必须同步保存当前最优候选自己的检查结果
            result.connection = candidate_connection;

            // 优先记录整数命令对应的明确分支。
            // 量化后落在奇异附近时，保留几何分支；
            // 几何分支也不确定时，继续保留参考记忆。
            result.branch.radial = nominal_branch.radial ? nominal_branch.radial  :  (ideal_branch.radial ? ideal_branch.radial : reference_branch.radial);

            result.branch.knee = nominal_branch.knee ? nominal_branch.knee  :  (ideal_branch.knee ? ideal_branch.knee : reference_branch.knee);
        }
    }

    if (result.selected) {
        result.status = leg_selection_status::success;
    }
    else if (has_speed_rejection) {
        result.status = leg_selection_status::no_candidate_within_speed_limits;
    }

    // 否则保持 no_candidate_under_policy。
    return result;
}



}  // namespace hexapod_core
