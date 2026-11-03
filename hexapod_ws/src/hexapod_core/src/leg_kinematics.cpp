#include "hexapod_core/leg_kinematics.hpp"

#include <utility>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <limits>

namespace hexapod_core {

namespace{

    // 只用于比较模型中的理想方向与数值，不是机械误差容差。
    constexpr double numerical_tolerance = 1e-12;
    // 数值比较容差；不代表实物脚端精度或实机运动余量。
    constexpr double ik_position_tolerance_m = 1e-12;
    constexpr double ik_residual_tolerance_m = 1e-10;
    constexpr double ik_singularity_sine_tolerance = 1e-6;

    double normalize_angle_rad(double angle_rad){
        return std::atan2(std::sin(angle_rad),std::cos(angle_rad));
    }

    constexpr double ik_angle_merge_tolerance_rad = 1e-10;

    bool same_joint_angles(const leg_joint_angles& left,const leg_joint_angles& right)
    {
        return
            std::abs(normalize_angle_rad(
                left.yaw_rad - right.yaw_rad))
                <= ik_angle_merge_tolerance_rad

            && std::abs(normalize_angle_rad(
                left.upper_leg_rad - right.upper_leg_rad))
                <= ik_angle_merge_tolerance_rad

            && std::abs(normalize_angle_rad(
                left.lower_leg_rad - right.lower_leg_rad))
                <= ik_angle_merge_tolerance_rad;
    }

    bool same_vector(const vector3& left, const vector3& right){
        return std::abs(left.x - right.x) < numerical_tolerance
            && std::abs(left.y - right.y) < numerical_tolerance
            && std::abs(left.z - right.z) < numerical_tolerance;
    }
    void require_finite_point(const vector3& point) {
        if (!std::isfinite(point.x)|| !std::isfinite(point.y)|| !std::isfinite(point.z)) {
            throw std::invalid_argument("Point coordinates must be finite metres");
        }
    }
}

//构造函数，里面包含一些安全检查
leg_kinematics::leg_kinematics(robot_model model): model_(std::move(model)) {

    const auto& geometry = model_.geometry();

    //FK 使用的几何参数必须存在。

    if(!geometry.first_leg_length_l2_m.value || !geometry.second_leg_length_m.value || !geometry.actual_foot_contact_point_m.value){
        throw std::logic_error("FK requires first length, second length and foot contact point");
    }

    const auto& convention = model_.convention();

    if(!convention){
        throw std::logic_error("FK requires a kinematic convention");
    }

    const auto& axes =  convention->positive_joint_axes_at_zero;

    if( !same_vector(convention->zero_leg_direction,vector3{0.0,-1.0,0.0})
        || !same_vector(axes[0],vector3{0.0,0.0,1.0})
        || !same_vector(axes[1],vector3{-1.0,0.0,0.0})
        || !same_vector(axes[2],vector3{-1.0,0.0,0.0})
    ){
        throw std::logic_error("Unsupported kinematic convention");
    }

    const auto& contact = *geometry.actual_foot_contact_point_m.value;
    const auto& point = contact.position_m;

    // 当前简化公式要求理想触点沿小腿局部 -Y。
    if(contact.frame != "lower_leg"
        || std::abs(point.x) > numerical_tolerance
        || std::abs(point.z) > numerical_tolerance
        || point.y >= 0.0
    ){
        throw std::logic_error(
            "FK requires an axial foot contact point in lower_leg"
        );
    }

}


//返回腿端点坐标 在 腿根部坐标系的表示
vector3 leg_kinematics::forward_in_leg_frame(const leg_joint_angles& angles) const{
    //finite : 有限的（哈哈哈，不认识单词）
    if(!std::isfinite(angles.yaw_rad)
        || !std::isfinite(angles.upper_leg_rad)
        || !std::isfinite(angles.lower_leg_rad)
    ){
        throw std::invalid_argument("Joint angles must be finite radians");
    }

    const auto& geometry = model_.geometry();

    const double first_length_m = *geometry.first_leg_length_l2_m.value;
    const double second_length_m = *geometry.second_leg_length_m.value;

    //这个contact point坐标实际表示的是第三关节到触点的向量
    //所以foot length实际是通俗意义上讲的第三段腿长
    //在侧边聊天中取消了cad模型中的“胶垫”对contact point的影响
    const auto& contact = *geometry.actual_foot_contact_point_m.value;
    const double foot_length_m = -contact.position_m.y;

    const double lower_absolute_rad = angles.upper_leg_rad + angles.lower_leg_rad;

    if(!std::isfinite(lower_absolute_rad)){
        throw std::invalid_argument("Combined pitch angles is not finite");
    }

    const double radial_m = first_length_m
                            + second_length_m * std::cos(angles.upper_leg_rad)
                            + foot_length_m * std::cos(lower_absolute_rad);

    const double height_m = second_length_m * std::sin(angles.upper_leg_rad)
                            + foot_length_m * std::sin(lower_absolute_rad);

    return vector3{
        radial_m * std::sin(angles.yaw_rad),
        -radial_m * std::cos(angles.yaw_rad),
        height_m
    };
}

//将 点 所在的坐标系 从腿根部 转换为 身体坐标系
vector3 leg_kinematics::leg_to_body_frame(leg_id leg,const vector3& point_in_leg) const {
    require_finite_point(point_in_leg);

    const auto& mounting =model_.leg(leg).body_to_first_axis;

    if (!mounting) {
        throw std::logic_error("Leg mounting transform is unknown");
    }

    const auto& transform = *mounting;
    const auto& r = transform.rotation;
    const auto& t = transform.translation_m;

    return vector3{
        r[0] * point_in_leg.x
            + r[1] * point_in_leg.y
            + r[2] * point_in_leg.z
            + t.x,

        r[3] * point_in_leg.x
            + r[4] * point_in_leg.y
            + r[5] * point_in_leg.z
            + t.y,

        r[6] * point_in_leg.x
            + r[7] * point_in_leg.y
            + r[8] * point_in_leg.z
            + t.z
    };
}

//将 点 所在的坐标系，从 身体坐标系 转换到 腿根部坐标系
vector3 leg_kinematics::body_to_leg_frame(leg_id leg,const vector3& point_in_body) const {
    require_finite_point(point_in_body);

    const auto& mounting = model_.leg(leg).body_to_first_axis;

    if (!mounting) {
        throw std::logic_error("Leg mounting transform is unknown");
    }

    const auto& transform = *mounting;
    const auto& r = transform.rotation;
    const auto& t = transform.translation_m;

    // 先减去腿根在机身坐标系中的安装位置。
    const vector3 relative{
        point_in_body.x - t.x,
        point_in_body.y - t.y,
        point_in_body.z - t.z
    };

    // 再乘旋转矩阵的转置。
    return vector3{
        r[0] * relative.x
            + r[3] * relative.y
            + r[6] * relative.z,

        r[1] * relative.x
            + r[4] * relative.y
            + r[7] * relative.z,

        r[2] * relative.x
            + r[5] * relative.y
            + r[8] * relative.z
    };
}

// 组合已有的局部 FK 和安装变换，不重复实现运动学公式。
vector3 leg_kinematics::forward_in_body_frame(leg_id leg, const leg_joint_angles& angles) const {
    const vector3 point_in_leg = forward_in_leg_frame(angles);
    return leg_to_body_frame(leg, point_in_leg);
}


leg_ik_result leg_kinematics::inverse_candidates_in_leg_frame(const vector3& point_in_leg) const{
    require_finite_point(point_in_leg);

    const auto& geometry = model_.geometry();
    const double a = *geometry.first_leg_length_l2_m.value;
    const double b = *geometry.second_leg_length_m.value;
    const double c = - geometry.actual_foot_contact_point_m.value->position_m.y;

    leg_ik_result result;
    result.solution.reserve(4);

    const double inner_reach_m = std::abs(b - c);
    const double outer_reach_m = b + c;
    const double total_reach_m = a + outer_reach_m;

    if(!std::isfinite(total_reach_m)){
        throw std::logic_error("IK geometry scale is not finite");
    }

    const double root_distance_m = std::hypot(point_in_leg.x, point_in_leg.y, point_in_leg.z);

    if(!std::isfinite(root_distance_m) || root_distance_m > total_reach_m + ik_position_tolerance_m){
        return result;
    }

    //hypot对应的公式为ρ = √(x² + y²)
    const double rho_m = std::hypot(point_in_leg.x, point_in_leg.y);
    const double z_m = point_in_leg.z;

    //数值上接近偏航轴时，仍先判断俯仰平面式否可达
    if(rho_m <= ik_position_tolerance_m){
        const double distance_m = std::hypot(a, z_m);

        if (distance_m >= inner_reach_m - ik_position_tolerance_m && distance_m <= outer_reach_m + ik_position_tolerance_m) {
            result.status = leg_ik_status::singular;
        }
        return result;
    }

    // 仅用于识别浮点舍入造成的几何边界偏差。
    const double boundary_roundoff_m = 16.0 * std::numeric_limits<double>::epsilon() * outer_reach_m;

    //分别用+1、-1求一次，因为第二、三端腿可能折叠越过腿根
    for(const double radial_sign : {1.0,-1.0}){
        const double radial_m = radial_sign * rho_m;
        const double yaw_rad = std::atan2(radial_sign*point_in_leg.x,-radial_sign*point_in_leg.y);

        const double u_m = radial_m - a;
        const double distance_m = std::hypot(u_m,z_m);

        if (distance_m < inner_reach_m - ik_position_tolerance_m  ||  distance_m > outer_reach_m + ik_position_tolerance_m) {
            continue;
        }

         // 已判定在几何边界内，只修正边界处的浮点误差。
        const double bounded_distance_m = std::clamp(distance_m,inner_reach_m,outer_reach_m);
        // 两段等长且完全反折到第二关节原点时，
        // 脚点无法唯一确定大腿角。
        if (b == c && bounded_distance_m <= boundary_roundoff_m) {
            result.solution.clear();
            result.status = leg_ik_status::singular;
            return result;
        }

        // 按 b+c 归一化后再平方，减少中间数值溢出的风险。
        const double d_scaled = bounded_distance_m / outer_reach_m;
        const double b_scaled = b / outer_reach_m;
        const double c_scaled = c / outer_reach_m;

        //余弦公式取反得补角，求出q3
        const double cos_knee_raw = (d_scaled * d_scaled - b_scaled * b_scaled - c_scaled * c_scaled) / (2.0 * b_scaled * c_scaled);

        if (!std::isfinite(cos_knee_raw)) {
            throw std::logic_error("IK geometry is numerically degenerate");
        }

        double cos_knee = std::clamp(cos_knee_raw, -1.0, 1.0);

        // 只修正浮点舍入级别的边界偏差。
        if (std::abs(bounded_distance_m - outer_reach_m)
                <= boundary_roundoff_m) {
            cos_knee = 1.0;
        }
        else if (std::abs(bounded_distance_m - inner_reach_m)
                <= boundary_roundoff_m) {
            cos_knee = -1.0;
        }


        const double sin_knee_magnitude = std::sqrt(std::max(0.0, 1.0 - cos_knee * cos_knee));

        const bool knee_near_singular = sin_knee_magnitude <= ik_singularity_sine_tolerance;

        const double knee_magnitude_rad = std::acos(cos_knee);

        //遍历q3的正负两种情况
        for (const double knee_sign : {1.0, -1.0}) {

            const double lower_leg_rad = knee_sign * knee_magnitude_rad;

            const double upper_leg_rad = normalize_angle_rad(std::atan2(z_m, u_m) - std::atan2(c * std::sin(lower_leg_rad), b + c * std::cos(lower_leg_rad)));

            const leg_joint_angles candidate{
                yaw_rad,
                upper_leg_rad,
                lower_leg_rad
            };


            const auto recovered = forward_in_leg_frame(candidate);

            const double residual_m = std::hypot(
                recovered.x - point_in_leg.x,
                recovered.y - point_in_leg.y,
                recovered.z - point_in_leg.z
            );

            if (!std::isfinite(residual_m) || residual_m > ik_residual_tolerance_m) {
                throw std::runtime_error("IK candidate failed FK verification");
            }

            const auto duplicate = std::find_if(result.solution.begin(), result.solution.end(), [&](const leg_ik_candidate& existing) {return same_joint_angles(existing.angles, candidate);});

            if (duplicate == result.solution.end()) {
                result.solution.push_back({
                    candidate,
                    knee_near_singular
                });
            }

            //这段其实可以注释掉
            else {
                duplicate->knee_near_singular = duplicate->knee_near_singular || knee_near_singular;
            }

        }

    }

    if (!result.solution.empty()) {
        result.status = leg_ik_status::success;
    }

    return result;

}

leg_ik_result leg_kinematics::inverse_candidates_in_body_frame(leg_id leg, const vector3& point_in_body) const {
    const vector3 point_in_leg = body_to_leg_frame(leg, point_in_body);

    return inverse_candidates_in_leg_frame(point_in_leg);
}

}// namespace hexapod_core
