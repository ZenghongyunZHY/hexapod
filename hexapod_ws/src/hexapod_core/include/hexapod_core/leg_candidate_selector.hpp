#pragma once

#include "hexapod_core/leg_kinematics.hpp"
#include "hexapod_core/joint_mapper.hpp"

#include <optional>
#include <array>
#include <vector>

namespace hexapod_core {

// 一套通过三轴映射检查的单腿候选。
// 这里只保存数据，不发送命令，也不进行反馈纠偏。
struct leg_mapped_candidate {
    // 原始几何候选：理想关节角与膝奇异标记。
    leg_ik_candidate ik_candidate;

    // 名义整数位置，依次为偏航、大腿、膝关节。
    std::array<servo_position, joints_per_leg> positions;

    // 将上述整数位置解码得到的名义关节角，单位 rad。
    leg_joint_angles nominal_angles;
};

//这用于描述 一条腿的 数套关节组的总状态
enum class leg_filter_status {
    success,//至少一套候选通过指定腿的映射检查
    geometric_unreachable,//上游 IK 已判定目标几何不可达
    geometric_singular,//上游 IK 存在未确定自由角，当前没有完整离散候选
    no_candidate_within_limits//上游有几何候选，但全部被映射范围检查淘汰
};

struct leg_filter_result {
    leg_filter_status status = leg_filter_status::no_candidate_within_limits;

    //存放通过映射的关节角
    std::vector<leg_mapped_candidate> candidates;
};

// 是否允许带膝奇异标记的候选进入选择。
enum class knee_singularity_policy {
    allow,
    reject
};

// 分支策略只检查候选端点，不代替整段轨迹检查。
enum class branch_continuity_policy {
    nearest,
    keep_reference_branch
};

// 这是 IK 分支标签，不是舵机安装方向符号。
enum class ik_branch_sign {
    negative,
    positive
};

// 在奇异附近，某个分支可能暂时无法由角度稳定判断。
// nullopt 表示未知，不使用某个正负号代替未知。
struct leg_branch_state {
    std::optional<ik_branch_sign> radial;
    std::optional<ik_branch_sign> knee;
};

struct leg_selection_options {
    knee_singularity_policy knee_singularity = knee_singularity_policy::reject;
    branch_continuity_policy branch_continuity = branch_continuity_policy::keep_reference_branch;
};

//connection ： 连接前后两帧姿态
struct leg_connection_request {
    // 从参考姿态连接到目标姿态的规划段时长，单位 s。
    double duration_s;

    // 按偏航、大腿、膝关节排列，单位 rad/s。
    // 调用方必须提供三个有限且为正的规划速度上限。
    // 表示名义平均速度预算，不代表设备已验证的最大速度。
    std::array<double, joints_per_leg> max_joint_speed_rad_s{};

    // 构造请求时必须同时提供时长和三个速度上限。
    leg_connection_request(double duration, std::array<double, joints_per_leg> speed_limits) : duration_s(duration), max_joint_speed_rad_s(speed_limits) {
    }
};

//用于防止速度超过上限
enum class leg_connection_status {
    within_average_speed_limits,
    average_speed_limit_exceeded
};

struct leg_connection_result {
    leg_connection_status status;

    std::array<bool, joints_per_leg> joint_passed;

    // 没有默认构造函数。
    // 生成检查结果时必须明确提供完整结果。
    leg_connection_result(leg_connection_status checked_status, std::array<bool, joints_per_leg> passed) : status(checked_status), joint_passed(passed) {
    }
};

enum class leg_selection_status {
    success,

    geometric_unreachable,
    geometric_singular,
    no_candidate_within_limits,
    no_candidate_under_policy,
    reference_branch_undetermined,

    no_candidate_within_speed_limits
};

struct leg_selection_result {
    leg_selection_status status = leg_selection_status::no_candidate_under_policy;

    // 仅 success 时有值。
    std::optional<leg_mapped_candidate> selected;

    // 所选候选相对参考角的角差平方和。
    std::optional<double> cost;

    // 供调用方接受本帧后保存，作为下一帧的分支记忆。
    leg_branch_state branch;

    // 仅在选定候选且本次要求时间检查时有值。
    // 未要求时间检查时保持 nullopt。
    std::optional<leg_connection_result> connection;
};

//数据大致流向： ik----(得到数组关节角组的解）---->filter_candidates                          筛选出可通过映射的关节角组
//                                                  ||                                            ||
//                                                  ||-------------调用map_candidate--------------||
class leg_candidate_selector {
 public:
    explicit leg_candidate_selector(robot_model model);

    // 将一套几何候选映射为指定腿的名义整数位置。
    // 超限返回 nullopt；非法输入或缺少标定仍抛异常。
    // 不发送命令，不进行连续选解或反馈纠偏。
    std::optional<leg_mapped_candidate> map_candidate(leg_id leg, const leg_ik_candidate& candidate) const;

    // 根据几何求解状态，过滤指定腿的全部候选。
    // 保留所有通过映射检查的候选，不进行优先级排序。
    // 输入状态矛盾、非法角度或缺少标定时抛异常。
    leg_filter_result filter_candidates(leg_id leg, const leg_ik_result& ik_result) const;

    // 先进行映射过滤，再应用策略并选择评分最小的一套。
    // reference_angles 必须来自有效参考，不提供默认全零值。
    // remembered_branch 可以为空，表示没有历史分支记忆。
    // 成功不代表轨迹中间点、碰撞或给定时间内可达已经检查通过。
    leg_selection_result select_candidate(
        leg_id leg,
        const leg_ik_result& ik_result,
        const leg_joint_angles& reference_angles,
        const leg_branch_state& remembered_branch,
        const leg_selection_options& options,
        std::optional<leg_connection_request> connection_request = std::nullopt
    ) const;

    // 检查两个姿态之间的平均速度预算。
    // 已提供的速度上限必须有限且为正数。
    // 不修改角度，不进行整圈归一化。
    leg_connection_result check_connection(
        leg_id leg,
        const leg_joint_angles& reference_angles,
        const leg_joint_angles& target_angles,
        const leg_connection_request& request) const;

 private:
    // 三个关节的普通角差平方和，角度输入单位为 rad。
    // 只用于比较姿态距离，不代表连接时间或执行可行性。
    static double joint_distance_cost(const leg_joint_angles& target_angles, const leg_joint_angles& reference_angles);

    // 检查参考角有限且位于该腿的安装限位内。
    // 不对参考角取整，不修改参考姿态。
    void require_reference_angles(leg_id leg, const leg_joint_angles& reference_angles) const;

    // 根据理想 FK 的有符号径向投影及 sin(q3) 判断分支。
    // 奇异附近对应字段返回 nullopt。
    static leg_branch_state branch_of(const leg_joint_angles& angles, const leg_kinematics& kinematics);

    static void require_connection_request(const leg_connection_request& request);

    robot_model model_;
    joint_mapper mapper_;
};

}  //
