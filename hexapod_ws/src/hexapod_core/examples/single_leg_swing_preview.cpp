#include "hexapod_core/body_segment_planner.hpp"
#include "hexapod_core/foot_trajectory.hpp"
#include "hexapod_core/joint_mapper.hpp"

#include <array>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace hexapod_core;

int main(int argc, char** argv) {
    try {
        if (argc != 2) {
            throw std::invalid_argument("usage: single_leg_swing_preview robot_hardware.json");
        }

        const auto model = robot_model::from_file(argv[1]);

        const joint_mapper mapper(model);
        const leg_kinematics kinematics(model);
        const body_segment_planner planner(model);

        // reference保存当前段的起始关节角，后面每通过一段，再用这一段的终点更新它。
        // branches保存已经选定的分支，fixed_points保存六条腿最开始的脚端点。
        std::array<leg_joint_angles, leg_count> reference{};
        std::array<leg_branch_state, leg_count> branches{};
        std::array<vector3, leg_count> fixed_points{};

        // 仅用于本离线示例的候选姿态。
        const std::array<double, joints_per_leg> candidate{0.0, degrees_to_radians(12.0), degrees_to_radians(-72.0)};

        // 先映射到整数位置，再解码为一致的名义初始角。
        for (const auto leg : leg_order) {
            const auto index = leg_index(leg);
            std::array<double, joints_per_leg> nominal{};

            for (std::size_t axis = 0; axis < joints_per_leg; ++axis) {

                const auto id = model.joint(leg, joint_order[axis]).binding.id;

                const auto position = mapper.joint_angle_rad_to_servo_position(id, candidate[axis]);

                nominal[axis] = mapper.servo_position_to_joint_angle_rad(id, position);
            }

            reference[index] = leg_joint_angles{nominal[0], nominal[1], nominal[2]};

            fixed_points[index] = kinematics.forward_in_body_frame(leg, reference[index]);
        }

        const auto moving_leg = leg_id::right_middle;
        const auto moving_index = leg_index(moving_leg);

        const auto start = fixed_points[moving_index];

        const vector3 end{
            start.x + 0.010,
            start.y,
            start.z
        };

        // 本例明确以固定机身系作为轨迹规划参考。
        const swing_trajectory_request swing{
            start,
            end,
            {0.0, 0.0, 1.0},
            0.008
        };

        // 合成参考平面，仅用于这个离线示例。
        const ground_plane_in_body ground{
            {0.0, 0.0, 1.0},
            -start.z + 0.002
        };

        std::array<std::array<double, joints_per_leg>,leg_count> speed_limits{};

        for (auto& limits : speed_limits) {
            limits.fill(1.0);
        }

        const leg_selection_options options{
            knee_singularity_policy::reject,
            branch_continuity_policy::keep_reference_branch
        };

        //第一层：在预设的脚端运动轨迹上按进度取点，将整条轨迹划分为20段。
        constexpr std::size_t segment_count = 20;

        constexpr double duration_s = 2.0;

        //第二层：检查每一段时，再将起终关节角之间的线性插值分成4小段，检查包括两端的5个点。
        //这里是在关节角之间取点，再通过正运动学得到脚端点，这些点不一定在预设轨迹上。
        constexpr std::size_t path_intervals = 4;

        const double segment_duration_s = duration_s / static_cast<double>(segment_count);

        std::vector<body_ik_solution> sequence;
        sequence.reserve(segment_count);

        for (std::size_t segment = 1; segment <= segment_count; ++segment) {
            const double ratio = static_cast<double>(segment) / static_cast<double>(segment_count);

            //先保留其他五条腿的目标点，再用当前进度算出摆动腿的目标脚端点。
            auto points = fixed_points;

            points[moving_index] = sample_swing_trajectory(swing, ratio);

            //当前目标脚端点和reference，再结合分支、时间和速度等信息，组成这一段的目标请求。
            const body_ik_request target{
                points,
                reference,
                branches,
                options,
                body_connection_request{
                    segment_duration_s,
                    speed_limits
                }
            };

            const body_segment_request request{
                target,
                ground,
                {},  //六条腿的最低离地间隙都设为0，仍然要检查脚端点不能低于参考地面。
                path_intervals
            };

            //plan先得到目标点的候选解，筛选并选定终点解，再检查reference到这个终点的关节插值路径。
            //选中的解如果没有通过路径检查，本段就失败，不会自动换另一个候选解重新检查。
            const auto result = planner.plan(request);

            if (result.status != body_segment_status::checks_passed || !result.solution) {
                std::cerr << "Failed segment: " << segment;

                if (result.failed_leg) {
                    std::cerr << ", leg_index: " << leg_index(*result.failed_leg);
                }

                if (result.selection_failure) {
                    std::cerr << ", selection_status: " << static_cast<int>(*result.selection_failure);
                }

                if (result.path_failure
                    && result.path_failure->failed_sample) {
                    std::cerr << ", path_ratio: " << result.path_failure ->failed_sample->ratio;
                }

                std::cerr << '\n';

                throw std::runtime_error("Offline swing sequence planning failed");
            }

            sequence.push_back(*result.solution);

            //这一段通过后，将终点的名义关节角作为下一段的reference，同时更新分支记忆。
            //这里更新的是离线规划状态，不代表实物已经运动到了这个位置。
            reference = sequence.back().nominal_angles;
            branches = sequence.back().branches;
        }

        // 全部段规划成功后，才开始输出完整记录。
        std::cout << "time_s,target_x_m,target_y_m,target_z_m" << ",nominal_x_m,nominal_y_m,nominal_z_m";

        for (std::size_t id = 1; id <= joint_count; ++id) {
            std::cout << ",p" << id;
        }

        std::cout << '\n';
        std::cout << std::fixed << std::setprecision(6);

        for (std::size_t index = 0; index < sequence.size(); ++index) {
            const auto& solution = sequence[index];

            const double ratio = static_cast<double>(index + 1) / static_cast<double>(segment_count);

            const auto target_point = sample_swing_trajectory(swing, ratio);

            const auto nominal_point = kinematics.forward_in_body_frame(moving_leg, solution.nominal_angles[moving_index]);

            const double time_s = static_cast<double>(index + 1) * segment_duration_s;

            std::cout
                << time_s
                << ',' << target_point.x
                << ',' << target_point.y
                << ',' << target_point.z
                << ',' << nominal_point.x
                << ',' << nominal_point.y
                << ',' << nominal_point.z;

            for (const auto& position : solution.targets.nominal_positions) {
                if (!position) {
                    throw std::logic_error("Complete preview solution has a missing position");
                }

                std::cout << ',' << position->value();
            }

            std::cout << '\n';
        }

        std::cerr << "OFFLINE: " << sequence.size() << " segments planned successfully.\n";
        return 0;

    } catch (const std::exception& error) {
        std::cerr << "ERROR: " << error.what() << '\n';
        return 1;
    }
}
