#include "hexapod_core/joint_mapper.hpp"
#include "hexapod_core/tripod_sequence_timeline.hpp"

#include <array>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <stdexcept>
#include <vector>

namespace {

using namespace hexapod_core;

tripod_sequence_schedule_request make_preview_request(const robot_model& model) {
    const joint_mapper mapper(model);
    const leg_kinematics kinematics(model);
    const planar_body_pose pose{{0.025, -0.010, 0.0}, 0.0};
    std::array<leg_joint_angles, leg_count> reference_angles{};
    std::array<vector3, leg_count> anchors{};
    std::array<stance_midpoint_reference, leg_count> midpoint_references{};
    std::array<std::array<double, joints_per_leg>, leg_count> speed_limits{};

    // 离线候选姿态。先按物理 ID 取整，再解码为初始参考角。
    const std::array<double, joints_per_leg> candidate{
        0.0, degrees_to_radians(12.0), degrees_to_radians(-72.0)
    };
    for (const auto leg : leg_order) {
        const auto index = leg_index(leg);
        std::array<double, joints_per_leg> angles{};
        for (std::size_t axis = 0; axis < joints_per_leg; ++axis) {
            const auto id = model.joint(leg, joint_order[axis]).binding.id;
            const auto position = mapper.joint_angle_rad_to_servo_position(id, candidate[axis]);
            angles[axis] = mapper.servo_position_to_joint_angle_rad(id, position);
        }

        reference_angles[index] = {angles[0], angles[1], angles[2]};
        const auto point_in_body = kinematics.forward_in_body_frame(leg, reference_angles[index]);
        anchors[index] = body_to_planning_frame(pose, point_in_body);
        midpoint_references[index] = {point_in_body.x, point_in_body.y};
        speed_limits[index].fill(1.0);
    }

    // 四周期离线回归的参数。周期内速度固定，周期之间逐次改变。
    return {
        {pose, gait_foot_state(anchors), reference_angles, {}},
        {
            {{4.0, 0.625}, 10, {0.0005, 0.0, 0.0}, 0.003},
            {{3.0, 0.5}, 6, {0.001, 0.0002, 0.0}, 0.002},
            {{5.0, 0.75}, 4, {0.00025, 0.0, 0.0}, 0.004},
            {{2.5, 0.5625}, 8, {0.0, 0.0, 0.0}, 0.0}
        },
        midpoint_references,
        anchors.front().z,
        {knee_singularity_policy::reject, branch_continuity_policy::keep_reference_branch},
        speed_limits,
        4,
        1e-10,
        0.001,
        foot_clearance_policy{0.002, 0.002, 0.002}
    };
}

void report_sequence_failure(const tripod_sequence_result& result) {
    std::cerr << "ERROR: Offline tripod sequence planning failed";
    if (result.failed_cycle_index) {
        std::cerr << ", cycle_index=" << *result.failed_cycle_index;
    }

    if (result.cycle_failure) {
        const auto& cycle = *result.cycle_failure;
        if (cycle.initial_failed_leg) {
            std::cerr << ", initial_leg_index=" << leg_index(*cycle.initial_failed_leg);
        }
        if (cycle.initial_foot_error_m) {
            std::cerr << ", initial_foot_error_m=" << *cycle.initial_foot_error_m;
        }
        if (cycle.failed_frame_index) {
            std::cerr << ", end_frame_index=" << *cycle.failed_frame_index;
        }
        if (cycle.segment_failure) {
            const auto& segment = *cycle.segment_failure;
            if (segment.failed_leg) {
                std::cerr << ", leg_index=" << leg_index(*segment.failed_leg);
            }
            if (segment.selection_failure) {
                std::cerr << ", selection_status=" << static_cast<int>(*segment.selection_failure);
            }
            if (segment.path_failure) {
                const auto& path = *segment.path_failure;
                std::cerr << ", path_status=" << static_cast<int>(path.status);
                if (path.failed_sample) {
                    std::cerr << ", path_ratio=" << path.failed_sample->ratio;
                }
                if (path.failed_clearance_m) {
                    std::cerr << ", clearance_m=" << *path.failed_clearance_m;
                }
            }
        }
    }
    std::cerr << '\n';
}

void write_csv(const std::vector<tripod_timed_segment>& timeline) {
    // 使用点号作为小数分隔符，输出足够精度以保留时间值。
    std::cout.imbue(std::locale::classic());
    std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);
    std::cout << "cycle_index,end_frame_index,start_time_s,end_time_s,duration_s";
    for (std::size_t id = 1; id <= joint_count; ++id) {
        std::cout << ",p" << id;
    }
    std::cout << '\n';

    for (const auto& segment : timeline) {
        std::cout << segment.cycle_index
            << ',' << segment.end_frame_index
            << ',' << segment.start_time_s
            << ',' << segment.end_time_s
            << ',' << (segment.end_time_s - segment.start_time_s);
        for (const auto& position : segment.nominal_positions) {
            std::cout << ',' << position.value().value();
        }
        std::cout << '\n';
    }
    if (!std::cout.flush()) {
        throw std::runtime_error("Could not write tripod preview CSV");
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc != 2) {
            throw std::invalid_argument("usage: tripod_sequence_preview robot_hardware.json");
        }
        const auto model = robot_model::from_file(argv[1]);
        const tripod_sequence_planner planner(model);
        const auto result = planner.plan(make_preview_request(model));
        if (result.status != tripod_sequence_status::checks_passed || !result.solution) {
            report_sequence_failure(result);
            return 1;
        }

        const auto timeline = make_tripod_sequence_timeline(*result.solution);
        write_csv(timeline);
        std::cerr << "OFFLINE: " << timeline.size() << " segments planned successfully.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "ERROR: " << error.what() << '\n';
        return 1;
    }
}
