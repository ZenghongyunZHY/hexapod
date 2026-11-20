#include "hexapod_core/body_foot_targets.hpp"
#include "hexapod_core/body_motion.hpp"
#include "hexapod_core/body_segment_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using namespace hexapod_core;

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void near(double actual, double expected, const std::string& message) {
    require(std::isfinite(actual) && std::abs(actual - expected) < 1e-12, message);
}

void point_near(const vector3& actual, const vector3& expected, const std::string& message) {
    near(actual.x, expected.x, message + " x");
    near(actual.y, expected.y, message + " y");
    near(actual.z, expected.z, message + " z");
}

template <typename exception_type, typename function_type>
void require_throws(function_type function, const std::string& message) {
    try {
        function();
    } catch (const exception_type&) {
        return;
    }
    throw std::runtime_error("missing expected exception: " + message);
}

body_foot_target_request synthetic_request() {
    std::array<vector3, leg_count> anchors{};
    std::array<leg_phase_state, leg_count> phases{};
    std::array<std::optional<swing_trajectory_request>, leg_count> swings{};
    for (std::size_t index = 0; index < leg_count; ++index) {
        const double offset = static_cast<double>(index) * 0.01;
        anchors[index] = {0.2 + offset, -0.15 + offset, 0.0};
        phases[index] = {index % 2 == 0 ? leg_phase::stance : leg_phase::swing, 0.5};
        const auto& start = anchors[index];
        swings[index].emplace(start, vector3{start.x + 0.02, start.y, 0.0},
            vector3{0.0, 0.0, 1.0}, 0.01);
    }
    return {{{0.01, -0.02, 0.1}, pi / 2.0}, phases, anchors, swings};
}

void test_independent_mixed_targets() {
    const auto request = synthetic_request();
    const auto result = sample_body_foot_targets(request);
    for (std::size_t index = 0; index < leg_count; ++index) {
        const auto& anchor = request.stance_points_in_planning_frame[index];
        const bool swing = index % 2 != 0;
        const vector3 expected{anchor.x + (swing ? 0.01 : 0.0),
            anchor.y, swing ? 0.01 : 0.0};
        point_near(result.points_in_planning_frame[index], expected,
            "independent stance/swing midpoint in planning frame");
        point_near(result.points_in_body[index],
            {expected.y + 0.02, 0.01 - expected.x, expected.z - 0.1},
            "independent quarter-turn inverse transform");
        require(request.phases[index].phase_ratio == 0.5
            && request.swings[index]->lift_height_m == 0.01,
            "request phase and curve remain unchanged");
    }
    // 显式覆盖六个数组位置，防止只为某组三腿生成目标。
    for (std::size_t moving = 0; moving < leg_count; ++moving) {
        auto single = request;
        for (auto& phase : single.phases) phase = {leg_phase::stance, 0.3};
        single.phases[moving] = {leg_phase::swing, 0.5};
        const auto targets = sample_body_foot_targets(single);
        for (std::size_t index = 0; index < leg_count; ++index) {
            auto expected = single.stance_points_in_planning_frame[index];
            if (index == moving) { expected.x += 0.01; expected.z += 0.01; }
            point_near(targets.points_in_planning_frame[index], expected,
                "each leg can independently use a swing request");
        }
    }
}

void test_endpoints_support_and_inactive_curves() {
    auto request = synthetic_request();
    for (auto& phase : request.phases) phase = {leg_phase::swing, 0.0};
    const auto start = sample_body_foot_targets(request);
    for (std::size_t index = 0; index < leg_count; ++index) {
        point_near(start.points_in_planning_frame[index],
            request.swings[index]->start_in_planning_frame, "exact swing start");
        request.phases[index].phase_ratio = 1.0;
    }
    const auto end = sample_body_foot_targets(request);
    for (std::size_t index = 0; index < leg_count; ++index) {
        point_near(end.points_in_planning_frame[index],
            request.swings[index]->end_in_planning_frame, "exact swing end");
        request.stance_points_in_planning_frame[index] =
            request.swings[index]->end_in_planning_frame;
        request.phases[index] = {leg_phase::stance, 0.0};
        request.swings[index].reset();
    }
    const auto landed = sample_body_foot_targets(request);
    for (std::size_t index = 0; index < leg_count; ++index) {
        point_near(landed.points_in_planning_frame[index],
            end.points_in_planning_frame[index], "planned touchdown has no target jump");
    }
    request.body_pose = {{0.02, -0.03, 0.1}, 0.0};
    for (auto& phase : request.phases) phase.phase_ratio = 1.0;
    const auto moved = sample_body_foot_targets(request);
    for (std::size_t index = 0; index < leg_count; ++index) {
        const auto& fixed = request.stance_points_in_planning_frame[index];
        point_near(moved.points_in_planning_frame[index], fixed,
            "support planning point is independent of body pose and stance progress");
        point_near(moved.points_in_body[index], {fixed.x - 0.02, fixed.y + 0.03, fixed.z - 0.1},
            "body motion changes support target in body frame");
    }
    // 支撑期不消费可选曲线，即使其中存放了当前不用的非法数据。
    request.swings[0].emplace(vector3{0.0, 0.0, 0.0}, vector3{0.0, 0.0, 0.0},
        vector3{0.0, 0.0, 0.0}, -1.0);
    point_near(sample_body_foot_targets(request).points_in_planning_frame[0],
        request.stance_points_in_planning_frame[0], "inactive curve is ignored");
}

void test_invalid_requests() {
    const auto valid = synthetic_request();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const std::array<double vector3::*, 3> coordinates{&vector3::x, &vector3::y, &vector3::z};
    for (const double bad : {nan, inf, -inf}) {
        for (const auto coordinate : coordinates) {
            auto invalid = valid;
            invalid.body_pose.body_origin_in_planning_frame.*coordinate = bad;
            require_throws<std::invalid_argument>([&] {
                (void)sample_body_foot_targets(invalid);
            }, "invalid pose translation");
            for (std::size_t index = 0; index < leg_count; ++index) {
                invalid = valid;
                invalid.stance_points_in_planning_frame[index].*coordinate = bad;
                require_throws<std::invalid_argument>([&] {
                    (void)sample_body_foot_targets(invalid);
                }, "all stored stance coordinates must be finite");
            }
        }
        auto invalid = valid;
        invalid.body_pose.yaw_rad = bad;
        require_throws<std::invalid_argument>([&] {
            (void)sample_body_foot_targets(invalid);
        }, "invalid pose yaw");
    }
    for (std::size_t index = 0; index < leg_count; ++index) {
        auto invalid = valid;
        invalid.phases[index].phase = static_cast<leg_phase>(99);
        require_throws<std::invalid_argument>([&] {
            (void)sample_body_foot_targets(invalid);
        }, "invalid leg phase enum");
        for (const double bad : {std::nextafter(0.0, -1.0), std::nextafter(1.0, 2.0), nan, inf, -inf}) {
            invalid = valid;
            invalid.phases[index].phase_ratio = bad;
            require_throws<std::invalid_argument>([&] {
                (void)sample_body_foot_targets(invalid);
            }, "invalid stage ratio on any leg");
        }
        invalid = valid;
        invalid.phases[index].phase = leg_phase::swing;
        invalid.swings[index].reset();
        require_throws<std::invalid_argument>([&] {
            (void)sample_body_foot_targets(invalid);
        }, "swing requires stored curve on every leg");
        require(!invalid.swings[index], "failed request does not invent a curve");

        invalid = valid;
        invalid.phases[index].phase = leg_phase::swing;
        invalid.swings[index]->lift_direction_in_planning_frame = {0.0, 0.0, 0.0};
        require_throws<std::invalid_argument>([&] {
            (void)sample_body_foot_targets(invalid);
        }, "active swing delegates direction validation");
        invalid.swings[index]->lift_direction_in_planning_frame = {0.0, 0.0, 1.0};
        invalid.swings[index]->lift_height_m = -1.0;
        require_throws<std::invalid_argument>([&] {
            (void)sample_body_foot_targets(invalid);
        }, "active swing delegates height validation");
    }
    auto overflow = valid;
    overflow.phases[5] = {leg_phase::stance, 0.0};
    overflow.body_pose = {{-std::numeric_limits<double>::max(), 0.0, 0.0}, 0.0};
    overflow.stance_points_in_planning_frame[5].x = std::numeric_limits<double>::max();
    require_throws<std::overflow_error>([&] {
        (void)sample_body_foot_targets(overflow);
    }, "last-leg transform overflow propagates instead of returning partial targets");
    require(overflow.stance_points_in_planning_frame[5].x == std::numeric_limits<double>::max(),
        "failed request input remains unchanged");
}

void test_moving_body_integration(const std::string& config_path) {
    const auto model = robot_model::from_file(config_path);
    const joint_mapper mapper(model);
    const leg_kinematics kinematics(model);
    const body_segment_planner planner(model);
    std::array<leg_joint_angles, leg_count> references{};
    std::array<leg_branch_state, leg_count> branches{};
    std::array<vector3, leg_count> anchors{};
    const std::array<double, 3> candidate{0.0, degrees_to_radians(12.0), degrees_to_radians(-72.0)};
    for (const auto leg : leg_order) {
        std::array<double, 3> nominal{};
        for (std::size_t axis = 0; axis < joints_per_leg; ++axis) {
            const auto id = model.joint(leg, joint_order[axis]).binding.id;
            nominal[axis] = mapper.servo_position_to_joint_angle_rad(id,
                mapper.joint_angle_rad_to_servo_position(id, candidate[axis]));
        }
        const auto index = leg_index(leg);
        references[index] = {nominal[0], nominal[1], nominal[2]};
        anchors[index] = kinematics.forward_in_body_frame(leg, references[index]);
    }
    std::array<std::optional<swing_trajectory_request>, leg_count> swings{};
    // 用既定三脚相位取得 B 组分配；本用例只覆盖一次 B 组摆动。
    auto phases = sample_tripod_phase({0.25, 0.625});
    for (std::size_t index = 0; index < leg_count; ++index) {
        if (phases[index].phase == leg_phase::swing) {
            const auto& start = anchors[index];
            swings[index].emplace(start, vector3{start.x + 0.004, start.y, start.z},
                vector3{0.0, 0.0, 1.0}, 0.003);
        }
    }
    const planar_body_pose initial{{0.0, 0.0, 0.0}, 0.0};
    const planar_body_velocity velocity{0.0005, 0.0, 0.0};
    const ground_plane_in_body ground{{0.0, 0.0, 1.0}, -anchors[0].z + 0.002};
    std::array<std::array<double, joints_per_leg>, leg_count> speeds{};
    for (auto& limits : speeds) limits.fill(1.0);
    const leg_selection_options options{knee_singularity_policy::reject,
        branch_continuity_policy::keep_reference_branch};
    double maximum_error_m = 0.0;
    for (int segment = 1; segment <= 20; ++segment) {
        const double ratio = static_cast<double>(segment) / 20.0;
        for (auto& phase : phases) phase.phase_ratio = ratio;
        const auto pose = advance_body_pose(initial, velocity, 2.0 * ratio);
        const auto targets = sample_body_foot_targets({pose, phases, anchors, swings});
        const body_ik_request target{targets.points_in_body, references, branches, options,
            body_connection_request{0.1, speeds}};
        const auto planned = planner.plan({target, ground, {}, 4});
        require(planned.status == body_segment_status::checks_passed && planned.solution,
            "moving-body mixed-foot segment " + std::to_string(segment));
        for (const auto leg : leg_order) {
            const auto index = leg_index(leg);
            if (phases[index].phase == leg_phase::stance) {
                point_near(targets.points_in_planning_frame[index], anchors[index],
                    "support anchor stays fixed through all segments");
            }
            const auto nominal = kinematics.forward_in_body_frame(leg,
                planned.solution->nominal_angles[index]);
            const auto& ideal = targets.points_in_body[index];
            const double error = std::hypot(nominal.x - ideal.x, nominal.y - ideal.y, nominal.z - ideal.z);
            maximum_error_m = std::max(maximum_error_m, error);
            require(error < 0.002, "nominal FK error within explicit offline test bound");
        }
        for (std::size_t axis = 0; axis < joint_count; ++axis) {
            require(planned.solution->targets.nominal_positions[axis]
                && !planned.solution->targets.commanded_positions[axis],
                "complete nominal solution without execution commands");
        }
        references = planned.solution->nominal_angles;
        branches = planned.solution->branches;
    }
    std::cout << "OFFLINE MOVING-BODY FOOT TARGETS: 20 segments passed; max nominal FK error = "
        << maximum_error_m * 1000.0 << " mm.\n";
}

}  // namespace

int main(int argc, char** argv) {
    try {
        require(argc == 2, "usage: body_foot_target_tests robot_hardware.json");
        test_independent_mixed_targets();
        test_endpoints_support_and_inactive_curves();
        test_invalid_requests();
        test_moving_body_integration(argv[1]);
        std::cout << "PASS: mixed feet, transforms, fixed supports, endpoints, missing requests, "
            "invalid input and moving-body planner integration.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
