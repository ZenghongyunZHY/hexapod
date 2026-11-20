#include "hexapod_core/swing_landing.hpp"
#include "hexapod_core/gait_foot_state.hpp"
#include "hexapod_core/body_foot_targets.hpp"
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

swing_landing_request example_request() {
    return {{{0.0, 0.0, 0.1}, 0.0}, {0.02, 0.0, 0.0}, {0.2, -0.15}, 0.8, 1.2, 0.0};
}

void test_independent_placements() {
    const auto forward = example_request();
    const auto landing = plan_swing_landing_point(forward);
    point_near(landing, {0.228, -0.15, 0.0}, "independent midpoint-based landing");
    // 同一个固定地面点在支撑起/中/终的机身 X 为 212/200/188 mm。
    const std::array<double, 3> times{0.8, 1.4, 2.0};
    const std::array<double, 3> expected_x{0.212, 0.2, 0.188};
    for (std::size_t index = 0; index < times.size(); ++index) {
        const auto pose = advance_body_pose(forward.lift_off_pose, forward.velocity, times[index]);
        point_near(planning_to_body_frame(pose, landing),
            {expected_x[index], -0.15, -0.1}, "support start/middle/end positions");
    }
    auto backward = forward;
    backward.velocity.vx_m_s = -0.02;
    point_near(plan_swing_landing_point(backward), {0.172, -0.15, 0.0},
        "backward motion reverses half-stance offset");

    auto sideways = forward;
    sideways.velocity = {0.0, 0.02, 0.0};
    point_near(plan_swing_landing_point(sideways), {0.2, -0.122, 0.0},
        "sideways midpoint placement");

    auto rotated = forward;
    rotated.lift_off_pose = {{0.1, -0.2, 0.3}, pi / 2.0};
    rotated.ground_height_m = 0.05;
    point_near(plan_swing_landing_point(rotated), {0.25, 0.028, 0.05},
        "independent translated quarter-turn heading");

    const swing_landing_request turning{{{0.0, 0.0, 0.1}, 0.0},
        {0.0, 0.0, pi / 4.0}, {0.2, -0.15}, 1.0, 2.0, 0.0};
    point_near(plan_swing_landing_point(turning), {0.15, 0.2, 0.0},
        "quarter-turn occurs at stance time midpoint rather than touchdown");
    auto stationary = rotated;
    stationary.velocity = {0.0, 0.0, 0.0};
    point_near(plan_swing_landing_point(stationary), {0.25, 0.0, 0.05},
        "stationary placement and explicit ground height");

    auto longer_stance = forward;
    longer_stance.next_stance_duration_s = 2.4;
    point_near(plan_swing_landing_point(longer_stance), {0.24, -0.15, 0.0},
        "next stance duration changes prediction horizon");
}

void test_midpoint_property_and_input_ownership() {
    for (const double yaw : {-pi / 2.0, -0.37, 0.0, 0.37, pi / 2.0}) {
        for (const auto& velocity : {planar_body_velocity{0.02, -0.01, 0.0},
                 planar_body_velocity{-0.02, 0.01, -0.3},
                 planar_body_velocity{0.01, 0.02, 0.7}}) {
            for (const double stance : {0.4, 1.2, 2.0}) {
                const swing_landing_request request{{{0.13, -0.27, 0.11}, yaw},
                    velocity, {-0.14, 0.18}, 0.8, stance, -0.02};
                const auto landing = plan_swing_landing_point(request);
                const auto middle_pose = advance_body_pose(request.lift_off_pose, velocity, 0.8 + stance / 2.0);
                point_near(planning_to_body_frame(middle_pose, landing), {-0.14, 0.18, -0.13},
                    "same midpoint reference recovered for translation and turning");
                require(landing.z == -0.02, "ground height is copied exactly");
                require(request.lift_off_pose.yaw_rad == yaw
                    && request.lift_off_pose.body_origin_in_planning_frame.x == 0.13
                    && request.velocity.vx_m_s == velocity.vx_m_s
                    && request.velocity.vy_m_s == velocity.vy_m_s
                    && request.velocity.wz_rad_s == velocity.wz_rad_s
                    && request.midpoint_reference_in_body.x_m == -0.14
                    && request.midpoint_reference_in_body.y_m == 0.18
                    && request.swing_duration_s == 0.8
                    && request.next_stance_duration_s == stance,
                    "request remains unchanged");
            }
        }
    }
}

void test_invalid_and_overflow() {
    const auto valid = example_request();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const std::array<double vector3::*, 3> coordinates{&vector3::x, &vector3::y, &vector3::z};
    const std::array<double planar_body_velocity::*, 3> components{
        &planar_body_velocity::vx_m_s, &planar_body_velocity::vy_m_s, &planar_body_velocity::wz_rad_s};
    const std::array<double swing_landing_request::*, 2> durations{
        &swing_landing_request::swing_duration_s, &swing_landing_request::next_stance_duration_s};
    for (const double bad : {nan, inf, -inf}) {
        for (const auto coordinate : coordinates) {
            auto invalid = valid;
            invalid.lift_off_pose.body_origin_in_planning_frame.*coordinate = bad;
            require_throws<std::invalid_argument>([&] { (void)plan_swing_landing_point(invalid); },
                "invalid initial pose coordinate");
        }
        for (const auto component : components) {
            auto invalid = valid;
            invalid.velocity.*component = bad;
            require_throws<std::invalid_argument>([&] { (void)plan_swing_landing_point(invalid); },
                "invalid velocity component");
        }
        for (const auto component : {&stance_midpoint_reference::x_m, &stance_midpoint_reference::y_m}) {
            auto invalid = valid;
            invalid.midpoint_reference_in_body.*component = bad;
            require_throws<std::invalid_argument>([&] { (void)plan_swing_landing_point(invalid); },
                "invalid horizontal reference");
        }
        auto invalid = valid;
        invalid.lift_off_pose.yaw_rad = bad;
        require_throws<std::invalid_argument>([&] { (void)plan_swing_landing_point(invalid); }, "invalid yaw");
        invalid = valid;
        invalid.ground_height_m = bad;
        require_throws<std::invalid_argument>([&] { (void)plan_swing_landing_point(invalid); }, "invalid ground");
    }
    for (const auto duration : durations) {
        for (const double bad : {0.0, -0.1, std::nextafter(0.0, -1.0), nan, inf, -inf}) {
            auto invalid = valid;
            invalid.*duration = bad;
            require_throws<std::invalid_argument>([&] { (void)plan_swing_landing_point(invalid); },
                "both durations are mandatory, finite and positive");
        }
    }
    const double maximum = std::numeric_limits<double>::max();
    auto overflow = valid;
    overflow.swing_duration_s = maximum;
    overflow.next_stance_duration_s = maximum;
    require_throws<std::overflow_error>([&] { (void)plan_swing_landing_point(overflow); },
        "prediction duration addition overflow");
    overflow = valid;
    overflow.velocity.wz_rad_s = maximum;
    require_throws<std::overflow_error>([&] { (void)plan_swing_landing_point(overflow); },
        "prediction integration overflow");
    overflow = valid;
    overflow.velocity = {0.0, 0.0, 0.0};
    overflow.lift_off_pose.body_origin_in_planning_frame.x = maximum;
    overflow.midpoint_reference_in_body.x_m = maximum;
    require_throws<std::overflow_error>([&] { (void)plan_swing_landing_point(overflow); },
        "horizontal placement transform overflow");
}

void test_saved_landing_and_body_planner(const std::string& config_path) {
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
    const planar_body_pose initial{{0.0, 0.0, 0.0}, 0.0};
    const planar_body_velocity velocity{0.0005, 0.0, 0.0};
    gait_foot_state feet(anchors);
    auto phases = sample_tripod_phase({0.25, 0.625});
    for (const auto leg : leg_order) {
        const auto index = leg_index(leg);
        if (phases[index].phase == leg_phase::swing) {
            const auto landing = plan_swing_landing_point({initial, velocity,
                {anchors[index].x, anchors[index].y}, 2.0, 2.0, anchors[index].z});
            feet = feet.begin_swing(leg, landing, {0.0, 0.0, 1.0}, 0.003);
        }
    }
    std::array<std::array<double, joints_per_leg>, leg_count> speeds{};
    for (auto& limits : speeds) limits.fill(1.0);
    const leg_selection_options options{knee_singularity_policy::reject,
        branch_continuity_policy::keep_reference_branch};
    const ground_plane_in_body ground{{0.0, 0.0, 1.0}, -anchors[0].z + 0.002};
    double maximum_error_m = 0.0;
    for (int segment = 1; segment <= 30; ++segment) {
        const bool swinging = segment <= 20;
        const double duration = swinging ? 0.1 : 0.2;
        const double time = swinging ? segment * 0.1 : 2.0 + (segment - 20) * 0.2;
        for (auto& phase : phases) phase.phase_ratio = swinging ? time / 2.0 : (time - 2.0) / 2.0;
        const auto pose = advance_body_pose(initial, velocity, time);
        const auto points = sample_body_foot_targets({pose, phases, feet.stance_points(), feet.swings()});
        const body_ik_request target{points.points_in_body, references, branches, options,
            body_connection_request{duration, speeds}};
        const auto result = planner.plan({target, ground, {}, 4});
        require(result.status == body_segment_status::checks_passed && result.solution,
            "midpoint landing, saved state and six-leg planner segment " + std::to_string(segment));
        for (const auto leg : leg_order) {
            const auto index = leg_index(leg);
            const auto nominal = kinematics.forward_in_body_frame(leg, result.solution->nominal_angles[index]);
            const auto& ideal = points.points_in_body[index];
            const double error = std::hypot(nominal.x - ideal.x, nominal.y - ideal.y, nominal.z - ideal.z);
            maximum_error_m = std::max(maximum_error_m, error);
            require(error < 0.002, "nominal error within explicit offline test bound");
            if (segment == 25 && (leg == leg_id::left_front || leg == leg_id::right_middle || leg == leg_id::left_rear)) {
                point_near(ideal, anchors[index], "B support midpoint recovers original nominal foot reference");
            }
        }
        for (std::size_t axis = 0; axis < joint_count; ++axis) {
            require(result.solution->targets.nominal_positions[axis]
                && !result.solution->targets.commanded_positions[axis],
                "complete nominal targets without execution commands");
        }
        references = result.solution->nominal_angles;
        branches = result.solution->branches;
        if (segment == 20) {
            for (const auto leg : leg_order) {
                const auto index = leg_index(leg);
                if (phases[index].phase == leg_phase::swing) {
                    feet = feet.finish_swing(leg);
                    phases[index] = {leg_phase::stance, 0.0};
                }
            }
        }
    }
    std::cout << "OFFLINE MIDPOINT LANDING: 20 swing and 10 stance segments passed; max nominal FK error = "
        << maximum_error_m * 1000.0 << " mm.\n";
}

}  // namespace

int main(int argc, char** argv) {
    try {
        require(argc == 2, "usage: swing_landing_tests robot_hardware.json");
        test_independent_placements();
        test_midpoint_property_and_input_ownership();
        test_invalid_and_overflow();
        test_saved_landing_and_body_planner(argv[1]);
        std::cout << "PASS: stance midpoint placement, straight/backward/lateral/turning, "
            "required durations, invalid input and saved-foot planner integration.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
