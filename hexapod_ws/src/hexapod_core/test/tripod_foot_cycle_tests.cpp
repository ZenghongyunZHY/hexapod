#include "hexapod_core/tripod_foot_cycle.hpp"

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

void point_near(const vector3& actual, const vector3& expected, const std::string& message) {
    require(std::isfinite(actual.x) && std::isfinite(actual.y) && std::isfinite(actual.z)
        && std::abs(actual.x - expected.x) < 1e-12
        && std::abs(actual.y - expected.y) < 1e-12
        && std::abs(actual.z - expected.z) < 1e-12, message);
}

template <typename exception_type, typename function_type>
void require_throws(function_type function, const std::string& message) {
    try { function(); } catch (const exception_type&) { return; }
    throw std::runtime_error("missing expected exception: " + message);
}

tripod_foot_cycle_request example_request(double duty = 0.625) {
    const std::array<vector3, leg_count> anchors{{
        {0.2, -0.15, 0.0}, {0.0, -0.2, 0.0}, {-0.2, -0.15, 0.0},
        {0.2, 0.15, 0.0}, {0.0, 0.2, 0.0}, {-0.2, 0.15, 0.0}}};
    std::array<stance_midpoint_reference, leg_count> references{};
    for (std::size_t index = 0; index < leg_count; ++index) references[index] = {anchors[index].x, anchors[index].y};
    return {{4.0, duty}, 10, {{0.0, 0.0, 0.1}, 0.0}, {0.02, 0.0, 0.0},
        gait_foot_state(anchors), references, 0.0, 0.01};
}

void test_independent_cycle() {
    const auto request = example_request();
    const auto cycle = plan_tripod_foot_cycle(request);
    require(cycle.frames.size() == 41, "forty segments and initial frame");
    for (std::size_t row = 0; row < cycle.frames.size(); ++row) {
        const auto& frame = cycle.frames[row];
        point_near(frame.body_pose.body_origin_in_planning_frame,
            {0.02 * frame.sample.time_s, 0.0, 0.1}, "independent body translation");
        for (std::size_t index = 0; index < leg_count; ++index) {
            const auto& anchor = request.initial_feet.stance_points()[index];
            const bool group_a = index == 0 || index == 2 || index == 4;
            vector3 expected = anchor;
            // B 在 0.5 秒起脚、2 秒落脚，预测到支撑中点 3.25 秒。
            // A 在 2.5 秒起脚、4 秒落脚，预测到支撑中点 5.25 秒。
            if (!group_a && row >= 20) expected.x += 0.065;
            if (group_a && row == 40) expected.x += 0.105;
            if ((!group_a && row <= 10) || (!group_a && row >= 20)
                || (group_a && row <= 30) || (group_a && row == 40)) {
                point_near(frame.targets.points_in_planning_frame[index], expected,
                    "independent fixed supports and exact swing endpoints");
            }
            const auto& planning = frame.targets.points_in_planning_frame[index];
            point_near(frame.targets.points_in_body[index],
                {planning.x - 0.02 * frame.sample.time_s, planning.y, planning.z - 0.1},
                "independent body target transform");
            if (row == 15 && !group_a) {
                point_near(planning, {anchor.x + 0.0325, anchor.y, 0.01},
                    "B midpoint lift and horizontal swing midpoint");
            }
            if (row == 35 && group_a) {
                point_near(planning, {anchor.x + 0.0525, anchor.y, 0.01},
                    "A midpoint lift and horizontal swing midpoint");
            }
            if (frame.sample.boundary && frame.sample.boundary->finish_swing[index]) {
                require(frame.phases[index].phase == leg_phase::swing
                    && frame.phases[index].phase_ratio == 1.0, "landing frame retains old swing endpoint");
            }
        }
    }
    point_near(cycle.final_pose.body_origin_in_planning_frame, {0.08, 0.0, 0.1}, "final pose");
    for (std::size_t index = 0; index < leg_count; ++index) {
        require(!cycle.final_feet.swings()[index] && !request.initial_feet.swings()[index],
            "cycle ends in support and input state stays unchanged");
        auto expected = request.initial_feet.stance_points()[index];
        expected.x += (index == 0 || index == 2 || index == 4) ? 0.105 : 0.065;
        point_near(cycle.final_feet.stance_points()[index], expected, "final saved landing points");
    }
}

void test_half_duty_and_cycle_join() {
    for (const double duty : {0.5, 0.625}) {
        auto request = example_request(duty);
        request.velocity = {0.01, -0.005, 0.2};
        const auto first = plan_tripod_foot_cycle(request);
        request.initial_pose = first.final_pose;
        request.initial_feet = first.final_feet;
        const auto second = plan_tripod_foot_cycle(request);
        for (std::size_t index = 0; index < leg_count; ++index) {
            point_near(first.frames.back().targets.points_in_planning_frame[index],
                second.frames.front().targets.points_in_planning_frame[index], "cycle join in planning coordinates");
            point_near(first.frames.back().targets.points_in_body[index],
                second.frames.front().targets.points_in_body[index], "cycle join in body coordinates");
            require(!second.final_feet.swings()[index], "second cycle completes all swings");
            const auto& end_phase = first.frames.back().phases[index];
            if (index == 0 || index == 2 || index == 4) {
                require(end_phase.phase == leg_phase::swing && end_phase.phase_ratio == 1.0, "A final swing endpoint");
            } else if (duty == 0.5) {
                require(end_phase.phase == leg_phase::stance && end_phase.phase_ratio == 1.0,
                    "B final support does not become next-cycle swing");
                require(first.frames.front().phases[index].phase == leg_phase::swing
                    && first.frames.front().phases[index].phase_ratio == 0.0, "B starts at initial boundary");
            }
        }
        const auto expected_pose = advance_body_pose(example_request().initial_pose, request.velocity, 8.0);
        point_near(second.final_pose.body_origin_in_planning_frame,
            expected_pose.body_origin_in_planning_frame, "two-cycle body integration consistency");
    }
    auto stationary = example_request();
    stationary.velocity = {0.0, 0.0, 0.0};
    stationary.lift_height_m = 0.0;
    const auto cycle = plan_tripod_foot_cycle(stationary);
    for (const auto& frame : cycle.frames) {
        for (std::size_t index = 0; index < leg_count; ++index) {
            point_near(frame.targets.points_in_planning_frame[index], stationary.initial_feet.stance_points()[index],
                "stationary zero-lift cycle preserves all foot points");
        }
    }
}

void test_invalid_input() {
    const auto valid = example_request();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (const double bad : {nan, inf, -inf}) {
        auto request = valid;
        request.ground_height_m = bad;
        require_throws<std::invalid_argument>([&] { (void)plan_tripod_foot_cycle(request); }, "invalid ground");
        request = valid;
        request.lift_height_m = bad;
        require_throws<std::invalid_argument>([&] { (void)plan_tripod_foot_cycle(request); }, "invalid lift");
        request = valid;
        request.velocity.vx_m_s = bad;
        require_throws<std::invalid_argument>([&] { (void)plan_tripod_foot_cycle(request); }, "invalid velocity");
        request = valid;
        request.initial_pose.yaw_rad = bad;
        require_throws<std::invalid_argument>([&] { (void)plan_tripod_foot_cycle(request); }, "invalid initial pose");
        for (std::size_t index = 0; index < leg_count; ++index) {
            for (const auto component : {&stance_midpoint_reference::x_m, &stance_midpoint_reference::y_m}) {
                request = valid;
                request.midpoint_references_in_body[index].*component = bad;
                require_throws<std::invalid_argument>([&] { (void)plan_tripod_foot_cycle(request); }, "invalid reference on any leg");
            }
        }
    }
    for (const auto leg : leg_order) {
        auto request = valid;
        const auto& anchor = request.initial_feet.stance_points()[leg_index(leg)];
        request.initial_feet = request.initial_feet.begin_swing(leg, anchor, {0.0, 0.0, 1.0}, 0.01);
        require_throws<std::invalid_argument>([&] { (void)plan_tripod_foot_cycle(request); }, "active initial swing rejected");
        require(request.initial_feet.swings()[leg_index(leg)].has_value(), "rejected request retains input state");
    }
    auto request = valid;
    request.lift_height_m = -0.01;
    require_throws<std::invalid_argument>([&] { (void)plan_tripod_foot_cycle(request); }, "negative lift");
    request = valid;
    request.intervals_per_stage = 0;
    require_throws<std::invalid_argument>([&] { (void)plan_tripod_foot_cycle(request); }, "invalid subdivision");
    request = valid;
    request.timing.stance_ratio = 1.0;
    require_throws<std::invalid_argument>([&] { (void)plan_tripod_foot_cycle(request); }, "invalid timing");
    request = valid;
    request.velocity.vx_m_s = std::numeric_limits<double>::max();
    require_throws<std::overflow_error>([&] { (void)plan_tripod_foot_cycle(request); }, "prediction overflow propagates");
}

}  // namespace

int main() {
    try {
        test_independent_cycle();
        test_half_duty_and_cycle_join();
        test_invalid_input();
        std::cout << "PASS: full tripod foot cycle, independent landings and lift, fixed supports, half-duty events, cycle joins and input rejection.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
