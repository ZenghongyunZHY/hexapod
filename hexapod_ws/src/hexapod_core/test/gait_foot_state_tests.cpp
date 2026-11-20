#include "hexapod_core/gait_foot_state.hpp"
#include "hexapod_core/body_foot_targets.hpp"

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

void point_equal(const vector3& actual, const vector3& expected, const std::string& message) {
    require(actual.x == expected.x && actual.y == expected.y && actual.z == expected.z, message);
}

void point_near(const vector3& actual, const vector3& expected, const std::string& message) {
    require(std::isfinite(actual.x) && std::isfinite(actual.y) && std::isfinite(actual.z)
        && std::abs(actual.x - expected.x) < 1e-12
        && std::abs(actual.y - expected.y) < 1e-12
        && std::abs(actual.z - expected.z) < 1e-12, message);
}

void state_equal(const gait_foot_state& actual, const gait_foot_state& expected,
    const std::string& message) {
    for (std::size_t index = 0; index < leg_count; ++index) {
        point_equal(actual.stance_points()[index], expected.stance_points()[index],
            message + " stance point");
        const auto& left = actual.swings()[index];
        const auto& right = expected.swings()[index];
        require(left.has_value() == right.has_value(), message + " curve presence");
        if (left) {
            point_equal(left->start_in_planning_frame, right->start_in_planning_frame,
                message + " curve start");
            point_equal(left->end_in_planning_frame, right->end_in_planning_frame,
                message + " curve end");
            point_equal(left->lift_direction_in_planning_frame, right->lift_direction_in_planning_frame,
                message + " curve direction");
            require(left->lift_height_m == right->lift_height_m, message + " curve height");
        }
    }
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

std::array<vector3, leg_count> synthetic_anchors() {
    // 显式合成坐标，只用于状态与采样验证，不代表实物站姿。
    return {{{0.2, -0.15, 0.0}, {0.0, -0.2, 0.0}, {-0.2, -0.15, 0.0},
        {0.2, 0.15, 0.0}, {0.0, 0.2, 0.0}, {-0.2, 0.15, 0.0}}};
}

void test_each_leg_lifecycle() {
    auto anchors = synthetic_anchors();
    const auto expected_anchors = anchors;
    const gait_foot_state original(anchors);
    anchors[0].x = 99.0;
    for (std::size_t index = 0; index < leg_count; ++index) {
        point_equal(original.stance_points()[index], expected_anchors[index],
            "constructor owns its anchor values");
        require(!original.swings()[index], "initially no swing curves");
    }
    for (const auto leg : leg_order) {
        const auto index = leg_index(leg);
        const auto& start = expected_anchors[index];
        const vector3 end{start.x + 0.02, start.y + 0.01, start.z};
        const auto moving = original.begin_swing(leg, end, {0.0, 0.0, 1.0}, 0.01);
        const auto saved_moving = moving;
        for (std::size_t other = 0; other < leg_count; ++other) {
            point_equal(moving.stance_points()[other], expected_anchors[other],
                "begin does not move stored stance points");
            require(moving.swings()[other].has_value() == (other == index),
                "only requested leg begins swing");
            require(!original.swings()[other], "begin leaves original unchanged");
        }
        point_equal(moving.swings()[index]->start_in_planning_frame, start,
            "curve starts at saved anchor");
        point_equal(moving.swings()[index]->end_in_planning_frame, end,
            "curve saves requested endpoint");
        point_near(sample_swing_trajectory(*moving.swings()[index], 0.5),
            {start.x + 0.01, start.y + 0.005, start.z + 0.01},
            "stored curve midpoint has independent geometry");

        const auto landed = moving.finish_swing(leg);
        state_equal(moving, saved_moving, "finish leaves moving source unchanged");
        for (std::size_t other = 0; other < leg_count; ++other) {
            point_equal(landed.stance_points()[other], other == index ? end : expected_anchors[other],
                "finish updates only requested anchor");
            require(!landed.swings()[other], "finished curve is cleared");
        }
        const vector3 second_end{end.x - 0.01, end.y, end.z};
        const auto restarted = landed.begin_swing(leg, second_end, {0.0, 0.0, 1.0}, 0.0);
        point_equal(restarted.swings()[index]->start_in_planning_frame, end,
            "next swing uses newly accepted anchor instead of original anchor");
        require(restarted.swings()[index]->lift_height_m == 0.0,
            "zero extra lift is a valid explicit curve input");
    }
}

void test_rejected_and_batched_updates() {
    const gait_foot_state committed(synthetic_anchors());
    auto pending = committed;
    for (const auto leg : {leg_id::right_front, leg_id::left_middle, leg_id::right_rear}) {
        const auto& point = pending.stance_points()[leg_index(leg)];
        pending = pending.begin_swing(leg, {point.x + 0.02, point.y, point.z},
            {0.0, 0.0, 1.0}, 0.01);
    }
    const auto snapshot = pending;
    require_throws<std::logic_error>([&] {
        (void)pending.begin_swing(leg_id::right_front, {0.3, -0.15, 0.0},
            {0.0, 0.0, 1.0}, 0.01);
    }, "duplicate begin is rejected");
    require_throws<std::logic_error>([&] {
        (void)pending.finish_swing(leg_id::right_middle);
    }, "finish without curve is rejected");
    state_equal(pending, snapshot, "failed transitions leave candidate unchanged");
    state_equal(committed, gait_foot_state(synthetic_anchors()),
        "unaccepted candidate leaves committed state unchanged");

    auto accepted = pending;
    auto landed = accepted.finish_swing(leg_id::right_front);
    require(landed.swings()[leg_index(leg_id::left_middle)]
        && landed.swings()[leg_index(leg_id::right_rear)],
        "landing one leg preserves other active curves");
    state_equal(accepted, snapshot, "copy and later transitions remain isolated");
    for (const auto bad_leg : {static_cast<leg_id>(-1), static_cast<leg_id>(6), static_cast<leg_id>(99)}) {
        require_throws<std::out_of_range>([&] {
            (void)committed.begin_swing(bad_leg, {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, 0.01);
        }, "invalid begin leg");
        require_throws<std::out_of_range>([&] {
            (void)pending.finish_swing(bad_leg);
        }, "invalid finish leg");
    }
}

void test_invalid_geometry() {
    const gait_foot_state original(synthetic_anchors());
    const std::array<double vector3::*, 3> coordinates{&vector3::x, &vector3::y, &vector3::z};
    for (const double bad : {std::numeric_limits<double>::quiet_NaN(),
             std::numeric_limits<double>::infinity(),
             -std::numeric_limits<double>::infinity()}) {
        for (const auto coordinate : coordinates) {
            for (std::size_t index = 0; index < leg_count; ++index) {
                auto anchors = synthetic_anchors();
                anchors[index].*coordinate = bad;
                require_throws<std::invalid_argument>([&] {
                    (void)gait_foot_state(anchors);
                }, "constructor rejects nonfinite anchor on every axis and leg");
            }
            vector3 endpoint{0.2, -0.2, 0.0};
            endpoint.*coordinate = bad;
            require_throws<std::invalid_argument>([&] {
                (void)original.begin_swing(leg_id::right_middle, endpoint, {0.0, 0.0, 1.0}, 0.01);
            }, "invalid endpoint rejected through curve validation");
            vector3 direction{0.0, 0.0, 1.0};
            direction.*coordinate = bad;
            require_throws<std::invalid_argument>([&] {
                (void)original.begin_swing(leg_id::right_middle, {0.2, -0.2, 0.0}, direction, 0.01);
            }, "invalid direction rejected through curve validation");
        }
        require_throws<std::invalid_argument>([&] {
            (void)original.begin_swing(leg_id::right_middle, {0.2, -0.2, 0.0}, {0.0, 0.0, 1.0}, bad);
        }, "nonfinite lift rejected");
    }
    for (const auto& direction : {vector3{0.0, 0.0, 0.0}, vector3{0.0, 0.0, 2.0}}) {
        require_throws<std::invalid_argument>([&] {
            (void)original.begin_swing(leg_id::right_middle, {0.2, -0.2, 0.0}, direction, 0.01);
        }, "nonunit direction rejected");
    }
    require_throws<std::invalid_argument>([&] {
        (void)original.begin_swing(leg_id::right_middle, {0.2, -0.2, 0.0}, {0.0, 0.0, 1.0}, -0.01);
    }, "negative lift rejected");
    state_equal(original, gait_foot_state(synthetic_anchors()),
        "all rejected geometry requests preserve original state");
}

void test_sampler_handoff() {
    const auto anchors = synthetic_anchors();
    const gait_foot_state initial(anchors);
    const auto leg = leg_id::right_middle;
    const auto index = leg_index(leg);
    const vector3 end{0.02, -0.2, 0.0};
    const auto moving = initial.begin_swing(leg, end, {0.0, 0.0, 1.0}, 0.01);
    const auto snapshot = moving;
    std::array<leg_phase_state, leg_count> phases{};
    for (auto& phase : phases) phase = {leg_phase::stance, 0.0};
    const planar_body_pose pose{{0.01, -0.02, 0.1}, pi / 2.0};
    const auto before = sample_body_foot_targets({pose, phases, initial.stance_points(), initial.swings()});
    phases[index] = {leg_phase::swing, 0.0};
    const auto lift = sample_body_foot_targets({pose, phases, moving.stance_points(), moving.swings()});
    for (std::size_t other = 0; other < leg_count; ++other) {
        point_equal(lift.points_in_planning_frame[other], before.points_in_planning_frame[other],
            "begin swing has no planning target jump");
    }
    for (const double ratio : {0.25, 0.5, 0.75, 1.0}) {
        phases[index].phase_ratio = ratio;
        const auto points = sample_body_foot_targets({pose, phases, moving.stance_points(), moving.swings()});
        for (std::size_t other = 0; other < leg_count; ++other) {
            if (other != index) point_equal(points.points_in_planning_frame[other], anchors[other],
                "other support anchors stay fixed");
        }
    }
    const auto terminal = sample_body_foot_targets({pose, phases, moving.stance_points(), moving.swings()});
    const auto landed = moving.finish_swing(leg);
    phases[index] = {leg_phase::stance, 0.0};
    const auto support = sample_body_foot_targets({pose, phases, landed.stance_points(), landed.swings()});
    for (std::size_t other = 0; other < leg_count; ++other) {
        point_equal(support.points_in_planning_frame[other], terminal.points_in_planning_frame[other],
            "planned touchdown has no planning-frame target jump");
        point_equal(support.points_in_body[other], terminal.points_in_body[other],
            "planned touchdown has no body-frame target jump");
    }
    state_equal(moving, snapshot, "sampling never changes stored swing endpoints");
}

}  // namespace

int main() {
    try {
        test_each_leg_lifecycle();
        test_rejected_and_batched_updates();
        test_invalid_geometry();
        test_sampler_handoff();
        std::cout << "PASS: six-leg foot state lifecycle, copy isolation, rejected transitions, "
            "geometry validation and continuous sampler handoff.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
