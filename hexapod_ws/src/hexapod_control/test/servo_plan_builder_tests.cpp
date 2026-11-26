#include "servo_plan_builder.hpp"

#include <array>
#include <iostream>
#include <stdexcept>

namespace {

using namespace hexapod_core;
using hexapod_control::make_servo_plan;

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

template <class Error, class Operation>
void require_error(Operation operation, const char* message) {
    try {
        operation();
    } catch (const Error&) {
        return;
    }
    throw std::runtime_error(message);
}

void run_tests() {
    std::array<std::uint16_t, joint_count> initial{};
    servo_positions targets;
    for (std::size_t axis = 0; axis < joint_count; ++axis) {
        initial[axis] = static_cast<std::uint16_t>(400 + axis);
        targets[axis].emplace(static_cast<int>(500 + axis));
    }

    // 相邻两段用取整后的共享边界，首段 200.75 ms 向上取整。
    const std::vector<tripod_timed_segment> timeline{
        {0, 1, 0.0, 0.20075, targets},
        {0, 2, 0.20075, 0.40075, targets},
        {1, 1, 0.40075, 0.60075, targets},
        {2, 1, 0.60075, 0.80075, targets}
    };
    const auto plan = make_servo_plan(timeline, 190, 500, 42, initial, 2);
    require(plan.plan_id == 42 && plan.initial_positions == initial, "Plan reference differs");
    require(plan.motion_start.sec == 0 && plan.motion_start.nanosec == 0, "Builder set absolute time");
    require(plan.segments.size() == timeline.size(), "Plan lost a nominal sample");
    std::uint32_t accumulated_ms = 0;
    for (std::size_t index = 0; index < timeline.size(); ++index) {
        const auto& segment = plan.segments[index];
        require(segment.start_offset_ms == accumulated_ms, "Integer boundary was not shared");
        require(segment.walking_cycle_end == (index == 1 || index == 2), "Incorrect walking boundary");
        accumulated_ms += segment.move_duration_ms;
        for (std::size_t axis = 0; axis < joint_count; ++axis) {
            require(segment.positions[axis] == 500 + axis, "Servo position was changed or reordered");
        }
    }
    require(accumulated_ms == 801, "Duration differs from rounded total");
    require(plan.segments[0].move_duration_ms == 201 && plan.segments[1].move_duration_ms == 200,
        "Durations differ from their shared boundaries");

    const auto recovery = make_servo_plan({{0, 1, 0.0, 0.25, targets}}, 190, 500, 43, initial, 0);
    require(!recovery.segments.front().walking_cycle_end, "Recovery counted as walking");

    require_error<std::out_of_range>([&] {
        make_servo_plan({{0, 1, 0.0, 0.05, targets}}, 190, 500, 42, initial, 1);
    }, "Short interval was accepted");
    require_error<std::out_of_range>([&] {
        make_servo_plan({{0, 1, 0.0, 0.6, targets}}, 190, 500, 42, initial, 1);
    }, "Overlong interval was accepted");
    require_error<std::out_of_range>([&] {
        make_servo_plan({{0, 1, 0.0, 0.20075, targets}, {0, 2, 0.20075, 0.40125, targets}},
            190, 500, 42, initial, 1);
    }, "Rounding shortened a segment below planned time");
    require_error<std::out_of_range>([&] {
        make_servo_plan({{0, 1, 0.0, 5e6, targets}}, 1, 500, 42, initial, 1);
    }, "Overflowing millisecond offset was accepted");

    auto bad = timeline;
    bad.back().nominal_positions.back() = std::nullopt;
    require_error<std::invalid_argument>([&] {
        make_servo_plan(bad, 190, 500, 42, initial, 2);
    }, "Missing position was accepted");
    bad = timeline;
    bad.back().cycle_index = 3;
    require_error<std::invalid_argument>([&] {
        make_servo_plan(bad, 190, 500, 42, initial, 2);
    }, "Missing cycle index was accepted");
    bad = timeline;
    bad.back().start_time_s += 0.001;
    require_error<std::invalid_argument>([&] {
        make_servo_plan(bad, 190, 500, 42, initial, 2);
    }, "Discontinuous interval was accepted");
    require_error<std::invalid_argument>([&] {
        make_servo_plan(timeline, 190, 500, 42, initial, 4);
    }, "Timeline with too few cycles was accepted");
    auto bad_initial = initial;
    bad_initial.back() = 1001;
    require_error<std::out_of_range>([&] {
        make_servo_plan(timeline, 190, 500, 42, bad_initial, 2);
    }, "Invalid initial servo position was accepted");
}

}  // namespace

int main() {
    try {
        run_tests();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
