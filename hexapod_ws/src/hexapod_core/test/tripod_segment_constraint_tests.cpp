#include "hexapod_core/tripod_segment_constraints.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using namespace hexapod_core;

namespace {

const foot_clearance_policy policy{0.0003, 0.0007, 0.0004};

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void near(double actual, double expected, const std::string& message) {
    require(std::isfinite(actual) && std::abs(actual - expected) < 1e-12, message);
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

tripod_foot_cycle_request fixture(double duty, std::size_t intervals_per_stage,
    double ground, double lift) {
    const std::array<vector3, leg_count> anchors{{
        {0.2, -0.15, ground}, {0.0, -0.2, ground}, {-0.2, -0.15, ground},
        {0.2, 0.15, ground}, {0.0, 0.2, ground}, {-0.2, 0.15, ground}}};
    std::array<stance_midpoint_reference, leg_count> references{};
    for (std::size_t index = 0; index < leg_count; ++index) {
        references[index] = {anchors[index].x, anchors[index].y};
    }
    return {{4.0, duty}, intervals_per_stage, {{0.0, 0.0, ground + 0.1}, 0.2},
        {0.02, -0.01, 0.15}, gait_foot_state(anchors), references, ground, lift};
}

bool belongs_to_group_a(leg_id leg) {
    return leg == leg_id::right_front || leg == leg_id::right_rear || leg == leg_id::left_middle;
}

void test_geometry_and_constraints() {
    constexpr std::size_t path_intervals = 4;
    for (const double duty : {0.5, 0.625}) {
        for (const std::size_t intervals_per_stage : {1U, 7U}) {
            for (const double ground : {-0.82, 0.0, 0.37}) {
                for (const double lift : {0.0, 0.01}) {
                    const auto request = fixture(duty, intervals_per_stage, ground, lift);
                    const auto cycle = plan_tripod_foot_cycle(request);
                    const std::size_t stages = duty == 0.5 ? 2 : 4;
                    require(cycle.frames.size() == stages * intervals_per_stage + 1, "geometry frame count");
                    for (std::size_t frame = 0; frame < cycle.frames.size(); ++frame) {
                        const auto& current = cycle.frames[frame];
                        const double time = current.sample.time_s;
                        for (const auto leg : leg_order) {
                            const auto index = leg_index(leg);
                            // 4 秒时序的独立边界预期。A 在支撑时间结束后抬脚，
                            // B 提前半个周期开始。落脚帧仍有阶段 1，但后继曲线已经清除。
                            const bool group_a = belongs_to_group_a(leg);
                            const double lift_time = 4.0 * duty - (group_a ? 0.0 : 2.0);
                            const double land_time = group_a ? 4.0 : 2.0;
                            const auto& curve = current.outgoing_swings[index];
                            require(curve.has_value() == (time >= lift_time && time < land_time),
                                "outgoing swing follows begin and finish events on each leg");
                            if (curve) {
                                require(curve->lift_height_m == lift
                                    && curve->start_in_planning_frame.z == ground
                                    && curve->end_in_planning_frame.z == ground,
                                    "stored curve keeps original planning height and lift");
                            }
                        }
                        if (frame == 0) continue;
                        const auto& previous = cycle.frames[frame - 1];
                        const auto table = make_tripod_segment_constraints(previous, current, ground, path_intervals, policy);
                        for (const auto leg : leg_order) {
                            const auto index = leg_index(leg);
                            const bool group_a = belongs_to_group_a(leg);
                            const double lift_time = 4.0 * duty - (group_a ? 0.0 : 2.0);
                            const double land_time = group_a ? 4.0 : 2.0;
                            const double start_time = previous.sample.time_s;
                            const auto& points = table[index];
                            require(points.size() == path_intervals + 1, "one constraint per path sample");
                            for (std::size_t point = 0; point < points.size(); ++point) {
                                const double t = start_time + (time - start_time) * (static_cast<double>(point) / path_intervals);
                                const auto& bounds = points[point];
                                require(bounds.ground.unit_normal.x == 0 && bounds.ground.unit_normal.y == 0
                                    && bounds.ground.unit_normal.z == 1, "body plane normal points upward");
                                near(bounds.ground.offset_m, 0.1, "body plane is relative to body origin");
                                if (t <= lift_time || t >= land_time) {
                                    near(bounds.minimum_clearance_m, -policy.contact_below_tolerance_m, "contact lower bound");
                                    require(bounds.maximum_clearance_m.has_value(), "contact samples have an upper bound");
                                    near(*bounds.maximum_clearance_m, policy.contact_above_tolerance_m, "contact upper bound");
                                } else {
                                    const double s = (t - lift_time) / (land_time - lift_time);
                                    const double ideal_height = lift * 64.0 * std::pow(s, 3) * std::pow(1.0 - s, 3);
                                    require(!bounds.maximum_clearance_m, "interior swing must not inherit contact upper bound");
                                    near(bounds.minimum_clearance_m,
                                        std::max(-policy.contact_below_tolerance_m, ideal_height - policy.swing_height_error_m),
                                        "interior lower bound follows original nonlinear swing");
                                }
                            }
                        }
                        for (const auto& curve : previous.outgoing_swings) {
                            if (curve) {
                                require(curve->start_in_planning_frame.z == ground && curve->end_in_planning_frame.z == ground,
                                    "constraint generation does not translate stored curve");
                            }
                        }
                    }
                }
            }
        }
    }
}

void test_invalid_input() {
    const auto cycle = plan_tripod_foot_cycle(fixture(0.625, 1, -0.2, 0.01));
    const auto& lift = cycle.frames[1];
    const auto& land = cycle.frames[2];
    const auto check = [&](const tripod_foot_frame& start, const tripod_foot_frame& end) {
        (void)make_tripod_segment_constraints(start, end, -0.2, 4, policy);
    };
    require_throws<std::invalid_argument>([&] { check(land, lift); }, "frame times must increase");
    auto next = land;
    next.body_pose.body_origin_in_planning_frame.z += 0.001;
    require_throws<std::invalid_argument>([&] { check(lift, next); }, "constant body height");
    const auto middle = leg_index(leg_id::right_middle);
    next = land;
    next.sample.boundary.reset();
    require_throws<std::invalid_argument>([&] { check(lift, next); }, "explicit finish event required");
    auto start = lift;
    start.outgoing_swings[middle]->start_in_planning_frame.z -= 0.001;
    require_throws<std::invalid_argument>([&] { check(start, land); }, "ideal curve starts below ground");
    start = lift;
    start.outgoing_swings[middle].reset();
    require_throws<std::invalid_argument>([&] { check(start, land); }, "missing active swing curve");
    start = lift;
    next = land;
    start.phases[middle].phase_ratio = 0.5;
    next.phases[middle].phase_ratio = std::nextafter(0.5, 1.0);
    require_throws<std::range_error>([&] { check(start, next); }, "unrepresentable interior progress");
    next = land;
    next.phases[middle].phase_ratio = std::numeric_limits<double>::quiet_NaN();
    require_throws<std::invalid_argument>([&] { check(lift, next); }, "nonfinite phase ratio");
    next = land;
    next.targets.points_in_planning_frame[leg_index(leg_id::right_front)].z += 0.001;
    require_throws<std::invalid_argument>([&] { check(lift, next); }, "support height cannot change");
    require_throws<std::invalid_argument>([&] {
        (void)make_tripod_segment_constraints(lift, land, -0.2, 0, policy);
    }, "zero path intervals");
    require_throws<std::length_error>([&] {
        (void)make_tripod_segment_constraints(lift, land, -0.2, std::numeric_limits<std::size_t>::max(), policy);
    }, "impossible path interval count");
    require_throws<std::invalid_argument>([&] {
        (void)make_tripod_segment_constraints(lift, land, -0.2, 4, {-1, 0, 0});
    }, "invalid clearance budget");
    require_throws<std::invalid_argument>([&] {
        (void)make_tripod_segment_constraints(lift, land, std::numeric_limits<double>::infinity(), 4, policy);
    }, "nonfinite planning ground");
}

}  // namespace

int main() {
    try {
        test_geometry_and_constraints();
        test_invalid_input();
        std::cout << "PASS: outgoing curves, alternating tripod transitions, independent sampled heights and invalid input.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
