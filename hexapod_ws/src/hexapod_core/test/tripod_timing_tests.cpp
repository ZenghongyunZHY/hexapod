#include "hexapod_core/tripod_timing.hpp"
#include "hexapod_core/tripod_phase.hpp"
#include "hexapod_core/gait_foot_state.hpp"

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

template <typename exception_type, typename function_type>
void require_throws(function_type function, const std::string& message) {
    try { function(); } catch (const exception_type&) { return; }
    throw std::runtime_error("missing expected exception: " + message);
}

void test_known_events() {
    const auto overlap = make_tripod_cycle_boundaries({4.0, 0.625});
    require(overlap.size() == 5, "five boundaries with overlap");
    const std::array<double, 5> times{0.0, 0.5, 2.0, 2.5, 4.0};
    const std::array<double, 5> ratios{0.0, 0.125, 0.5, 0.625, 1.0};
    for (std::size_t row = 0; row < overlap.size(); ++row) {
        const auto& boundary = overlap[row];
        require(boundary.time_s == times[row] && boundary.cycle_ratio == ratios[row],
            "independent exact timeline");
        require(boundary.cycle_complete == (row == 4), "completion only at last row");
        for (std::size_t leg = 0; leg < leg_count; ++leg) {
            const bool group_a = leg == 0 || leg == 2 || leg == 4;
            require(boundary.begin_swing[leg] == (group_a ? row == 3 : row == 1),
                "independent begin masks");
            require(boundary.finish_swing[leg] == (group_a ? row == 4 : row == 2),
                "independent finish masks");
        }
    }
    const auto touching = make_tripod_cycle_boundaries({4.0, 0.5});
    require(touching.size() == 3, "three boundaries at half duty");
    for (std::size_t row = 0; row < touching.size(); ++row) {
        require(touching[row].time_s == 2.0 * row, "half duty times");
        for (std::size_t leg = 0; leg < leg_count; ++leg) {
            const bool group_a = leg == 0 || leg == 2 || leg == 4;
            require(touching[row].begin_swing[leg] == (group_a ? row == 1 : row == 0),
                "coincident begin masks");
            require(touching[row].finish_swing[leg] == (group_a ? row == 2 : row == 1),
                "coincident finish masks");
        }
    }
}

void test_lifecycle_and_phase_intervals() {
    for (const double duty : {0.5, std::nextafter(0.5, 1.0), 0.625, 0.75, std::nextafter(1.0, 0.0)}) {
        const tripod_timing_request request{4.0, duty};
        const auto boundaries = make_tripod_cycle_boundaries(request);
        std::array<vector3, leg_count> anchors{};
        for (std::size_t index = 0; index < leg_count; ++index) anchors[index] = {0.01 * index, 0.1, 0.0};
        gait_foot_state state(anchors);
        std::array<int, leg_count> begins{};
        std::array<int, leg_count> finishes{};
        for (std::size_t row = 0; row < boundaries.size(); ++row) {
            const auto& boundary = boundaries[row];
            require(boundary.cycle_complete == (row + 1 == boundaries.size()), "final completion flag");
            for (const auto leg : leg_order) {
                const auto index = leg_index(leg);
                if (boundary.finish_swing[index]) {
                    require(state.swings()[index].has_value(), "cannot finish before start");
                    const auto endpoint = sample_swing_trajectory(*state.swings()[index], 1.0);
                    state = state.finish_swing(leg);
                    require(state.stance_points()[index].x == endpoint.x, "endpoint saved on planned landing");
                    ++finishes[index];
                }
                if (boundary.begin_swing[index]) {
                    require(!state.swings()[index], "cannot begin twice");
                    const auto& start = state.stance_points()[index];
                    state = state.begin_swing(leg, {start.x + 0.01, start.y, start.z}, {0.0, 0.0, 1.0}, 0.005);
                    ++begins[index];
                }
            }
            if (row + 1 < boundaries.size()) {
                const auto& next = boundaries[row + 1];
                require(next.time_s > boundary.time_s && next.cycle_ratio > boundary.cycle_ratio,
                    "strictly ordered boundaries");
                const double middle = boundary.cycle_ratio + 0.5 * (next.cycle_ratio - boundary.cycle_ratio);
                // 极窄区间可能没有可表示的内部点；边界仍必须保留。
                if (middle > boundary.cycle_ratio && middle < next.cycle_ratio) {
                    const auto phases = sample_tripod_phase({middle, duty});
                    for (std::size_t index = 0; index < leg_count; ++index) {
                        require(state.swings()[index].has_value() == (phases[index].phase == leg_phase::swing),
                            "events agree with phase sampler inside each interval");
                    }
                }
            }
        }
        for (std::size_t index = 0; index < leg_count; ++index) {
            require(begins[index] == 1 && finishes[index] == 1 && !state.swings()[index],
                "one complete swing per leg and no next-cycle start");
        }
        require(request.period_s == 4.0 && request.stance_ratio == duty, "request unchanged");
    }
}

void test_invalid_and_resolution() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (const double period : {0.0, -1.0, nan, inf, -inf}) {
        require_throws<std::invalid_argument>([&] { (void)make_tripod_cycle_boundaries({period, 0.625}); },
            "invalid period");
    }
    for (const double duty : {std::nextafter(0.5, 0.0), 1.0, -1.0, nan, inf, -inf}) {
        require_throws<std::invalid_argument>([&] { (void)make_tripod_cycle_boundaries({4.0, duty}); },
            "invalid stance ratio");
    }
    for (const double duty : {0.5, 0.625, std::nextafter(0.5, 1.0)}) {
        require_throws<std::range_error>([&] {
            (void)make_tripod_cycle_boundaries({std::numeric_limits<double>::denorm_min(), duty});
        }, "collapsed time boundaries are rejected");
    }
    for (const double period : {1e-200, 1e200, std::numeric_limits<double>::max()}) {
        const auto boundaries = make_tripod_cycle_boundaries({period, 0.625});
        require(boundaries.front().time_s == 0.0 && boundaries.back().time_s == period,
            "representable extremes preserve exact period endpoints");
        for (std::size_t index = 1; index < boundaries.size(); ++index) {
            require(std::isfinite(boundaries[index].time_s)
                && boundaries[index].time_s > boundaries[index - 1].time_s,
                "extreme representable period stays ordered");
        }
    }
}

}  // namespace

int main() {
    try {
        test_known_events();
        test_lifecycle_and_phase_intervals();
        test_invalid_and_resolution();
        std::cout << "PASS: exact tripod boundaries, simultaneous events, full foot lifecycle, phase agreement, invalid input and time resolution.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
