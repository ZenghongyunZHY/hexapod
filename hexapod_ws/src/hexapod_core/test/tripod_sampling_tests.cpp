#include "hexapod_core/tripod_sampling.hpp"

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

void boundary_equal(const tripod_boundary& actual, const tripod_boundary& expected) {
    require(actual.cycle_ratio == expected.cycle_ratio && actual.time_s == expected.time_s
        && actual.begin_swing == expected.begin_swing && actual.finish_swing == expected.finish_swing
        && actual.cycle_complete == expected.cycle_complete, "all boundary values and events preserved exactly");
}

void test_known_timeline() {
    const auto samples = make_tripod_cycle_samples({4.0, 0.625}, 2);
    const std::array<double, 9> times{0.0, 0.25, 0.5, 1.25, 2.0, 2.25, 2.5, 3.25, 4.0};
    require(samples.size() == times.size(), "four stages produce nine points for two intervals each");
    for (std::size_t index = 0; index < times.size(); ++index) {
        require(samples[index].time_s == times[index]
            && samples[index].cycle_ratio == times[index] / 4.0, "independent half-stage times");
        require(samples[index].boundary.has_value() == (index % 2 == 0),
            "only actual event rows carry boundary metadata");
    }
    const auto half = make_tripod_cycle_samples({4.0, 0.5}, 2);
    require(half.size() == 5, "two stages at half duty");
    for (std::size_t index = 0; index < half.size(); ++index) {
        require(half[index].time_s == static_cast<double>(index), "independent half-duty times");
    }
    require(half.front().boundary->begin_swing[1], "initial B swing retained at half duty");
    require(half[2].boundary->finish_swing[1] && half[2].boundary->begin_swing[0],
        "simultaneous event retained in one row");
    require(half.back().boundary->cycle_complete && !half.back().boundary->begin_swing[1],
        "no next cycle starts at final row");
}

void test_invariants() {
    for (const double duty : {0.5, 0.625, 0.7, 0.875}) {
        for (const double period : {0.125, 4.0, 7.3, 1e-100, 1e100}) {
            const tripod_timing_request request{period, duty};
            const auto boundaries = make_tripod_cycle_boundaries(request);
            for (const std::size_t count : {1U, 2U, 3U, 10U, 31U}) {
                const auto samples = make_tripod_cycle_samples(request, count);
                require(samples.size() == (boundaries.size() - 1) * count + 1, "sample count contract");
                std::size_t boundary_count = 0;
                std::array<int, leg_count> begins{};
                std::array<int, leg_count> finishes{};
                for (std::size_t index = 0; index < samples.size(); ++index) {
                    const auto& sample = samples[index];
                    require(std::isfinite(sample.time_s) && std::isfinite(sample.cycle_ratio), "finite samples");
                    if (index > 0) {
                        require(sample.time_s > samples[index - 1].time_s
                            && sample.cycle_ratio > samples[index - 1].cycle_ratio, "strict sample order");
                    }
                    if (sample.boundary) {
                        require(index == boundary_count * count, "each boundary appears exactly once at stage end");
                        boundary_equal(*sample.boundary, boundaries[boundary_count]);
                        require(sample.time_s == sample.boundary->time_s
                            && sample.cycle_ratio == sample.boundary->cycle_ratio, "metadata matches sample time");
                        ++boundary_count;
                        for (std::size_t leg = 0; leg < leg_count; ++leg) {
                            begins[leg] += sample.boundary->begin_swing[leg] ? 1 : 0;
                            finishes[leg] += sample.boundary->finish_swing[leg] ? 1 : 0;
                        }
                    } else {
                        const auto& start = boundaries[boundary_count - 1];
                        const auto& end = boundaries[boundary_count];
                        require(sample.time_s > start.time_s && sample.time_s < end.time_s
                            && sample.cycle_ratio > start.cycle_ratio && sample.cycle_ratio < end.cycle_ratio,
                            "ordinary samples strictly inside current stage");
                    }
                }
                require(boundary_count == boundaries.size(), "all original boundaries retained");
                require(samples.front().time_s == 0.0 && samples.front().cycle_ratio == 0.0
                    && samples.back().time_s == period && samples.back().cycle_ratio == 1.0,
                    "exact cycle endpoints");
                for (std::size_t leg = 0; leg < leg_count; ++leg) {
                    require(begins[leg] == 1 && finishes[leg] == 1, "one lift and landing per leg regardless of subdivision");
                }
                if (count == 1) {
                    for (std::size_t row = 0; row < boundaries.size(); ++row) boundary_equal(*samples[row].boundary, boundaries[row]);
                }
            }
            require(request.period_s == period && request.stance_ratio == duty, "input unchanged");
        }
    }
}

void test_bad_inputs_and_precision() {
    require_throws<std::invalid_argument>([] { (void)make_tripod_cycle_samples({4.0, 0.625}, 0); }, "zero intervals");
    for (const double period : {0.0, -1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) {
        require_throws<std::invalid_argument>([&] { (void)make_tripod_cycle_samples({period, 0.625}, 2); }, "invalid period delegated");
    }
    for (const double duty : {0.49, 1.0, std::numeric_limits<double>::quiet_NaN()}) {
        require_throws<std::invalid_argument>([&] { (void)make_tripod_cycle_samples({4.0, duty}, 2); }, "invalid duty delegated");
    }
    require_throws<std::length_error>([] {
        (void)make_tripod_cycle_samples({4.0, 0.625}, std::numeric_limits<std::size_t>::max());
    }, "overflow rejected before allocation");
    const double narrow = std::nextafter(0.5, 1.0);
    const auto boundaries_only = make_tripod_cycle_samples({4.0, narrow}, 1);
    require(boundaries_only.size() == 5, "distinct narrow boundaries retained");
    require_throws<std::range_error>([&] { (void)make_tripod_cycle_samples({4.0, narrow}, 2); },
        "no representable midpoint in one-ulp stage");
    const double tiny = std::numeric_limits<double>::denorm_min();
    (void)make_tripod_cycle_samples({8.0 * tiny, 0.625}, 1);
    require_throws<std::range_error>([&] { (void)make_tripod_cycle_samples({8.0 * tiny, 0.625}, 2); },
        "distinct event times but collapsed inserted sample times");
}

}  // namespace

int main() {
    try {
        test_known_timeline();
        test_invariants();
        test_bad_inputs_and_precision();
        std::cout << "PASS: exact cycle samples, unique event preservation, count/order, half-duty events, input rejection and floating resolution.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
