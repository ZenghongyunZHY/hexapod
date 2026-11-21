#include "command_input.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace hexapod_control {
namespace {

constexpr std::int64_t nanoseconds_per_second = 1'000'000'000;
constexpr command_components centered{};

bool finite(const command_components& values) {
    return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}

std::size_t source_index(teleop_source source) {
    return static_cast<std::size_t>(source);
}

double read_axis(const joystick_axis& axis, const std::vector<float>& sample) {
    if (axis.index >= sample.size()) {
        throw std::invalid_argument("Joy sample is missing a configured axis");
    }
    const double value = sample[axis.index];
    if (!std::isfinite(value)) {
        throw std::invalid_argument("Joy axis is nonfinite");
    }
    const double bounded = std::clamp(value, axis.minimum, axis.maximum);
    const double span = bounded >= axis.center ? axis.maximum - axis.center : axis.center - axis.minimum;
    return axis.direction * ((bounded - axis.center) / span);
}

}  // namespace

void joystick_config::validate() const {
    if (!std::isfinite(deadzone) || deadzone < 0.0 || deadzone >= 1.0) {
        throw std::invalid_argument("joystick_deadzone must be in [0, 1)");
    }
    if (axes[0].index == axes[1].index || axes[0].index == axes[2].index || axes[1].index == axes[2].index) {
        throw std::invalid_argument("axis_indices must select three distinct axes");
    }
    for (const auto& axis : axes) {
        const double lower_span = axis.center - axis.minimum;
        const double upper_span = axis.maximum - axis.center;
        if (!std::isfinite(lower_span) || lower_span <= 0.0 || !std::isfinite(upper_span) || upper_span <= 0.0) {
            throw std::invalid_argument("axis calibration needs finite minimum < center < maximum");
        }
        if (axis.direction != -1.0 && axis.direction != 1.0) {
            throw std::invalid_argument("axis_directions must use -1.0 or 1.0");
        }
    }
}

command_components joystick_config::read(const std::vector<float>& sample) const {
    command_components result{read_axis(axes[0], sample), read_axis(axes[1], sample), read_axis(axes[2], sample)};
    if (std::hypot(result[0], result[1]) <= deadzone) {
        result[0] = 0.0;
        result[1] = 0.0;
    }
    if (std::abs(result[2]) <= deadzone) {
        result[2] = 0.0;
    }
    return normalize_teleop(result);
}

command_components normalize_teleop(command_components input) {
    if (!finite(input)) {
        throw std::invalid_argument("TELEOP requires finite inputs");
    }
    // 先缩放较大的分量，避免有限输入在求平移模长时溢出。
    const double component = std::max({1.0, std::abs(input[0]), std::abs(input[1])});
    input[0] /= component;
    input[1] /= component;
    const double radius = std::max(1.0, std::hypot(input[0], input[1]));
    return {input[0] / radius, input[1] / radius, std::clamp(input[2], -1.0, 1.0)};
}

void require_walk(const command_components& velocity, std::uint32_t cycles) {
    if (!finite(velocity) || velocity == centered || cycles == 0) {
        throw std::invalid_argument("WALK_CYCLES requires a finite nonzero physical velocity and positive cycle_count");
    }
}

nanoseconds require_fresh_stamp(std::int32_t sec, std::uint32_t nanosec, nanoseconds now, nanoseconds timeout) {
    if (sec < 0 || nanosec >= nanoseconds_per_second || (sec == 0 && nanosec == 0)) {
        throw std::invalid_argument("input does not have a valid system timestamp");
    }
    const nanoseconds stamp(static_cast<std::int64_t>(sec) * nanoseconds_per_second + nanosec);
    if (stamp > now || now - stamp >= timeout) {
        throw std::invalid_argument("input timestamp is expired or ahead of the robot clock");
    }
    return stamp;
}

std::uint64_t command_sequence::next(nanoseconds monotonic_time) {
    if (monotonic_time.count() < 0 || last_ == std::numeric_limits<std::uint64_t>::max()) {
        throw std::overflow_error("cannot allocate a new command sequence");
    }
    // 同一台机器人上的单调时钟在接收节点重启时继续前进。
    last_ = std::max(last_ + 1, static_cast<std::uint64_t>(monotonic_time.count()));
    return last_;
}

teleop_gate::teleop_gate(nanoseconds timeout) : timeout_(timeout) {
    if (timeout_ <= nanoseconds::zero()) {
        throw std::invalid_argument("input timeout must be positive");
    }
}

bool teleop_gate::accept(teleop_source source, nanoseconds stamp, nanoseconds now, const command_components& input) {
    if (finite_walk_active_ || awaiting_reference_) {
        return false;
    }
    if (owner_ && now - *last_sample_[source_index(*owner_)] >= timeout_) {
        owner_.reset();
    }
    const auto index = source_index(source);
    auto& previous = last_sample_[index];
    if (previous && stamp <= *previous) {
        return false;
    }
    if (previous && stamp - *previous >= timeout_) {
        needs_center_[index] = true;
    }
    previous = stamp;
    if (needs_center_[index]) {
        if (input != centered) {
            return false;
        }
        needs_center_[index] = false;
    }
    if (owner_ && *owner_ != source) {
        return false;
    }
    if (input == centered) {
        owner_.reset();
    } else {
        owner_ = source;
    }
    return true;
}

void teleop_gate::stop() {
    owner_.reset();
    needs_center_.fill(true);
}

void teleop_gate::requested_walk(std::uint64_t sequence) {
    if (walk_requests_.size() == 64) {
        walk_requests_.pop_front();
    }
    walk_requests_.push_back(sequence);
}

void teleop_gate::planner_status(std::uint64_t sequence, std::string_view state) {
    if (state == "planned" || state == "rejected") {
        const auto request = std::find(walk_requests_.begin(), walk_requests_.end(), sequence);
        if (request != walk_requests_.end()) {
            walk_requests_.erase(request);
            if (state == "planned") {
                finite_walk_active_ = true;
                stop();
            }
        }
        return;
    }
    if (state == "fault" || state == "waiting_for_reference") {
        finite_walk_active_ = false;
        awaiting_reference_ = true;
        stop();
    } else if (state == "standing") {
        finite_walk_active_ = false;
        awaiting_reference_ = false;
    }
}

}  // namespace hexapod_control
