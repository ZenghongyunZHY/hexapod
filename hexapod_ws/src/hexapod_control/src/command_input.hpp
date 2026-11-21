#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string_view>
#include <vector>

namespace hexapod_control {

using command_components = std::array<double, 3>;
using nanoseconds = std::chrono::nanoseconds;

struct joystick_axis {
    std::size_t index;
    double minimum;
    double center;
    double maximum;
    double direction;
};

struct joystick_config {
    std::array<joystick_axis, 3> axes;
    double deadzone;

    void validate() const;
    command_components read(const std::vector<float>& axes) const;
};

command_components normalize_teleop(command_components input);
void require_walk(const command_components& velocity, std::uint32_t cycles);
nanoseconds require_fresh_stamp(std::int32_t sec, std::uint32_t nanosec, nanoseconds now, nanoseconds timeout);

class command_sequence {
 public:
    std::uint64_t next(nanoseconds monotonic_time);

 private:
    std::uint64_t last_ = 0;
};

enum class teleop_source { joystick, request };

class teleop_gate {
 public:
    explicit teleop_gate(nanoseconds timeout);

    bool accept(teleop_source source, nanoseconds stamp, nanoseconds now, const command_components& input);
    void stop();
    void requested_walk(std::uint64_t sequence);
    void planner_status(std::uint64_t sequence, std::string_view state);

 private:
    nanoseconds timeout_;
    std::array<std::optional<nanoseconds>, 2> last_sample_{};
    std::array<bool, 2> needs_center_{true, false};
    std::optional<teleop_source> owner_;
    std::deque<std::uint64_t> walk_requests_;
    bool finite_walk_active_ = false;
    bool awaiting_reference_ = false;
};

}  // namespace hexapod_control
