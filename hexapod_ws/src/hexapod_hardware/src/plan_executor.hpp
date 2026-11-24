#pragma once

#include "servo_controller.hpp"

#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace hexapod_hardware {

struct timed_segment {
  std::uint32_t start_offset_ms;
  std::uint16_t move_duration_ms;
  servo_positions positions;
  bool walking_cycle_end;
};

struct timed_plan {
  std::uint64_t id;
  std::chrono::nanoseconds motion_start;
  servo_positions initial_positions;
  std::vector<timed_segment> segments;
};

enum class execution_state : std::uint8_t { accepted, completed, rejected, failed, interrupted };

struct execution_report {
  std::uint64_t id;
  execution_state state;
  std::uint32_t completed_segments;
  std::string reason;
  // Conservative delay beyond the nominal end, populated only on admission.
  std::uint64_t maximum_finish_delay_ms = 0;
};

struct execution_options {
  std::chrono::milliseconds send_advance{90};
  std::chrono::nanoseconds wire_time{move_wire_time};
  std::chrono::milliseconds completion_margin{5};
  std::chrono::milliseconds frame_lateness{25};
  std::chrono::milliseconds write_timeout{25};
};

struct frame_dispatch {
  std::uint64_t id;
  std::size_t index;
  servo_positions positions;
  std::uint16_t move_duration_ms;
  monotonic_time write_deadline;
};

// Worker-owned and independent of ROS, UART and the source of either clock.
class plan_executor {
 public:
  using position_validator = std::function<void(const servo_positions&)>;

  plan_executor(execution_options options, position_validator validate_positions);

  void confirm(const servo_positions& positions);
  bool has_plan() const noexcept { return !queue_.empty(); }
  bool has_reference() const noexcept { return reference_.has_value(); }

  std::optional<execution_report> admit(timed_plan plan, std::chrono::nanoseconds wall_now, monotonic_time steady_now);
  std::optional<execution_report> request_stop(std::uint64_t id, monotonic_time now);
  std::optional<execution_report> fail(std::string reason, monotonic_time now);

  std::optional<execution_report> advance(monotonic_time now);
  std::optional<frame_dispatch> due(monotonic_time now) const;
  void mark_sent(const frame_dispatch& frame, monotonic_time written_at);
  std::optional<monotonic_time> next_wakeup(monotonic_time earliest_dispatch = {}) const;

 private:
  struct accepted_plan {
    timed_plan plan;
    monotonic_time nominal_start;
    std::size_t send_through;
    std::uint64_t maximum_finish_delay_ms;
    std::size_t sent = 0;
    std::size_t completed = 0;
    // Absent only until the first write of an idle-start plan.
    std::optional<monotonic_time> next_write{};
    std::deque<monotonic_time> completions{};
  };

  void validate(const timed_plan& plan) const;
  static std::chrono::milliseconds segment_end(const timed_segment& segment);
  std::uint64_t finish_delay_ms(std::size_t count, const accepted_plan* predecessor) const;
  monotonic_time scheduled_write(const accepted_plan& plan) const;
  static void update_completed(accepted_plan& plan, monotonic_time now);

  execution_options options_;
  position_validator validate_positions_;
  std::optional<servo_positions> reference_;
  std::deque<accepted_plan> queue_;
  std::deque<std::uint64_t> early_stops_;
  std::uint64_t newest_plan_ = 0;
};

}  // namespace hexapod_hardware
