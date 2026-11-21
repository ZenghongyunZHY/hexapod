#include "plan_executor.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

namespace hexapod_hardware {
namespace {

using namespace std::chrono_literals;

execution_report report(std::uint64_t id, execution_state state, std::size_t completed, std::string reason = {}) {
  return {id, state, static_cast<std::uint32_t>(completed), std::move(reason)};
}

}  // namespace

plan_executor::plan_executor(execution_options options, position_validator validate_positions)
    : options_(options), validate_positions_(std::move(validate_positions)) {
  if (!validate_positions_ || options_.wire_time < 0ns || options_.send_advance <= options_.wire_time || options_.completion_margin < 0ms) {
    throw std::invalid_argument("Invalid serial timing or position validator");
  }
}

void plan_executor::confirm(const servo_positions& positions) {
  if (has_plan()) {
    throw std::logic_error("Cannot replace the reference while a plan is active");
  }
  validate_positions_(positions);
  reference_ = positions;
}

std::chrono::milliseconds plan_executor::segment_end(const timed_segment& segment) {
  return std::chrono::milliseconds(segment.start_offset_ms) + std::chrono::milliseconds(segment.move_duration_ms);
}

void plan_executor::validate(const timed_plan& plan) const {
  if (plan.segments.empty() || plan.segments.size() > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("Invalid number of servo segments");
  }
  validate_positions_(plan.initial_positions);
  auto next_offset = 0ms;
  for (const auto& segment : plan.segments) {
    if (std::chrono::milliseconds(segment.start_offset_ms) != next_offset) {
      throw std::invalid_argument("Servo segments overlap or are not contiguous from offset zero");
    }
    const auto duration = std::chrono::milliseconds(segment.move_duration_ms);
    if (segment.move_duration_ms < protocol_min_move_ms || segment.move_duration_ms > protocol_max_move_ms ||
        (options_.wire_time != 0ns && duration < options_.send_advance)) {
      throw std::invalid_argument("Servo segment duration cannot fit the configured serial send budget");
    }
    validate_positions_(segment.positions);
    next_offset = segment_end(segment);
  }
}

std::optional<execution_report> plan_executor::admit(timed_plan plan, std::chrono::nanoseconds wall_now, monotonic_time steady_now) {
  if (plan.id == 0) {
    return std::nullopt;
  }
  if (plan.id <= newest_plan_) {
    return std::nullopt;
  }
  newest_plan_ = plan.id;
  if (std::find(early_stops_.begin(), early_stops_.end(), plan.id) != early_stops_.end()) {
    return report(plan.id, execution_state::interrupted, 0, "Stop arrived before the plan");
  }

  try {
    if (!reference_) {
      throw std::logic_error("Standing reference must be confirmed first");
    }
    if (queue_.size() == 2) {
      throw std::logic_error("Only one following servo plan can be queued");
    }
    validate(plan);
    monotonic_time start;
    if (!queue_.empty()) {
      const auto& previous = queue_.back();
      const auto& final = previous.plan.segments.back();
      if (previous.send_through != previous.plan.segments.size() || plan.initial_positions != final.positions ||
          plan.motion_start != previous.plan.motion_start + segment_end(final)) {
        throw std::invalid_argument("Following plan does not match the preceding end time and positions");
      }
      start = previous.start + segment_end(final);
    } else {
      if (plan.initial_positions != *reference_) {
        throw std::invalid_argument("Initial positions do not match the current execution reference");
      }
      start = steady_now + (plan.motion_start - wall_now);
    }
    if (steady_now >= start - options_.send_advance) {
      throw std::invalid_argument("Insufficient lead time before the first frame");
    }
    const auto count = plan.segments.size();
    queue_.push_back({std::move(plan), start, count});
    return report(newest_plan_, execution_state::accepted, 0);
  } catch (const std::logic_error& error) {
    // Rejection invalidates the planner's reference. Clear dependent motion now too.
    queue_.clear();
    reference_.reset();
    return report(newest_plan_, execution_state::rejected, 0, error.what());
  }
}

std::optional<execution_report> plan_executor::request_stop(std::uint64_t id, monotonic_time now) {
  const auto plan = std::find_if(queue_.begin(), queue_.end(), [id](const auto& item) { return item.plan.id == id; });
  if (plan == queue_.end()) {
    if (id > newest_plan_ && std::find(early_stops_.begin(), early_stops_.end(), id) == early_stops_.end()) {
      early_stops_.push_back(id);
      if (early_stops_.size() > 64) {
        early_stops_.pop_front();
      }
    }
    return std::nullopt;
  }
  if (plan != queue_.begin()) {
    if (plan->sent != 0) {
      return fail("Following plan already sent its first frame when cancelled", now);
    }
    queue_.pop_back();
    return report(id, execution_state::interrupted, 0, "Following plan cancelled before its first write");
  }
  if (queue_.size() == 2 && queue_.back().sent != 0) {
    return fail("Following plan already sent its first frame when preceding plan was cancelled", now);
  }
  if (queue_.size() == 2) {
    queue_.pop_back();
  }
  auto& active = queue_.front();
  if (active.sent == 0) {
    queue_.pop_front();
    return report(id, execution_state::interrupted, 0, "Stopped before any servo target was sent");
  }
  for (auto i = active.sent - 1; i < active.send_through; ++i) {
    if (active.plan.segments[i].walking_cycle_end) {
      active.send_through = i + 1;
      break;
    }
  }
  return advance(now);
}

void plan_executor::update_completed(accepted_plan& plan, monotonic_time now) const {
  while (plan.completed < plan.sent && now >= plan.start + segment_end(plan.plan.segments[plan.completed]) + options_.completion_margin) {
    ++plan.completed;
  }
}

std::optional<execution_report> plan_executor::fail(std::string reason, monotonic_time now) {
  if (queue_.empty() && !reference_) {
    return std::nullopt;
  }
  auto failure = report(0, execution_state::failed, 0, std::move(reason));
  if (!queue_.empty()) {
    auto& active = queue_.front();
    update_completed(active, now);
    failure.id = active.plan.id;
    failure.completed_segments = static_cast<std::uint32_t>(active.completed);
  }
  queue_.clear();
  reference_.reset();
  return failure;
}

std::optional<execution_report> plan_executor::advance(monotonic_time now) {
  if (queue_.empty()) {
    return std::nullopt;
  }
  auto& active = queue_.front();
  update_completed(active, now);
  if (active.completed != active.send_through) {
    return std::nullopt;
  }
  const auto complete = active.completed == active.plan.segments.size();
  const auto result = report(active.plan.id, complete ? execution_state::completed : execution_state::interrupted, active.completed,
      complete ? "" : "Stopped at a complete walking-cycle boundary");
  reference_ = active.plan.segments[active.completed - 1].positions;
  queue_.pop_front();
  return result;
}

std::optional<frame_dispatch> plan_executor::due(monotonic_time now) const {
  for (const auto& plan : queue_) {
    if (plan.sent == plan.send_through) {
      continue;
    }
    const auto& segment = plan.plan.segments[plan.sent];
    const auto start = plan.start + std::chrono::milliseconds(segment.start_offset_ms);
    if (now < start - options_.send_advance) {
      return std::nullopt;
    }
    const auto deadline = start - options_.wire_time;
    if (now >= deadline) {
      throw std::runtime_error("Servo frame send deadline missed");
    }
    return frame_dispatch{plan.plan.id, plan.sent, segment.positions, segment.move_duration_ms, deadline};
  }
  return std::nullopt;
}

void plan_executor::mark_sent(const frame_dispatch& frame) {
  for (auto& plan : queue_) {
    if (plan.sent < plan.send_through) {
      if (plan.plan.id != frame.id || plan.sent != frame.index) {
        break;
      }
      ++plan.sent;
      return;
    }
  }
  throw std::logic_error("Dispatched frame does not match the execution queue");
}

std::optional<monotonic_time> plan_executor::next_wakeup() const {
  if (queue_.empty()) {
    return std::nullopt;
  }
  std::optional<monotonic_time> wakeup;
  const auto& active = queue_.front();
  if (active.completed < active.sent) {
    wakeup = active.start + segment_end(active.plan.segments[active.completed]) + options_.completion_margin;
  }
  for (const auto& plan : queue_) {
    if (plan.sent < plan.send_through) {
      const auto send_time = plan.start + std::chrono::milliseconds(plan.plan.segments[plan.sent].start_offset_ms) - options_.send_advance;
      return wakeup ? std::min(*wakeup, send_time) : send_time;
    }
  }
  return wakeup;
}

}  // namespace hexapod_hardware
