#include "serial_runtime.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

namespace hexapod_hardware {
namespace {

using namespace std::chrono_literals;
using steady_clock = std::chrono::steady_clock;

struct stop_request { std::uint64_t id; };
struct transition_request {};
struct plan_request {
  timed_plan plan;
  std::optional<std::chrono::nanoseconds> standing_stamp;
};
using serial_request = std::variant<plan_request, stop_request, transition_request>;

std::chrono::nanoseconds wall_now() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch());
}

std::pair<double, double> supply_limits_mv(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Cannot open hardware configuration for voltage limits");
  }
  const auto limits = nlohmann::json::parse(input).at("servo").at("supply_voltage_v");
  if (!limits.is_array() || limits.size() != 2) {
    throw std::runtime_error("servo.supply_voltage_v must contain a minimum and maximum");
  }
  const double minimum = limits.at(0).get<double>() * 1000.0;
  const double maximum = limits.at(1).get<double>() * 1000.0;
  if (!std::isfinite(minimum) || !std::isfinite(maximum) || minimum <= 0.0 || maximum < minimum || maximum > 65535.0) {
    throw std::runtime_error("Invalid servo.supply_voltage_v limits");
  }
  return {minimum, maximum};
}

constexpr auto query_voltage_budget = 60ms;
constexpr auto query_positions_budget = 400ms;
constexpr auto diagnostic_period = 2s;
constexpr auto reconnect_period = 1s;
constexpr auto link_check_period = 20ms;

serial_settings checked_settings(serial_settings settings, const plan_executor::position_validator& validate_positions,
                                 const serial_runtime::standing_validator& validate_standing) {
  if (settings.mode == serial_mode::execute) {
    if (!settings.standing_positions || !validate_positions || !validate_standing) {
      throw std::invalid_argument("Execute mode requires nominal standing positions and a readback validator");
    }
    validate_positions(*settings.standing_positions);
    if (settings.standing_transition_time.count() < protocol_min_move_ms ||
        settings.standing_transition_time.count() > protocol_max_move_ms) {
      throw std::invalid_argument("Standing transition duration is outside the controller range");
    }
  }
  return settings;
}

}  // namespace

class serial_runtime::implementation {
 public:
  implementation(serial_settings settings, plan_executor::position_validator validate_positions,
                 standing_validator validate_standing)
      : settings_(checked_settings(std::move(settings), validate_positions, validate_standing)),
        voltage_limits_(supply_limits_mv(settings_.hardware_config)),
        executor_(settings_.timing, std::move(validate_positions)), validate_standing_(std::move(validate_standing)),
        thread_([this] { run(); }) {}

  ~implementation() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      shutdown_ = true;
    }
    condition_.notify_all();
    thread_.join();
  }

  void enqueue(serial_request request) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (transition_pending_ || shutdown_) {
        return;
      }
      requests_.push_back(std::move(request));
    }
    condition_.notify_one();
  }

  std::optional<std::chrono::nanoseconds> request_transition() {
    std::chrono::nanoseconds stamp;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (settings_.mode != serial_mode::execute || transition_pending_ || shutdown_) {
        return std::nullopt;
      }
      transition_pending_ = true;
      requests_.clear();
      requests_.push_back(transition_request{});
      stamp = next_reference_stamp();
    }
    condition_.notify_one();
    return stamp;
  }

  std::vector<serial_event> drain_events() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<serial_event> drained;
    drained.swap(events_);
    return drained;
  }

 private:
  void emit(serial_event event) {
    std::lock_guard<std::mutex> lock(mutex_);
    events_.push_back(std::move(event));
  }

  void emit(std::optional<execution_report> status) {
    if (status) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!transition_pending_) {
        events_.push_back(std::move(*status));
      }
    }
  }

  // Called with mutex_ held. Epoch timestamps stay ordered across wall clock adjustments.
  std::chrono::nanoseconds next_reference_stamp() {
    last_reference_stamp_ = std::max(wall_now(), last_reference_stamp_ + 1ns);
    return last_reference_stamp_;
  }

  void invalidate_reference() {
    if (!standing_stamp_) {
      return;
    }
    standing_stamp_.reset();
    std::lock_guard<std::mutex> lock(mutex_);
    if (!transition_pending_) {
      events_.push_back(standing_invalidation{next_reference_stamp()});
    }
  }

  bool voltage_valid(std::uint16_t millivolts) const {
    return millivolts >= voltage_limits_.first && millivolts <= voltage_limits_.second;
  }

  void connect(monotonic_time now) {
    if (controller_ || now < retry_at_) {
      return;
    }
    try {
      controller_ = std::make_unique<servo_controller>(settings_.device);
      next_diagnostic_ = now;
      next_link_check_ = now;
      emit(link_diagnostic{diagnostic_level::ok, "UART connected", std::nullopt, std::nullopt});
    } catch (const std::exception& error) {
      disconnect(error.what());
    }
  }

  void disconnect(const std::string& reason) {
    const auto now = steady_clock::now();
    if (transition_active_) {
      finish_transition(reason);
    }
    emit(executor_.fail(reason, now));
    invalidate_reference();
    emit(link_diagnostic{diagnostic_level::error, reason, std::nullopt, std::nullopt});
    controller_.reset();
    retry_at_ = now + reconnect_period;
  }

  position_readback read_positions() {
    const auto started = steady_clock::now();
    const auto positions = controller_->positions(started + query_positions_budget);
    const auto completed = steady_clock::now();
    return {positions, wall_now(), std::chrono::duration_cast<std::chrono::microseconds>(completed - started)};
  }

  void finish_transition(std::string reason, bool ready = false) {
    transition_active_ = false;
    readback_at_.reset();
    std::lock_guard<std::mutex> lock(mutex_);
    const auto stamp = next_reference_stamp();
    if (ready) {
      standing_stamp_ = stamp;
    } else {
      standing_stamp_.reset();
      events_.push_back(standing_invalidation{stamp});
    }
    events_.push_back(standing_confirmation{standing_stamp_, std::move(reason)});
    transition_pending_ = false;
  }

  void advance_transition(monotonic_time now) {
    if (!controller_) {
      finish_transition("Serial controller is disconnected");
      return;
    }
    if (readback_at_) {
      if (now < *readback_at_) {
        return;
      }
      const auto measured = read_positions();
      emit(link_diagnostic{diagnostic_level::ok, "Standing position readback received", std::nullopt, measured});
      try {
        validate_standing_(measured.positions, *settings_.standing_positions);
        executor_.confirm(*settings_.standing_positions);
      } catch (const std::exception& error) {
        finish_transition(error.what());
        return;
      }
      finish_transition("Nominal standing reference ready", true);
      return;
    }
    if (now < board_motion_until_) {
      return;
    }
    const auto voltage = controller_->voltage_mv(now + query_voltage_budget);
    if (!voltage_valid(voltage)) {
      const char* reason = "Supply voltage is outside configured hardware limits";
      emit(link_diagnostic{diagnostic_level::error, reason, voltage, std::nullopt});
      finish_transition(reason);
      return;
    }
    const auto duration = static_cast<std::uint16_t>(settings_.standing_transition_time.count());
    send_move(*settings_.standing_positions, duration, steady_clock::now() + settings_.timing.send_advance);
    readback_at_ = board_motion_until_ + settings_.timing.completion_margin;
  }

  void process_requests(std::deque<serial_request>& requests) {
    for (auto& request : requests) {
      if (std::holds_alternative<transition_request>(request)) {
        standing_stamp_.reset();
        (void)executor_.fail("Standing transition invalidated the previous epoch", steady_clock::now());
        transition_active_ = true;
        continue;
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (transition_pending_) {
          break;
        }
      }
      if (auto* plan = std::get_if<plan_request>(&request)) {
        if (settings_.mode == serial_mode::execute &&
            (!standing_stamp_ || plan->standing_stamp != standing_stamp_ || !controller_)) {
          continue;
        }
        if (settings_.mode == serial_mode::probe) {
          emit(serial_event{execution_report{plan->plan.id, execution_state::rejected, 0, "Execution backend is read-only"}});
          continue;
        }
        emit(executor_.admit(std::move(plan->plan), wall_now(), steady_clock::now()));
        if (!executor_.has_reference()) {
          invalidate_reference();
        }
      } else if (const auto* stop = std::get_if<stop_request>(&request)) {
        emit(executor_.request_stop(stop->id, steady_clock::now()));
        if (!executor_.has_reference()) {
          invalidate_reference();
        }
      }
    }
  }

  monotonic_time send_move(const servo_positions& positions, std::uint16_t duration, monotonic_time deadline) {
    const auto board_time = settings_.timing.wire_time + std::chrono::milliseconds(duration);
    try {
      const auto written_at = controller_->send_move(positions, duration, deadline);
      board_motion_until_ = written_at + board_time;
      return written_at;
    } catch (...) {
      // A late write can report failure even after the final bytes reached the kernel.
      board_motion_until_ = std::max(board_motion_until_, steady_clock::now() + board_time);
      throw;
    }
  }

  bool dispatch_scheduled_frame() {
    // A transition and one frame write have the same commit boundary. Once the
    // request acquires the guard, no frame from the old plan reaches the UART.
    std::lock_guard<std::mutex> lock(mutex_);
    if (transition_pending_ || shutdown_) {
      return true;
    }
    const auto frame = executor_.due(steady_clock::now());
    if (!frame) {
      return false;
    }
    const auto written_at = send_move(frame->positions, frame->move_duration_ms, frame->write_deadline);
    executor_.mark_sent(*frame, written_at);
    return true;
  }

  void diagnose(monotonic_time now) {
    if (!controller_ || now < next_diagnostic_ || now < board_motion_until_) {
      return;
    }
    if (const auto wakeup = executor_.next_wakeup(); wakeup && *wakeup <= now + query_voltage_budget + 5ms) {
      return;
    }
    next_diagnostic_ = now + diagnostic_period;
    const auto millivolts = controller_->voltage_mv(now + query_voltage_budget);
    const bool valid = voltage_valid(millivolts);
    const char* reason = valid ? "Controller voltage readback received" : "Supply voltage is outside configured hardware limits";
    if (!valid && settings_.mode == serial_mode::execute) {
      emit(executor_.fail(reason, now));
      invalidate_reference();
    }
    emit(link_diagnostic{valid ? diagnostic_level::ok : diagnostic_level::warn, reason, millivolts, std::nullopt});
  }

  void run() {
    while (true) {
      std::deque<serial_request> ready;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (shutdown_) {
          break;
        }
        ready.swap(requests_);
      }
      connect(steady_clock::now());
      try {
        process_requests(ready);
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (shutdown_) {
            break;
          }
        }
        const auto now = steady_clock::now();
        if (controller_ && now >= next_link_check_) {
          controller_->check_link();
          next_link_check_ = now + link_check_period;
        }
        if (transition_active_) {
          advance_transition(now);
        } else {
          while (auto status = executor_.advance(now)) {
            emit(std::move(status));
          }
          if (!dispatch_scheduled_frame()) {
            diagnose(now);
          }
        }
      } catch (const std::exception& error) {
        disconnect(error.what());
      }
      auto wakeup = steady_clock::now() + link_check_period;
      if (transition_active_) {
        wakeup = std::min(wakeup, readback_at_.value_or(board_motion_until_));
      } else if (const auto execution = executor_.next_wakeup()) {
        wakeup = std::min(wakeup, *execution);
      }
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait_until(lock, wakeup, [this] { return shutdown_ || !requests_.empty(); });
    }
    controller_.reset();
  }

  serial_settings settings_;
  std::pair<double, double> voltage_limits_;
  plan_executor executor_;
  standing_validator validate_standing_;
  bool transition_active_ = false;
  std::optional<monotonic_time> readback_at_;
  std::optional<std::chrono::nanoseconds> standing_stamp_;
  std::unique_ptr<servo_controller> controller_;
  monotonic_time retry_at_{};
  monotonic_time next_diagnostic_{};
  monotonic_time next_link_check_{};
  monotonic_time board_motion_until_{};
  std::mutex mutex_;
  std::condition_variable condition_;
  bool shutdown_ = false;
  bool transition_pending_ = false;
  std::chrono::nanoseconds last_reference_stamp_{};
  std::deque<serial_request> requests_;
  std::vector<serial_event> events_;
  std::thread thread_;
};

serial_runtime::serial_runtime(serial_settings settings, plan_executor::position_validator validate_positions,
                               standing_validator validate_standing)
    : impl_(std::make_unique<implementation>(std::move(settings), std::move(validate_positions), std::move(validate_standing))) {}

serial_runtime::~serial_runtime() = default;

void serial_runtime::submit(timed_plan plan, std::optional<std::chrono::nanoseconds> standing_stamp) {
  impl_->enqueue(plan_request{std::move(plan), standing_stamp});
}
void serial_runtime::request_stop(std::uint64_t id) { impl_->enqueue(stop_request{id}); }
std::optional<std::chrono::nanoseconds> serial_runtime::request_standing_transition() { return impl_->request_transition(); }
std::vector<serial_event> serial_runtime::drain_events() { return impl_->drain_events(); }

}  // namespace hexapod_hardware
