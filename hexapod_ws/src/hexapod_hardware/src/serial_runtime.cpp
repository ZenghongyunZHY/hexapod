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
struct confirmation_request {};
using serial_request = std::variant<timed_plan, stop_request, confirmation_request>;

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

}  // namespace

class serial_runtime::implementation {
 public:
  implementation(serial_settings settings, plan_executor::position_validator validate_positions)
      : settings_(std::move(settings)), voltage_limits_(supply_limits_mv(settings_.hardware_config)),
        executor_(settings_.timing, std::move(validate_positions)), preview_positions_(settings_.preview_positions),
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
      requests_.push_back(std::move(request));
    }
    condition_.notify_one();
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
      emit(serial_event{std::move(*status)});
    }
  }

  bool voltage_valid(std::uint16_t millivolts) const {
    return millivolts >= voltage_limits_.first && millivolts <= voltage_limits_.second;
  }

  void connect(monotonic_time now) {
    if (settings_.mode == serial_mode::preview || controller_ || now < retry_at_) {
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
    emit(executor_.fail(reason, now));
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

  void confirm() {
    if (settings_.mode == serial_mode::probe || executor_.has_plan()) {
      emit(standing_confirmation{std::nullopt, "Standing confirmation requires idle preview or execute mode"});
      return;
    }
    if (settings_.mode == serial_mode::preview) {
      if (!preview_positions_) {
        emit(standing_confirmation{std::nullopt, "Supply explicit preview_positions before confirming a simulated stance"});
        return;
      }
      complete_confirmation({*preview_positions_, wall_now(), 0us});
      return;
    }
    if (!controller_) {
      emit(standing_confirmation{std::nullopt, "Serial controller is disconnected"});
      return;
    }
    if (steady_clock::now() < wire_busy_until_) {
      emit(standing_confirmation{std::nullopt, "Previous UART packet is still draining"});
      return;
    }
    try {
      const auto millivolts = controller_->voltage_mv(steady_clock::now() + query_voltage_budget);
      if (!voltage_valid(millivolts)) {
        const std::string reason = "Supply voltage is outside configured hardware limits";
        emit(executor_.fail(reason, steady_clock::now()));
        emit(link_diagnostic{diagnostic_level::error, reason, millivolts, std::nullopt});
        emit(standing_confirmation{std::nullopt, reason});
        return;
      }
      const auto observation = read_positions();
      emit(link_diagnostic{diagnostic_level::ok, "Full servo position readback received", millivolts, observation});
      complete_confirmation(observation);
    } catch (const std::exception& error) {
      disconnect(error.what());
      emit(standing_confirmation{std::nullopt, error.what()});
    }
  }

  void complete_confirmation(const position_readback& readback) {
    try {
      executor_.confirm(readback.positions);
      emit(standing_confirmation{readback, "Standing reference published"});
    } catch (const std::exception& error) {
      emit(executor_.fail(error.what(), steady_clock::now()));
      emit(standing_confirmation{std::nullopt, error.what()});
    }
  }

  void process_requests(std::deque<serial_request>& requests) {
    for (auto& request : requests) {
      if (auto* plan = std::get_if<timed_plan>(&request)) {
        if (settings_.mode == serial_mode::probe || (settings_.mode == serial_mode::execute && !controller_)) {
          emit(serial_event{execution_report{plan->id, execution_state::rejected, 0, "Execution backend is read-only or disconnected"}});
        } else {
          const auto system = wall_now();
          const auto steady = steady_clock::now();
          emit(executor_.admit(std::move(*plan), system, steady));
        }
      } else if (const auto* stop = std::get_if<stop_request>(&request)) {
        emit(executor_.request_stop(stop->id, steady_clock::now()));
      } else {
        confirm();
      }
    }
  }

  void send_frame(const frame_dispatch& frame) {
    if (settings_.mode == serial_mode::preview) {
      preview_positions_ = frame.positions;
    } else {
      controller_->send_move(frame.positions, frame.move_duration_ms, frame.write_deadline);
      wire_busy_until_ = steady_clock::now() + settings_.timing.wire_time;
    }
    executor_.mark_sent(frame);
  }

  void diagnose(monotonic_time now) {
    if (!controller_ || now < next_diagnostic_ || now < wire_busy_until_) {
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
    }
    std::optional<position_readback> readback;
    if (settings_.mode == serial_mode::probe) {
      readback = read_positions();
    }
    emit(link_diagnostic{valid ? diagnostic_level::ok : diagnostic_level::warn, reason, millivolts, readback});
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
        while (auto status = executor_.advance(now)) {
          emit(serial_event{std::move(*status)});
        }
        if (const auto frame = executor_.due(now)) {
          send_frame(*frame);
        } else {
          diagnose(now);
        }
      } catch (const std::exception& error) {
        disconnect(error.what());
      }
      auto wakeup = steady_clock::now() + link_check_period;
      if (const auto execution = executor_.next_wakeup()) {
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
  std::optional<servo_positions> preview_positions_;
  std::unique_ptr<servo_controller> controller_;
  monotonic_time retry_at_{};
  monotonic_time next_diagnostic_{};
  monotonic_time next_link_check_{};
  monotonic_time wire_busy_until_{};
  std::mutex mutex_;
  std::condition_variable condition_;
  bool shutdown_ = false;
  std::deque<serial_request> requests_;
  std::vector<serial_event> events_;
  std::thread thread_;
};

serial_runtime::serial_runtime(serial_settings settings, plan_executor::position_validator validate_positions)
    : impl_(std::make_unique<implementation>(std::move(settings), std::move(validate_positions))) {}

serial_runtime::~serial_runtime() = default;

void serial_runtime::submit(timed_plan plan) { impl_->enqueue(std::move(plan)); }
void serial_runtime::request_stop(std::uint64_t id) { impl_->enqueue(stop_request{id}); }
void serial_runtime::request_confirmation() { impl_->enqueue(confirmation_request{}); }
std::vector<serial_event> serial_runtime::drain_events() { return impl_->drain_events(); }

}  // namespace hexapod_hardware
