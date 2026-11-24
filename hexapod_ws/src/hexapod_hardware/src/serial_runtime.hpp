#pragma once

#include "plan_executor.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace hexapod_hardware {

enum class serial_mode { preview, probe, execute };
enum class diagnostic_level : std::uint8_t { ok, warn, error };

struct serial_settings {
  serial_mode mode;
  std::string device;
  std::string hardware_config;
  std::optional<servo_positions> preview_positions;
  execution_options timing;
  std::optional<servo_positions> standing_positions = std::nullopt;
  std::chrono::milliseconds standing_transition_time{2000};
};

struct position_readback {
  servo_positions positions;
  std::chrono::nanoseconds received_at;
  std::chrono::microseconds query_duration;
};

struct standing_confirmation {
  std::optional<std::chrono::nanoseconds> ready_stamp;
  std::string reason;
};

struct standing_invalidation {
  std::chrono::nanoseconds stamp;
};

struct link_diagnostic {
  diagnostic_level level;
  std::string reason;
  std::optional<std::uint16_t> millivolts;
  std::optional<position_readback> readback;
};

using serial_event = std::variant<execution_report, standing_confirmation, standing_invalidation, link_diagnostic>;

// The only owner of the serial controller is the work thread. No ROS executor waits on UART I/O.
class serial_runtime {
 public:
  using standing_validator = std::function<void(const servo_positions&, const servo_positions&)>;

  serial_runtime(serial_settings settings, plan_executor::position_validator validate_positions,
                 standing_validator validate_standing = {});
  ~serial_runtime();

  serial_runtime(const serial_runtime&) = delete;
  serial_runtime& operator=(const serial_runtime&) = delete;

  void submit(timed_plan plan, std::optional<std::chrono::nanoseconds> standing_stamp = std::nullopt);
  void request_stop(std::uint64_t id);
  void request_confirmation();
  std::optional<std::chrono::nanoseconds> request_standing_transition();
  std::vector<serial_event> drain_events();

 private:
  class implementation;
  std::unique_ptr<implementation> impl_;
};

}  // namespace hexapod_hardware
