#include "serial_runtime.hpp"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {

using namespace hexapod_hardware;
using namespace std::chrono_literals;

void require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

template <typename Event>
Event wait_for(serial_runtime& runtime, const char* description) {
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline) {
    for (const auto& event : runtime.drain_events()) {
      if (const auto* result = std::get_if<Event>(&event)) {
        return *result;
      }
    }
    std::this_thread::sleep_for(1ms);
  }
  throw std::runtime_error(description);
}

timed_plan make_plan(std::uint64_t id, const servo_positions& start, const servo_positions& end) {
  const auto start_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()) + 250ms;
  return {id, start_time, start, {{0, 120, end, true}}};
}

void exercise_preview(const char* config) {
  const servo_positions standing{500, 550, 200, 500, 550, 200, 500, 550, 200, 500, 450, 800, 500, 450, 800, 500, 450, 800};
  auto next = standing;
  next.front() = 501;
  serial_settings settings{serial_mode::preview, "", config, standing, {70ms, 0ns, 5ms}};
  serial_runtime runtime(settings, [](const servo_positions&) {});

  runtime.submit(make_plan(1, standing, next));
  const auto rejected = wait_for<execution_report>(runtime, "Unconfirmed plan did not produce a rejection");
  require(rejected.id == 1 && rejected.state == execution_state::rejected, "Unconfirmed plan was not rejected");
  runtime.request_confirmation();
  const auto confirmed = wait_for<standing_confirmation>(runtime, "Standing confirmation did not complete");
  require(confirmed.ready_stamp.has_value(), "Preview standing did not use the explicit fixture");

  const auto plan = make_plan(2, standing, next);
  const auto earliest_end = plan.motion_start - settings.timing.send_advance + 120ms + settings.timing.completion_margin;
  runtime.submit(plan);
  const auto accepted = wait_for<execution_report>(runtime, "Confirmed plan did not produce an acknowledgement");
  require(accepted.id == 2 && accepted.state == execution_state::accepted, "Confirmed plan was not accepted");
  const auto completed = wait_for<execution_report>(runtime, "Preview did not report completion");
  require(completed.id == 2 && completed.state == execution_state::completed && completed.completed_segments == 1,
      "Preview reported an incomplete plan");
  const auto observed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch());
  require(observed >= earliest_end, "Preview reported completion before the virtual write and move duration elapsed");

  runtime.request_confirmation();
  const auto after_plan = wait_for<standing_confirmation>(runtime, "Final preview position could not be read");
  require(after_plan.ready_stamp && *after_plan.ready_stamp > *confirmed.ready_stamp, "Preview did not refresh standing readiness");
  runtime.submit(make_plan(3, next, standing));
  const auto returned = wait_for<execution_report>(runtime, "Preview did not accept a plan starting at its last target");
  require(returned.id == 3 && returned.state == execution_state::accepted, "Preview backend did not apply the planned frame");
}

}  // namespace

int main(int argc, char** argv) {
  try {
    require(argc == 2, "Pass the hardware configuration path");
    exercise_preview(argv[1]);
    std::cout << "Preview serial worker acceptance and completion passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
