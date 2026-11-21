#include "plan_executor.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace {

using namespace std::chrono_literals;
using namespace hexapod_hardware;

const monotonic_time steady_epoch{};
constexpr auto wall_epoch = 1000s;
constexpr auto first_start = 250ms;

void require(bool condition, const char* description) {
  if (!condition) throw std::runtime_error(description);
}

servo_positions positions(std::uint16_t value) {
  servo_positions result{};
  result.fill(value);
  return result;
}

plan_executor make_executor() {
  return plan_executor(execution_options{}, [](const servo_positions& target) {
    for (const auto value : target) {
      if (value < 140 || value > 860) throw std::out_of_range("Position outside installed limits");
    }
  });
}

timed_plan make_plan(std::uint64_t id = 1) {
  return {id, wall_epoch + first_start, positions(400), {
      {0, 200, positions(450), false}, {200, 200, positions(500), true},
      {400, 200, positions(550), false}, {600, 200, positions(600), true},
      {800, 200, positions(400), false}}};
}

void expect(const std::optional<execution_report>& result, std::uint64_t id, execution_state state, std::uint32_t completed) {
  require(result && result->id == id && result->state == state && result->completed_segments == completed, "Unexpected execution report");
}

void send_frame(plan_executor& executor, std::uint64_t id, std::size_t index, std::chrono::milliseconds expected_at) {
  const auto frame = executor.due(steady_epoch + expected_at - 90ms);
  require(frame && frame->id == id && frame->index == index, "Expected dispatch at its early wire time");
  require(frame->write_deadline == steady_epoch + expected_at - move_wire_time, "Frame did not reserve full wire time");
  executor.mark_sent(*frame);
}

void full_validation() {
  auto executor = make_executor();
  expect(executor.admit(make_plan(), wall_epoch, steady_epoch), 1, execution_state::rejected, 0);
  require(!executor.has_plan() && !executor.has_reference(), "Unconfirmed stance cannot admit motion");
  executor.confirm(positions(400));
  auto invalid = make_plan(2);
  invalid.segments.back().positions = positions(900);
  expect(executor.admit(std::move(invalid), wall_epoch, steady_epoch), 2, execution_state::rejected, 0);
  require(!executor.has_plan() && !executor.has_reference(), "Invalid final frame must reject the entire plan");

  executor.confirm(positions(400));
  auto short_frame = make_plan(3);
  short_frame.segments.front().move_duration_ms = 50;
  expect(executor.admit(std::move(short_frame), wall_epoch, steady_epoch), 3, execution_state::rejected, 0);

  executor.confirm(positions(400));
  auto gap = make_plan(4);
  gap.segments.back().start_offset_ms += 1;
  expect(executor.admit(std::move(gap), wall_epoch, steady_epoch), 4, execution_state::rejected, 0);

  executor.confirm(positions(400));
  expect(executor.admit(make_plan(5), wall_epoch + 160ms, steady_epoch), 5, execution_state::rejected, 0);
  require(!executor.due(steady_epoch), "Invalid plans must never release a frame");
}

void consecutive_batches() {
  auto executor = make_executor();
  executor.confirm(positions(400));
  auto one = make_plan();
  one.segments.resize(2);
  expect(executor.admit(std::move(one), wall_epoch, steady_epoch), 1, execution_state::accepted, 0);
  require(!executor.due(steady_epoch + first_start - 91ms), "First frame must wait for send time");
  send_frame(executor, 1, 0, 250ms);

  auto two = make_plan(2);
  two.initial_positions = positions(500);
  two.motion_start += 400ms;
  two.segments.resize(1);
  // A wall-clock step after the first plan must not shift the contiguous second plan.
  expect(executor.admit(std::move(two), wall_epoch + 5s, steady_epoch + 200ms), 2, execution_state::accepted, 0);
  send_frame(executor, 1, 1, 450ms);
  send_frame(executor, 2, 0, 650ms);
  require(!executor.advance(steady_epoch + 654ms), "Completion cannot be published before the nominal end plus margin");
  expect(executor.advance(steady_epoch + 655ms), 1, execution_state::completed, 2);
  expect(executor.advance(steady_epoch + 855ms), 2, execution_state::completed, 1);
  require(!executor.has_plan() && executor.has_reference(), "Completion should retain the final command reference");
}

void reject_bad_successor() {
  auto executor = make_executor();
  executor.confirm(positions(400));
  auto one = make_plan();
  one.segments.resize(2);
  expect(executor.admit(std::move(one), wall_epoch, steady_epoch), 1, execution_state::accepted, 0);
  auto two = make_plan(2);
  two.initial_positions = positions(500);
  two.motion_start += 401ms;
  expect(executor.admit(std::move(two), wall_epoch, steady_epoch), 2, execution_state::rejected, 0);
  require(!executor.has_plan() && !executor.has_reference(), "Rejected continuation should clear the shared execution state");
}

void stop_at_boundary() {
  auto executor = make_executor();
  executor.confirm(positions(400));
  expect(executor.admit(make_plan(), wall_epoch, steady_epoch), 1, execution_state::accepted, 0);
  auto following = make_plan(2);
  following.motion_start += 1000ms;
  expect(executor.admit(std::move(following), wall_epoch + 150ms, steady_epoch + 150ms), 2, execution_state::accepted, 0);
  send_frame(executor, 1, 0, 250ms);
  send_frame(executor, 1, 1, 450ms);
  require(!executor.request_stop(1, steady_epoch + 550ms), "Intermediate stop cannot claim completion");
  require(!executor.due(steady_epoch + 560ms), "Stop must suppress the next cycle before its first write");
  expect(executor.advance(steady_epoch + 655ms), 1, execution_state::interrupted, 2);
  require(!executor.has_plan(), "Stopping the active plan must also withdraw its dependent successor");

  auto next = make_plan(3);
  next.initial_positions = positions(500);
  next.motion_start += 800ms;
  expect(executor.admit(std::move(next), wall_epoch + 655ms, steady_epoch + 655ms), 3, execution_state::accepted, 0);
  expect(executor.request_stop(3, steady_epoch + 656ms), 3, execution_state::interrupted, 0);
}

void honor_transmitted_frames() {
  auto executor = make_executor();
  executor.confirm(positions(400));
  expect(executor.admit(make_plan(), wall_epoch, steady_epoch), 1, execution_state::accepted, 0);
  send_frame(executor, 1, 0, 250ms);
  send_frame(executor, 1, 1, 450ms);
  send_frame(executor, 1, 2, 650ms);
  require(!executor.request_stop(1, steady_epoch + 600ms), "A frame already sent ahead makes the previous boundary unavailable");
  send_frame(executor, 1, 3, 850ms);
  require(!executor.due(steady_epoch + 970ms), "Recovery must remain unsent if the walking boundary can still stop");
  expect(executor.advance(steady_epoch + 1055ms), 1, execution_state::interrupted, 4);

  executor.confirm(positions(400));
  expect(executor.admit(make_plan(2), wall_epoch, steady_epoch), 2, execution_state::accepted, 0);
  for (std::size_t i = 0; i < 5; ++i) send_frame(executor, 2, i, 250ms + i * 200ms);
  require(!executor.request_stop(2, steady_epoch + 1000ms), "A recovery target has already been written");
  expect(executor.advance(steady_epoch + 1255ms), 2, execution_state::completed, 5);
}

void reordered_stop_and_fault() {
  auto executor = make_executor();
  executor.confirm(positions(400));
  require(!executor.request_stop(1, steady_epoch), "Stop-before-plan should be remembered without preemptive status");
  expect(executor.admit(make_plan(), wall_epoch, steady_epoch), 1, execution_state::interrupted, 0);
  require(!executor.admit(make_plan(), wall_epoch, steady_epoch), "Repeated plan must not dispatch a second time");
  expect(executor.admit(make_plan(2), wall_epoch, steady_epoch), 2, execution_state::accepted, 0);
  try {
    executor.due(steady_epoch + first_start);
    throw std::logic_error("Late dispatch should fail instead of writing");
  } catch (const std::runtime_error&) {
    expect(executor.fail("Late frame", steady_epoch + first_start), 2, execution_state::failed, 0);
  }
  require(!executor.has_plan() && !executor.has_reference(), "Failure must invalidate motion and reference together");
  executor.confirm(positions(400));
  expect(executor.fail("Link lost while idle", steady_epoch), 0, execution_state::failed, 0);
  expect(executor.admit(make_plan(3), wall_epoch, steady_epoch), 3, execution_state::rejected, 0);
}

}  // namespace

int main() {
  try {
    full_validation();
    consecutive_batches();
    reject_bad_successor();
    stop_at_boundary();
    honor_transmitted_frames();
    reordered_stop_and_fault();
    std::cout << "Serial execution admission, clocks, boundary stop and fault tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
