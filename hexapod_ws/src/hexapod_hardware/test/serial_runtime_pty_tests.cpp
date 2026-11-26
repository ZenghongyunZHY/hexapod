#include "serial_runtime.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {
using namespace std::chrono_literals;
using namespace hexapod_hardware;

std::chrono::nanoseconds wall_now() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch());
}

void require(bool check, const char* reason) {
    if (!check) throw std::runtime_error(reason);
}

template<typename Event>
Event wait_event(serial_runtime& runtime, const char* expected) {
    const auto deadline = std::chrono::steady_clock::now() + 6s;
    while (std::chrono::steady_clock::now() < deadline) {
        for (const auto& event : runtime.drain_events()) {
            if (const auto* match = std::get_if<Event>(&event)) return *match;
        }
        std::this_thread::sleep_for(2ms);
    }
    throw std::runtime_error(std::string("did not receive ") + expected);
}

servo_positions uniform(std::uint16_t position) {
    servo_positions result{};
    result.fill(position);
    return result;
}

std::chrono::nanoseconds transition(serial_runtime& runtime, bool expect_ready) {
    const auto invalidated = runtime.request_standing_transition();
    require(invalidated.has_value(), "standing transition was not accepted");
    require(!runtime.request_standing_transition(), "duplicate standing transition was accepted");
    const auto result = wait_event<standing_confirmation>(runtime, "standing transition result");
    require(result.ready_stamp.has_value() == expect_ready, "unexpected standing verification outcome");
    if (result.ready_stamp) {
        require(*result.ready_stamp > *invalidated, "ready epoch must be newer than invalidation");
    }
    return result.ready_stamp.value_or(*invalidated);
}

void check_plan(serial_runtime& runtime, std::chrono::nanoseconds epoch, std::uint64_t id) {
    const auto motion_start = wall_now() + 350ms;
    runtime.submit({id, motion_start, uniform(550), {{0, 160, uniform(575), false}, {160, 160, uniform(576), true}}}, epoch);
    const auto accepted = wait_event<execution_report>(runtime, "plan acceptance");
    require(accepted.id == id && accepted.state == execution_state::accepted, "a fresh plan must start at the nominal reference");
    const auto completed = wait_event<execution_report>(runtime, "plan completion");
    require(completed.id == id && completed.state == execution_state::completed && completed.completed_segments == 2,
            "expected complete virtual execution and two completed segments");
    require(wall_now() >= motion_start - 120ms + move_wire_time + 320ms + 10ms,
            "reported completion before the earliest estimated board end");
}

void exercise_preemption(serial_runtime& runtime) {
    const auto epoch = transition(runtime, true);
    const auto start = wall_now() + 400ms;
    runtime.submit({100, start, uniform(550), {{0, 500, uniform(560), false}, {500, 500, uniform(565), true}}}, epoch);
    auto status = wait_event<execution_report>(runtime, "active plan acceptance");
    require(status.id == 100 && status.state == execution_state::accepted, "active plan was not accepted");
    runtime.submit({101, start + 1000ms, uniform(565), {{0, 160, uniform(590), true}}}, epoch);
    status = wait_event<execution_report>(runtime, "following plan acceptance");
    require(status.id == 101 && status.state == execution_state::accepted, "following plan was not queued");

    while (wall_now() < start + 60ms) {
        std::this_thread::sleep_for(2ms);
    }
    const auto invalidated = runtime.request_standing_transition();
    require(invalidated && *invalidated > epoch, "standing transition did not preempt active motion");
    require(!runtime.request_standing_transition(), "duplicate high-priority transition was accepted");
    runtime.request_stop(103);
    runtime.submit({900, wall_now() + 1s, uniform(550), {{0, 160, uniform(591), true}}}, epoch);
    const auto ready = wait_event<standing_confirmation>(runtime, "preempting standing completion");
    require(ready.ready_stamp && *ready.ready_stamp > *invalidated, "preempting transition failed to become ready");

    // The stale plan has the correct nominal positions and a future start time.
    // Its previous epoch alone must keep it from invalidating the fresh plan.
    runtime.submit({901, wall_now() + 900ms, uniform(550), {{0, 160, uniform(592), true}}}, epoch);
    check_plan(runtime, *ready.ready_stamp, 103);
    const auto invalid = wait_event<standing_invalidation>(runtime, "UART loss invalidation");
    require(invalid.stamp > *ready.ready_stamp, "disconnect did not revoke readiness");
}

void exercise_failure(serial_runtime& runtime) {
    const auto failed = transition(runtime, false);
    runtime.submit({900, wall_now() + 250ms, uniform(550), {{0, 160, uniform(593), true}}}, failed);
    std::this_thread::sleep_for(500ms);
    for (const auto& event : runtime.drain_events()) {
        require(!std::holds_alternative<execution_report>(event), "failed transition accepted a motion request");
    }
    const auto retry = transition(runtime, true);
    require(retry > failed, "retry did not create a new readiness epoch");
    check_plan(runtime, retry, 110);
    runtime.submit({111, wall_now() + 350ms, uniform(576), {{0, 160, uniform(577), true}}}, retry);
    const auto accepted = wait_event<execution_report>(runtime, "continuation acceptance");
    require(accepted.id == 111 && accepted.state == execution_state::accepted,
            "continuation did not use the preceding plan's final target");
    const auto completed = wait_event<execution_report>(runtime, "continuation completion");
    require(completed.id == 111 && completed.state == execution_state::completed && completed.completed_segments == 1,
            "continuation did not complete its UART action");
}

void exercise_timing_failure(serial_runtime& runtime, bool write_timeout) {
    const auto epoch = transition(runtime, true);
    runtime.submit({120, wall_now() + 400ms, uniform(550), {
        {0, 240, uniform(580), false}, {240, 240, uniform(581), false}, {480, 240, uniform(582), true}
    }}, epoch);
    const auto accepted = wait_event<execution_report>(runtime, "relative plan acceptance");
    require(accepted.id == 120 && accepted.state == execution_state::accepted, "relative plan was not accepted");
    require(accepted.maximum_finish_delay_ms > 0, "relative plan acceptance did not report its delay budget");
    const auto failed = wait_event<execution_report>(runtime, "relative plan failure");
    require(failed.id == 120 && failed.state == execution_state::failed, "expected a relative plan failure");
    const auto expected_reason = write_timeout ? "UART write deadline expired" : "late";
    require(failed.reason.find(expected_reason) != std::string::npos, failed.reason.c_str());
    require(failed.completed_segments == (write_timeout ? 0U : 1U),
            "failure did not report the confirmed prefix of completed frames");
}

void exercise(const std::string& port, const std::string& hardware_config, const std::string& mode) {
    if (mode == "probe") {
        serial_runtime runtime({serial_mode::probe, port, hardware_config, {120ms, move_wire_time, 10ms}},
                               [](const servo_positions&) {});
        auto diagnostic = wait_event<link_diagnostic>(runtime, "connection or voltage diagnostic");
        if (!diagnostic.millivolts) {
            diagnostic = wait_event<link_diagnostic>(runtime, "voltage diagnostic");
        }
        require(diagnostic.millivolts == 7400, "probe did not report controller voltage");
        require(!runtime.request_standing_transition(), "probe accepted standing transition");
        runtime.submit({130, wall_now() + 350ms, uniform(550), {{0, 160, uniform(577), true}}});
        const auto rejected = wait_event<execution_report>(runtime, "read-only plan refusal");
        require(rejected.id == 130 && rejected.state == execution_state::rejected, "probe accepted a motion plan");
        return;
    }
    const auto timing_failure = mode == "write_timeout" || mode == "late_wake";
    const auto completion_margin = timing_failure ? 90ms : 10ms;
    serial_runtime runtime({serial_mode::execute, port, hardware_config, {120ms, move_wire_time, completion_margin},
                           uniform(550), 280ms}, [](const servo_positions&) {},
                           [](const servo_positions& measured, const servo_positions& nominal) {
                               auto allowed = nominal;
                               ++allowed.front();
                               require(measured == allowed, "virtual standing verification rejected the readback");
                           });
    if (mode == "failure") {
        exercise_failure(runtime);
    } else if (mode == "preempt") {
        exercise_preemption(runtime);
    } else if (timing_failure) {
        exercise_timing_failure(runtime, mode == "write_timeout");
    } else {
        throw std::runtime_error("unknown virtual UART test mode");
    }
}
}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc != 4) {
            throw std::runtime_error("expected PTY device and hardware configuration");
        }
        exercise(argv[1], argv[2], argv[3]);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
