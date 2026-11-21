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
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < deadline) {
        for (const auto& event : runtime.drain_events()) {
            if (const auto* match = std::get_if<Event>(&event)) return *match;
        }
        std::this_thread::sleep_for(2ms);
    }
    throw std::runtime_error(std::string("did not receive ") + expected);
}

void exercise(const std::string& port, const std::string& hardware_config) {
    serial_runtime runtime({serial_mode::execute, port, hardware_config, std::nullopt, {120ms, move_wire_time, 10ms}},
                           [](const servo_positions&) {});
    runtime.request_confirmation();
    const auto standing = wait_event<standing_confirmation>(runtime, "standing confirmation");
    require(standing.reference.has_value(), "runtime refused the virtual standing position");
    auto targets = standing.reference->positions;
    targets.front() += 1;
    const auto motion_start = wall_now() + 350ms;
    runtime.submit({100, motion_start, standing.reference->positions, {{0, 120, targets, true}}});
    const auto accepted = wait_event<execution_report>(runtime, "plan acceptance");
    require(accepted.id == 100 && accepted.state == execution_state::accepted, "expected acceptance of the virtual plan");
    const auto completed = wait_event<execution_report>(runtime, "plan completion");
    require(completed.id == 100 && completed.state == execution_state::completed && completed.completed_segments == 1,
            "expected complete virtual execution and a single completed segment");
    require(wall_now() >= motion_start + 120ms, "reported completion before the nominal plan end");

    const auto failed = wait_event<execution_report>(runtime, "idle reference loss after disconnect");
    require(failed.id == 0 && failed.state == execution_state::failed, "UART loss did not invalidate the idle reference");
}
}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc != 3) throw std::runtime_error("expected PTY device and hardware configuration");
        exercise(argv[1], argv[2]);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
