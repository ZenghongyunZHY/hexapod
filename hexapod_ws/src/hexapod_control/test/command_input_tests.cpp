#include "command_input.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {

using namespace hexapod_control;
using namespace std::chrono_literals;

constexpr command_components centered{};
constexpr command_components forward{1.0, 0.0, 0.0};

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

template <class Operation>
void require_invalid(Operation operation, const char* message) {
    try {
        operation();
    } catch (const std::invalid_argument&) {
        return;
    }
    throw std::runtime_error(message);
}

void inspect_calibration() {
    joystick_config calibration{{{{1, 100.0, 500.0, 700.0, -1.0}, {0, -1.0, 0.0, 1.0, 1.0}, {2, -1.0, 0.0, 1.0, 1.0}}}, 0.05};
    calibration.validate();
    require(calibration.read({0.0F, 300.0F, 0.0F}) == command_components{0.5, 0.0, 0.0}, "Lower span or forward sign differs");
    require(calibration.read({0.0F, 600.0F, 0.0F}) == command_components{-0.5, 0.0, 0.0}, "Upper span or forward sign differs");
    require(calibration.read({0.0F, 800.0F, -0.04F}) == command_components{-1.0, 0.0, 0.0}, "Raw range clipping differs");
    const auto diagonal = calibration.read({1.0F, 700.0F, -0.04F});
    require(std::abs(std::hypot(diagonal[0], diagonal[1]) - 1.0) < 1e-12 && diagonal[0] == -diagonal[1] && diagonal[2] == 0.0,
        "Joystick radial limit or independent deadzone differs");
    require(calibration.read({0.03F, 504.0F, 0.03F}) == centered, "Neutral was not preserved");
    require(calibration.read({0.01F, 500.0F, -1.0F}) == command_components{0.0, 0.0, -1.0}, "Yaw deadzone is coupled to translation");
    require_invalid([&] { calibration.read({1.0F}); }, "Missing Joy axes were accepted");
    require_invalid([&] { calibration.read({0.0F, std::numeric_limits<float>::quiet_NaN(), 0.0F}); }, "Nonfinite Joy sample was accepted");
    auto invalid = calibration;
    invalid.axes[0].center = invalid.axes[0].minimum;
    require_invalid([&] { invalid.validate(); }, "Invalid span was accepted");
    invalid = calibration;
    invalid.axes[0].index = invalid.axes[1].index;
    require_invalid([&] { invalid.validate(); }, "Ambiguous axis assignment was accepted");
}

void inspect_normalization_and_walk() {
    require(normalize_teleop({0.6, 0.8, -0.5}) == command_components{0.6, 0.8, -0.5}, "Unit-circle boundary changed");
    const auto diagonal = normalize_teleop({2.0, -1.0, -2.0});
    require(std::abs(std::hypot(diagonal[0], diagonal[1]) - 1.0) < 1e-12 && diagonal[0] == -2.0 * diagonal[1], "Radial cap changed direction");
    require(diagonal[2] == -1.0, "Yaw did not clamp independently");
    const auto overflow_risk = normalize_teleop({1.7e308, 1.7e308, 1.7e308});
    require(std::abs(overflow_risk[0] - std::sqrt(0.5)) < 1e-12 && overflow_risk[0] == overflow_risk[1], "Large input changed direction");
    require_invalid([] { normalize_teleop({std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}); }, "Nonfinite TELEOP was accepted");
    require_walk({0.003, 0.0, -0.01}, 4);
    require_invalid([] { require_walk(centered, 4); }, "A stationary WALK_CYCLES was accepted");
    require_invalid([] { require_walk({0.003, 0.0, 0.0}, 0); }, "Zero walking cycles were accepted");
    require_invalid([] { require_walk({0.003, std::numeric_limits<double>::infinity(), 0.0}, 1); }, "Nonfinite walk was accepted");
}

void inspect_time() {
    require(require_fresh_stamp(100, 10, 100s + 509ns, 500ns) == 100s + 10ns, "Sample time was refreshed or converted incorrectly");
    require_invalid([] { require_fresh_stamp(100, 10, 100s + 510ns, 500ns); }, "Expired sample was accepted");
    require_invalid([] { require_fresh_stamp(100, 10, 100s + 9ns, 500ns); }, "Future sample was accepted");
    require_invalid([] { require_fresh_stamp(0, 0, 100ns, 500ns); }, "Missing sample timestamp was accepted");
    require_invalid([] { require_fresh_stamp(99, 1'000'000'000, 100s, 500ns); }, "Invalid nanosecond component was accepted");
    command_sequence original;
    require(original.next(10'000ns) == 10'000 && original.next(10'000ns) == 10'001, "Sequence is not strictly increasing");
    command_sequence restarted;
    require(restarted.next(12'000ns) == 12'000, "Receiver restart reset sequence to a small value");
}

void inspect_reconnect_and_stop() {
    teleop_gate gate(500ms);
    require(!gate.accept(teleop_source::joystick, 10ms, 10ms, forward), "A click-in with a held stick started motion");
    require(gate.accept(teleop_source::joystick, 20ms, 20ms, centered), "Fresh center did not arm the joystick");
    require(gate.accept(teleop_source::joystick, 30ms, 30ms, forward), "Armed joystick did not send motion");
    require(!gate.accept(teleop_source::joystick, 30ms, 40ms, forward), "Duplicate joystick sample was accepted");
    require(!gate.accept(teleop_source::request, 40ms, 40ms, forward), "Second source took over a held joystick");
    require(!gate.accept(teleop_source::joystick, 530ms, 530ms, forward), "Reconnect replayed held input");
    require(gate.accept(teleop_source::joystick, 540ms, 540ms, centered), "Center did not rearm after lost samples");
    require(gate.accept(teleop_source::joystick, 550ms, 550ms, centered), "Online neutral sample was suppressed");
    require(gate.accept(teleop_source::request, 560ms, 560ms, centered), "Second source could not arm after timeout");
    require(gate.accept(teleop_source::request, 570ms, 570ms, forward), "Second source could not take over after center");
    gate.stop();
    require(!gate.accept(teleop_source::request, 580ms, 580ms, forward), "A stream bypassed an explicit stop");
    require(!gate.accept(teleop_source::joystick, 590ms, 590ms, forward), "Held joystick bypassed an explicit stop");
    require(gate.accept(teleop_source::request, 600ms, 600ms, centered), "New neutral did not rearm after stop");
}

void inspect_finite_walk() {
    teleop_gate gate(500ms);
    require(gate.accept(teleop_source::request, 10ms, 10ms, forward), "Initial request stream was blocked");
    gate.requested_walk(1);
    gate.planner_status(1, "rejected");
    require(gate.accept(teleop_source::request, 20ms, 20ms, forward), "Rejected WALK_CYCLES interrupted active teleop");
    gate.requested_walk(2);
    gate.planner_status(2, "planned");
    gate.requested_walk(3);
    gate.planner_status(3, "rejected");
    gate.planner_status(2, "interrupted");
    require(!gate.accept(teleop_source::request, 30ms, 30ms, centered), "Premature center rearmed input during a finite task");
    gate.planner_status(4, "standing");
    require(!gate.accept(teleop_source::request, 40ms, 40ms, forward), "Held intent restarted after finite task");
    require(gate.accept(teleop_source::request, 50ms, 50ms, centered), "Center after standing did not rearm");
    require(gate.accept(teleop_source::request, 60ms, 60ms, forward), "New intent after finite task did not restart");
}

void inspect_reference_loss() {
    teleop_gate gate(500ms);
    require(gate.accept(teleop_source::request, 10ms, 10ms, forward), "Initial request stream was blocked");
    gate.planner_status(1, "fault");
    require(!gate.accept(teleop_source::request, 20ms, 20ms, centered), "Center before confirmed reference rearmed motion");
    require(!gate.accept(teleop_source::request, 30ms, 30ms, forward), "Intent leaked while standing reference was invalid");
    gate.planner_status(0, "standing");
    require(!gate.accept(teleop_source::request, 40ms, 40ms, forward), "Held intent restarted after a reference loss");
    require(gate.accept(teleop_source::request, 50ms, 50ms, centered), "Center after reference confirmation did not rearm");
    require(gate.accept(teleop_source::request, 60ms, 60ms, forward), "Fresh intent after reference confirmation was blocked");
}

}  // namespace

int main() {
    try {
        inspect_calibration();
        inspect_normalization_and_walk();
        inspect_time();
        inspect_reconnect_and_stop();
        inspect_finite_walk();
        inspect_reference_loss();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
