#include "hexapod_demo/demo.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace hexapod_demo {
namespace {
volatile std::sig_atomic_t interrupted = 0;
void stop_signal(int) { interrupted = 1; }
struct Stopped {};
struct Options { bool execute = false, probe = false, packets = false; std::string port = "/dev/ttyS0", frames; };
int integer(const std::string &value) {
  std::size_t count = 0; const int result = std::stoi(value, &count);
  if (count != value.size()) throw std::runtime_error("Invalid integer in frame table: " + value);
  return result;
}
void voltage_check(int mv) {
  std::cout << "BATTERY " << mv << " mV\n";
  // User-authorized allowance for this demo; the datasheet upper limit remains 8.4 V.
  if (mv < 6000 || mv > 8700) throw std::runtime_error("Battery outside demo allowance 6.0..8.7 V; no further frame sent");
}
void check_stop(std::chrono::steady_clock::time_point deadline) {
  if (interrupted) throw Stopped{};
  if (std::chrono::steady_clock::now() > deadline) throw std::runtime_error("90-second run deadline exceeded");
}
void wait_frame(int duration, std::chrono::steady_clock::time_point deadline) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(duration);
  while (std::chrono::steady_clock::now() < end) {
    check_stop(deadline); std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  check_stop(deadline);
}
void print_pose(const Pose &pose) {
  for (int i = 0; i < 18; ++i) std::cout << (i ? " " : "") << i + 1 << ':' << pose[i];
  std::cout << '\n';
}
}  // namespace

Frames load_frames(const std::string &path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot open frame table: " + path);
  Frames frames; std::string line; bool header = false;
  while (std::getline(input, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty() || line[0] == '#') continue;
    std::stringstream stream(line); std::vector<std::string> fields; std::string field;
    while (std::getline(stream, field, ',')) fields.push_back(field);
    if (!header) {
      if (fields.size() != 21 || fields[0] != "name" || fields[1] != "move_ms" || fields[2] != "settle_ms")
        throw std::runtime_error("Invalid frame-table header");
      for (int id = 1; id <= 18; ++id) if (fields[id + 2] != "id" + std::to_string(id))
        throw std::runtime_error("Frame columns must be ID1 through ID18, in order");
      header = true; continue;
    }
    if (fields.size() != 21 || fields[0].empty()) throw std::runtime_error("Each frame needs exactly 18 positions");
    Frame frame{fields[0], integer(fields[1]), integer(fields[2]), {}};
    if (frame.move_ms < 150 || frame.move_ms > 5000 || frame.settle_ms < 100 || frame.settle_ms > 2000)
      throw std::runtime_error("Frame timing outside demonstration envelope");
    for (int id = 1; id <= 18; ++id) frame.positions[id - 1] = integer(fields[id + 2]);
    (void)move_packet(frame.positions, frame.move_ms);
    if (!frames.emplace(frame.name, frame).second) throw std::runtime_error("Duplicate frame name: " + frame.name);
  }
  if (!header || frames.empty()) throw std::runtime_error("Empty frame table");
  return frames;
}
std::vector<Step> stand_plan() { return {{"stand", 0, false}}; }
std::vector<Step> walk_plan() {
  std::vector<Step> plan{{"stand", 0, false}};
  // One cycle = one complete A swing and one complete B swing, including both landings.
  // A = right-front + left-middle + right-rear. B is the complementary tripod.
  for (int cycle = 1; cycle <= 6; ++cycle) {
    plan.push_back({cycle == 1 ? "a_lift_neutral" : "a_lift", cycle, false});
    plan.push_back({"a_swing", cycle, false});
    plan.push_back({"a_land", cycle, false});
    plan.push_back({"b_lift", cycle, false});
    plan.push_back({"b_swing", cycle, false});
    plan.push_back({"b_land", cycle, true});
  }
  // Recovery lifts return each tripod to neutral, without pushing with its support tripod.
  for (const auto *name : {"recover_a_lift", "recover_a_center", "recover_a_land",
                           "recover_b_lift", "recover_b_center", "recover_b_land", "stand"})
    plan.push_back({name, 0, false});
  return plan;
}
int run_demo(bool walking, int argc, char **argv) {
  std::unique_ptr<Controller> controller;
  bool moved = false;
  std::cout << std::unitbuf;
  const auto hold = [&]() {
    if (!controller || !moved) return;
    try {
      controller->hold_current();
      std::cerr << "HOLD: current positions commanded; torque stays enabled. No automatic return to standing.\n";
    } catch (const std::exception &e) {
      std::cerr << "HOLD FAILED: " << e.what() << ". Current board move may finish; use the physical power switch if needed.\n";
    }
  };
  try {
    Options options;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--help" || arg == "-h") {
        std::cout << (walking ? "walk_demo" : "stand_demo")
          << " [--dry-run | --probe | --execute] [--port /dev/ttyS0] [--frames FILE] [--packets]\n"
          << "Default: dry-run, no UART open. --probe: battery and all 18 positions only.\n"
          << "--execute: run fixed frames; walking is exactly 6 complete cycles, then neutral recovery.\n"
          << "Ctrl+C: stop future frames and attempt position hold; not a hardware emergency stop.\n";
        return 0;
      }
      if (arg == "--execute") options.execute = true;
      else if (arg == "--probe") options.probe = true;
      else if (arg == "--dry-run") { /* Default; checked for contradiction below. */ }
      else if (arg == "--packets") options.packets = true;
      else if (arg == "--port" || arg == "--frames") {
        if (++i >= argc) throw std::runtime_error(arg + " requires a value");
        (arg == "--port" ? options.port : options.frames) = argv[i];
      } else throw std::runtime_error("Unknown argument: " + arg);
    }
    bool explicit_dry = false;
    for (int i = 1; i < argc; ++i) if (std::string(argv[i]) == "--dry-run") explicit_dry = true;
    if ((options.execute && options.probe) || (explicit_dry && (options.execute || options.probe)))
      throw std::runtime_error("Choose only one mode: --dry-run, --probe, or --execute");
    if (options.packets && (options.execute || options.probe)) throw std::runtime_error("--packets is for dry-run only");
    if (options.frames.empty()) options.frames = ament_index_cpp::get_package_share_directory("hexapod_demo") + "/config/poses.csv";
    const auto frames = load_frames(options.frames);
    const auto plan = walking ? walk_plan() : stand_plan();
    int planned_ms = 0;
    for (const auto &step : plan) {
      if (!frames.count(step.frame)) throw std::runtime_error("Missing frame: " + step.frame);
      const auto &f = frames.at(step.frame);
      planned_ms += f.move_ms + f.settle_ms + 150; // Include serial transfer and readback budget estimate.
    }
    if (planned_ms > 75000) throw std::runtime_error("Frame plan too long for the 90-second deadline");
    std::cout << "PLAN " << (walking ? "six complete tripod cycles + neutral recovery" : "standing once")
              << "; frames=" << plan.size() << "; estimated_ms=" << planned_ms << '\n';
    if (!options.execute && !options.probe) {
      std::cout << "DRY RUN: no UART access; frame values are candidates, not mechanically validated limits.\n";
      for (const auto &step : plan) {
        const auto &frame = frames.at(step.frame);
        std::cout << "FRAME " << frame.name << " cycle=" << step.cycle << " move_ms=" << frame.move_ms << ' ';
        print_pose(frame.positions);
        if (options.packets) {
          std::cout << "PACKET "; for (auto byte : move_packet(frame.positions, frame.move_ms))
            std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << ' ';
          std::cout << std::dec << std::setfill(' ') << '\n';
        }
      }
      return 0;
    }
    std::signal(SIGINT, stop_signal); std::signal(SIGTERM, stop_signal); std::signal(SIGHUP, stop_signal);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(90);
    controller = std::make_unique<Controller>(options.port);
    voltage_check(controller->battery_mv());
    const auto initial = controller->positions();
    std::cout << "POSITIONS "; print_pose(initial);
    check_stop(deadline);
    if (options.probe) { std::cout << "PROBE_OK: no motion command transmitted.\n"; return 0; }
    std::cout << "EXECUTE: fixed-frame candidate; keep the physical power switch accessible.\n";
    for (const auto &step : plan) {
      check_stop(deadline);
      if (step.cycle && (step.frame == "a_lift" || step.frame == "a_lift_neutral")) {
        std::cout << "CYCLE " << step.cycle << "/6\n";
        voltage_check(controller->battery_mv()); check_stop(deadline);
      }
      const auto &frame = frames.at(step.frame);
      std::cout << "FRAME " << frame.name << " cycle=" << step.cycle << '\n';
      moved = true; controller->move(frame.positions, frame.move_ms);
      // At 9600 baud the 61-byte command takes about 64 ms to leave the UART.
      wait_frame(frame.move_ms + frame.settle_ms + 70, deadline);
      const auto actual = controller->positions(); check_stop(deadline);
      for (int id = 1; id <= 18; ++id)
        if (std::abs(actual[id - 1] - frame.positions[id - 1]) > 30)
          throw std::runtime_error("Readback differs from target at ID" + std::to_string(id) + "; aborting further frames");
      if (step.cycle_end) std::cout << "CYCLE_COMPLETE " << step.cycle << "/6\n";
    }
    std::cout << "COMPLETE: standing pose retained, torque enabled, no future frames queued.\n";
    return 0;
  } catch (const Stopped &) {
    std::cerr << "INTERRUPTED: no more gait frames.\n"; hold(); return 130;
  } catch (const std::exception &error) {
    std::cerr << "ERROR: " << error.what() << '\n'; hold(); return 1;
  }
}
}  // namespace hexapod_demo
