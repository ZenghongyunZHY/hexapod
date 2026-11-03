#include "hexapod_demo/demo.hpp"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <cstdio>
using namespace hexapod_demo;
void require(bool value, const std::string &message) { if (!value) throw std::runtime_error(message); }
int main(int argc, char **argv) {
  try {
    require(argc == 2, "Need fixed frame table");
    Pose p{}; p.fill(500); p[0] = 800;
    const auto packet = move_packet(p, 1000);
    const Bytes prefix{0x55,0x55,0x3b,0x03,0x12,0xe8,0x03,0x01,0x20,0x03};
    require(packet.size() == 61 && std::equal(prefix.begin(), prefix.end(), packet.begin()), "Controller command 3 wire format");
    require(packet[58] == 18 && packet[59] == 0xf4 && packet[60] == 1, "Last physical ID and little-endian position");
    p[4] = 1001; bool rejected = false;
    try { (void)move_packet(p, 1000); } catch (const std::invalid_argument &) { rejected = true; }
    require(rejected, "Out-of-range target rejection");
    const auto query = positions_query();
    require(query.size() == 23 && query[2] == 21 && query[3] == 21 && query[4] == 18 && query.back() == 18, "Read all 18 IDs");
    const auto frames = load_frames(argv[1]);
    const auto &stand = frames.at("stand").positions;
    for (int leg = 0; leg < 6; ++leg) {
      require(stand[leg*3] == 500 && stand[leg*3+1] == (leg < 3 ? 550 : 450), "Neutral hip and raised femur per desired stance");
      require(stand[leg*3+2] == (leg < 3 ? 200 : 800), "User-corrected knee direction in bent-leg standing");
    }
    // Only the chosen alternating tripod may lift, and both right pitches increase to lift.
    for (int id : {2,3,8,9}) require(frames.at("a_lift_neutral").positions[id-1] > stand[id-1], "Right A lift direction");
    for (int id : {14,15}) require(frames.at("a_lift_neutral").positions[id-1] < stand[id-1], "Left A lift direction");
    for (int id : {4,5,6,10,11,12,16,17,18})
      require(frames.at("a_lift_neutral").positions[id-1] == stand[id-1], "Support tripod stays unchanged on initial lift");
    const auto plan = walk_plan();
    require(plan.front().frame == "stand" && plan.back().frame == "stand", "Starts and ends at identical standing pose");
    for (int cycle = 1; cycle <= 6; ++cycle) {
      const auto count = std::count_if(plan.begin(), plan.end(), [cycle](const Step &s){return s.cycle == cycle;});
      require(count == 6, "Each complete cycle has both swing and landing phases");
      require(std::count_if(plan.begin(), plan.end(), [cycle](const Step &s){return s.cycle == cycle && s.cycle_end;}) == 1, "Exactly one completion per cycle");
    }
    require(std::none_of(plan.begin(), plan.end(), [](const Step &s){return s.cycle > 6;}), "No seventh cycle");
    require(frames.at("recover_b_land").positions == stand, "Recovery returns all 18 IDs to standing");
    int ms = 0;
    for (const auto &s : plan) { const auto &f = frames.at(s.frame); ms += f.move_ms + f.settle_ms + 150; }
    require(ms < 75000, "Finite execution budget");
    require(stand_plan().size() == 1, "Standing demo sends one target frame only");
    const std::string bad = std::string(argv[1]) + ".invalid-test";
    { std::ofstream out(bad); out << "name,move_ms,settle_ms,id2\n"; }
    rejected = false; try { (void)load_frames(bad); } catch (const std::runtime_error &) { rejected = true; }
    std::remove(bad.c_str()); require(rejected, "Incorrect frame column mapping rejected");
    std::cout << "PASS: controller packets, 18-ID directions, alternating supports, six complete cycles, neutral recovery, input validation.\n";
    return 0;
  } catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
