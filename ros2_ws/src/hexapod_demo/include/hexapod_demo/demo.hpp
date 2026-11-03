#pragma once
#include <array>
#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace hexapod_demo {
using Pose = std::array<int, 18>;
using Bytes = std::vector<std::uint8_t>;
struct Frame { std::string name; int move_ms; int settle_ms; Pose positions; };
using Frames = std::map<std::string, Frame>;
struct Step { std::string frame; int cycle; bool cycle_end = false; };
Bytes move_packet(const Pose &positions, int duration_ms);
Bytes positions_query();
Frames load_frames(const std::string &path);
std::vector<Step> stand_plan();
std::vector<Step> walk_plan();
int run_demo(bool walking, int argc, char **argv);

class Controller {
 public:
  explicit Controller(const std::string &path);
  ~Controller();
  Controller(const Controller &) = delete;
  Controller &operator=(const Controller &) = delete;
  Pose positions(int timeout_ms = 900);
  int battery_mv(int timeout_ms = 900);
  void move(const Pose &pose, int duration_ms);
  void hold_current();
 private:
  int fd_ = -1;
  Bytes rx_;
  void send(const Bytes &bytes);
  Bytes query(const Bytes &request, int expected_command, int timeout_ms);
};
}  // namespace hexapod_demo
