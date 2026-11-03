#include "hexapod_demo/demo.hpp"
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>

namespace hexapod_demo {
namespace {
[[noreturn]] void io_error(const std::string &op) {
  throw std::runtime_error(op + ": " + std::strerror(errno));
}
void le16(Bytes &out, int value) {
  out.push_back(static_cast<std::uint8_t>(value & 255));
  out.push_back(static_cast<std::uint8_t>((value >> 8) & 255));
}
int remaining_ms(std::chrono::steady_clock::time_point end) {
  return static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
      end - std::chrono::steady_clock::now()).count());
}
Bytes position_request(const std::vector<int> &ids) {
  Bytes out{0x55, 0x55, static_cast<std::uint8_t>(ids.size() + 3), 21,
            static_cast<std::uint8_t>(ids.size())};
  for (const int id : ids) out.push_back(static_cast<std::uint8_t>(id));
  return out;
}
}  // namespace

Bytes move_packet(const Pose &positions, int duration_ms) {
  if (duration_ms < 20 || duration_ms > 5000) throw std::invalid_argument("Move duration must be 20..5000 ms");
  Bytes out{0x55, 0x55, 59, 3, 18};  // Length = 18*3+5; total packet size = 61.
  le16(out, duration_ms);
  for (int i = 0; i < 18; ++i) {
    if (positions[i] < 0 || positions[i] > 1000) throw std::invalid_argument("Position outside 0..1000");
    out.push_back(static_cast<std::uint8_t>(i + 1));
    le16(out, positions[i]);
  }
  return out;
}
Bytes positions_query() {
  Bytes out{0x55, 0x55, 21, 21, 18};
  for (int id = 1; id <= 18; ++id) out.push_back(static_cast<std::uint8_t>(id));
  return out;
}
Controller::Controller(const std::string &path) {
  fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) io_error("Open " + path);
  try {
    if (::flock(fd_, LOCK_EX | LOCK_NB) < 0) io_error("UART already locked by another demo");
    termios t{};
    if (::tcgetattr(fd_, &t) < 0) io_error("Read UART settings");
    ::cfmakeraw(&t);
    t.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    t.c_cflag |= CS8 | CLOCAL | CREAD;
    t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 0;
    ::cfsetispeed(&t, B9600); ::cfsetospeed(&t, B9600);
    if (::tcsetattr(fd_, TCSANOW, &t) < 0) io_error("Configure UART 9600 8N1");
  } catch (...) { ::close(fd_); fd_ = -1; throw; }
}
Controller::~Controller() { if (fd_ >= 0) ::close(fd_); }
void Controller::send(const Bytes &bytes) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
  std::size_t offset = 0;
  while (offset < bytes.size()) {
    const int timeout = remaining_ms(end);
    if (timeout <= 0) throw std::runtime_error("UART write timeout; partial packet may have been transmitted");
    pollfd p{fd_, POLLOUT, 0};
    const int ready = ::poll(&p, 1, timeout);
    if (ready < 0) { if (errno == EINTR) continue; io_error("UART write poll"); }
    if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) throw std::runtime_error("UART disconnected");
    if (!ready) continue;
    const auto n = ::write(fd_, bytes.data() + offset, bytes.size() - offset);
    if (n < 0) { if (errno == EINTR || errno == EAGAIN) continue; io_error("UART write"); }
    offset += static_cast<std::size_t>(n);
  }
}
Bytes Controller::query(const Bytes &request, int command, int timeout_ms) {
  if (::tcflush(fd_, TCIFLUSH) < 0) io_error("Clear stale UART response");
  rx_.clear();
  send(request);
  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (remaining_ms(end) > 0) {
    while (rx_.size() >= 4) {
      if (rx_[0] != 0x55 || rx_[1] != 0x55 || rx_[2] < 2 || rx_[2] > 64) {
        rx_.erase(rx_.begin()); continue;
      }
      const auto total = static_cast<std::size_t>(rx_[2] + 2);
      if (rx_.size() < total) break;
      const int received_command = rx_[3];
      Bytes payload(rx_.begin() + 4, rx_.begin() + static_cast<std::ptrdiff_t>(total));
      rx_.erase(rx_.begin(), rx_.begin() + static_cast<std::ptrdiff_t>(total));
      if (received_command == command) return payload;
    }
    pollfd p{fd_, POLLIN, 0};
    const int timeout = remaining_ms(end);
    if (timeout <= 0) break;
    const int ready = ::poll(&p, 1, std::min(timeout, 25));
    if (ready < 0) { if (errno == EINTR) continue; io_error("UART read poll"); }
    if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) throw std::runtime_error("UART disconnected");
    if (!ready) continue;
    std::uint8_t buffer[256]; const auto n = ::read(fd_, buffer, sizeof(buffer));
    if (n < 0) { if (errno == EINTR || errno == EAGAIN) continue; io_error("UART read"); }
    rx_.insert(rx_.end(), buffer, buffer + n);
    if (rx_.size() > 4096) throw std::runtime_error("UART response overflow");
  }
  throw std::runtime_error("Controller response timed out for command " + std::to_string(command));
}
Pose Controller::positions(int timeout_ms) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  Pose pose{}; std::array<bool, 18> seen{};
  for (int attempt = 1; attempt <= 3; ++attempt) {
    std::vector<int> missing;
    for (int id = 1; id <= 18; ++id) if (!seen[id - 1]) missing.push_back(id);
    const int remaining = remaining_ms(end);
    if (remaining <= 0) break;
    const auto response = query(position_request(missing), 21, remaining);
    if (response.empty() || response[0] > missing.size() ||
        response.size() != 1 + static_cast<std::size_t>(response[0]) * 3)
      throw std::runtime_error("Malformed position response: count and payload length disagree");
    for (std::size_t i = 1; i < response.size(); i += 3) {
      const int id = response[i], position = response[i + 1] | (response[i + 2] << 8);
      if (id < 1 || id > 18 || seen[id - 1] || position > 1000 ||
          std::find(missing.begin(), missing.end(), id) == missing.end())
        throw std::runtime_error("Invalid, unexpected, or duplicate servo ID in position response");
      seen[id - 1] = true; pose[id - 1] = position;
    }
    if (std::all_of(seen.begin(), seen.end(), [](bool value) { return value; })) return pose;
    if (attempt < 3 && remaining_ms(end) > 20) {
      std::cerr << "POSITION_RETRY: missing IDs";
      for (int id = 1; id <= 18; ++id) if (!seen[id - 1]) std::cerr << ' ' << id;
      std::cerr << "; targeted retry " << attempt << "/2\n";
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  std::string missing;
  for (int id = 1; id <= 18; ++id) if (!seen[id - 1]) missing += ' ' + std::to_string(id);
  throw std::runtime_error("Position read incomplete after bounded retries; missing IDs:" + missing);
}
int Controller::battery_mv(int timeout_ms) {
  const auto response = query(Bytes{0x55, 0x55, 2, 15}, 15, timeout_ms);
  if (response.size() != 2) throw std::runtime_error("Invalid battery response");
  return response[0] | (response[1] << 8);
}
void Controller::move(const Pose &pose, int duration_ms) { send(move_packet(pose, duration_ms)); }
void Controller::hold_current() {
  // Command 7 stops action groups only; it cannot stop a command-3 move.
  // Read the real positions, then replace the active target with those positions.
  move(positions(350), 20);
}
}  // namespace hexapod_demo
