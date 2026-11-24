#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace hexapod_hardware {

constexpr std::size_t servo_count = 18;
constexpr std::uint16_t protocol_min_move_ms = 20;
constexpr std::uint16_t protocol_max_move_ms = 5000;
constexpr std::uint16_t protocol_max_position = 1000;
constexpr std::size_t move_packet_size = 7 + 3 * servo_count;
// 9600 8N1, including the start and stop bits for each byte.
constexpr std::chrono::nanoseconds move_wire_time{(move_packet_size * 10ULL * 1'000'000'000 + 9599) / 9600};

using servo_positions = std::array<std::uint16_t, servo_count>;
using move_packet = std::array<std::uint8_t, move_packet_size>;
using monotonic_time = std::chrono::steady_clock::time_point;

move_packet encode_move(const servo_positions& positions, std::uint16_t duration_ms);

class servo_controller {
public:
    explicit servo_controller(const std::string& device);
    ~servo_controller();

    servo_controller(const servo_controller&) = delete;
    servo_controller& operator=(const servo_controller&) = delete;

    void check_link();
    // Returns the monotonic time observed after the kernel accepted the complete packet.
    // Physical wire time follows.
    monotonic_time send_move(const servo_positions& positions, std::uint16_t duration_ms, monotonic_time deadline);
    servo_positions positions(monotonic_time deadline);
    std::uint16_t voltage_mv(monotonic_time deadline);

private:
    monotonic_time send(const std::uint8_t* bytes, std::size_t size, monotonic_time deadline);
    std::vector<std::uint8_t> query(const std::vector<std::uint8_t>& request, std::uint8_t command, monotonic_time deadline);

    int fd_ = -1;
};

}  // namespace hexapod_hardware
