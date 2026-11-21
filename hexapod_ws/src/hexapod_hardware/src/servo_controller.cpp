#include "servo_controller.hpp"

#include <algorithm>
#include <stdexcept>

#if defined(__unix__) || defined(__APPLE__)
#include <cerrno>
#include <fcntl.h>
#include <limits>
#include <poll.h>
#include <sys/file.h>
#include <system_error>
#include <termios.h>
#include <thread>
#include <unistd.h>
#endif

namespace hexapod_hardware {
namespace {
constexpr std::uint8_t header_byte = 0x55;
constexpr std::uint8_t move_command = 3;

void write_le16(std::uint8_t* bytes, std::uint16_t value) {
    bytes[0] = static_cast<std::uint8_t>(value & 0xff);
    bytes[1] = static_cast<std::uint8_t>(value >> 8);
}

#if defined(__unix__) || defined(__APPLE__)
constexpr std::uint8_t voltage_command = 15;
constexpr std::uint8_t position_command = 21;
constexpr auto retry_spacing = std::chrono::milliseconds(20);
using monotonic_clock = std::chrono::steady_clock;
using byte_vector = std::vector<std::uint8_t>;

[[noreturn]] void serial_error(const std::string& operation) {
    const int code = errno;
    throw std::system_error(code, std::generic_category(), operation);
}

bool retry_io() {
    return errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK;
}

std::uint16_t read_le16(const std::uint8_t* bytes) {
    return static_cast<std::uint16_t>(bytes[0] | (bytes[1] << 8));
}

bool poll_serial(int fd, short events, int timeout_ms) {
    pollfd descriptor{fd, events, 0};
    if (::poll(&descriptor, 1, timeout_ms) < 0) {
        if (errno == EINTR) return false;
        serial_error("poll UART");
    }
    if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        throw std::runtime_error("UART disconnected");
    }
    return (descriptor.revents & events) != 0;
}

void wait_serial(int fd, short events, monotonic_time deadline, const char* error) {
    auto now = monotonic_clock::now();
    while (now < deadline) {
        const auto remaining = std::chrono::ceil<std::chrono::milliseconds>(deadline - now).count();
        const auto timeout = static_cast<int>(std::min(remaining, static_cast<std::chrono::milliseconds::rep>(std::numeric_limits<int>::max())));
        if (poll_serial(fd, events, timeout) && monotonic_clock::now() < deadline) return;
        now = monotonic_clock::now();
    }
    throw std::runtime_error(error);
}

byte_vector position_request(const std::array<bool, servo_count>& seen) {
    const auto missing = static_cast<std::uint8_t>(std::count(seen.begin(), seen.end(), false));
    byte_vector request{header_byte, header_byte, static_cast<std::uint8_t>(missing + 3), position_command, missing};
    request.reserve(5 + missing);
    for (std::size_t index = 0; index < seen.size(); ++index) {
        if (!seen[index]) request.push_back(static_cast<std::uint8_t>(index + 1));
    }
    return request;
}

bool take_reply(byte_vector& buffer, std::uint8_t command, byte_vector& payload) {
    while (buffer.size() >= 4) {
        if (buffer[0] != header_byte || buffer[1] != header_byte || buffer[2] < 2 || buffer[2] > 64) {
            buffer.erase(buffer.begin());
            continue;
        }
        const auto packet_size = static_cast<std::size_t>(buffer[2]) + 2;
        if (buffer.size() < packet_size) return false;
        const auto packet_end = buffer.begin() + static_cast<byte_vector::difference_type>(packet_size);
        if (buffer[3] == command) {
            payload.assign(buffer.cbegin() + 4, byte_vector::const_iterator(packet_end));
            return true;
        }
        buffer.erase(buffer.begin(), packet_end);
    }
    return false;
}
#else
[[noreturn]] void unavailable() {
    throw std::runtime_error("the real UART backend is available on POSIX only");
}
#endif
}  // namespace

move_packet encode_move(const servo_positions& positions, std::uint16_t duration_ms) {
    if (duration_ms < protocol_min_move_ms || duration_ms > protocol_max_move_ms) {
        throw std::invalid_argument("move duration must be 20..5000 ms");
    }
    move_packet packet{header_byte, header_byte, static_cast<std::uint8_t>(move_packet_size - 2), move_command, static_cast<std::uint8_t>(servo_count)};
    write_le16(packet.data() + 5, duration_ms);
    for (std::size_t index = 0; index < positions.size(); ++index) {
        if (positions[index] > protocol_max_position) throw std::invalid_argument("servo position must be 0..1000");
        const auto offset = 7 + index * 3;
        packet[offset] = static_cast<std::uint8_t>(index + 1);
        write_le16(packet.data() + offset + 1, positions[index]);
    }
    return packet;
}

#if defined(__unix__) || defined(__APPLE__)
servo_controller::servo_controller(const std::string& device) {
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) serial_error("open UART " + device);
    try {
        if (::flock(fd_, LOCK_EX | LOCK_NB) < 0) serial_error("lock UART " + device);
        termios settings{};
        if (::tcgetattr(fd_, &settings) < 0) serial_error("read UART settings");
        ::cfmakeraw(&settings);
        settings.c_iflag &= ~(IXON | IXOFF | IXANY);
        settings.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
#ifdef CRTSCTS
        settings.c_cflag &= ~CRTSCTS;
#endif
        settings.c_cflag |= CS8 | CLOCAL | CREAD;
        settings.c_cc[VMIN] = 0;
        settings.c_cc[VTIME] = 0;
        if (::cfsetispeed(&settings, B9600) < 0 || ::cfsetospeed(&settings, B9600) < 0 ||
            ::tcsetattr(fd_, TCSANOW, &settings) < 0) {
            serial_error("configure UART 9600 8N1");
        }
        if (::tcflush(fd_, TCIFLUSH) < 0) serial_error("clear UART input");
    } catch (...) {
        ::close(fd_);
        throw;
    }
}

servo_controller::~servo_controller() {
    ::close(fd_);
}

void servo_controller::check_link() {
    poll_serial(fd_, 0, 0);
}

void servo_controller::send(const std::uint8_t* bytes, std::size_t size, monotonic_time deadline) {
    constexpr auto timeout = "UART write deadline expired; a partial packet may have been queued";
    std::size_t offset = 0;
    while (offset < size) {
        wait_serial(fd_, POLLOUT, deadline, timeout);
        const auto count = ::write(fd_, bytes + offset, size - offset);
        if (count < 0) {
            if (retry_io()) continue;
            serial_error("write UART");
        }
        if (count == 0) throw std::runtime_error("UART write made no progress");
        offset += static_cast<std::size_t>(count);
    }
    if (monotonic_clock::now() >= deadline) throw std::runtime_error(timeout);
}

void servo_controller::send_move(const servo_positions& positions, std::uint16_t duration_ms, monotonic_time deadline) {
    const auto packet = encode_move(positions, duration_ms);
    send(packet.data(), packet.size(), deadline);
}

std::vector<std::uint8_t> servo_controller::query(const byte_vector& request, std::uint8_t command, monotonic_time deadline) {
    if (monotonic_clock::now() >= deadline) throw std::runtime_error("UART query deadline expired");
    if (::tcflush(fd_, TCIFLUSH) < 0) serial_error("clear stale UART replies");
    send(request.data(), request.size(), deadline);
    byte_vector buffer;
    byte_vector payload;
    std::array<std::uint8_t, 256> chunk{};
    while (true) {
        wait_serial(fd_, POLLIN, deadline, "UART response deadline expired");
        const auto count = ::read(fd_, chunk.data(), chunk.size());
        if (count < 0) {
            if (retry_io()) continue;
            serial_error("read UART");
        }
        buffer.insert(buffer.end(), chunk.begin(), chunk.begin() + count);
        if (take_reply(buffer, command, payload)) {
            if (monotonic_clock::now() >= deadline) throw std::runtime_error("UART response deadline expired");
            return payload;
        }
    }
}

servo_positions servo_controller::positions(monotonic_time deadline) {
    std::array<bool, servo_count> seen{};
    servo_positions positions{};
    for (int attempt = 0; attempt < 3; ++attempt) {
        const auto request = position_request(seen);
        const auto reply = query(request, position_command, deadline);
        if (reply.empty() || reply[0] > request[4] || reply.size() != 1 + 3 * static_cast<std::size_t>(reply[0])) {
            throw std::runtime_error("position response count and payload length disagree");
        }
        for (std::size_t index = 1; index < reply.size(); index += 3) {
            const auto id = reply[index];
            const auto position = read_le16(reply.data() + index + 1);
            if (id == 0 || id > servo_count || seen[id - 1] || position > protocol_max_position) {
                throw std::runtime_error("invalid or duplicate servo in position response");
            }
            positions[id - 1] = position;
            seen[id - 1] = true;
        }
        if (std::all_of(seen.begin(), seen.end(), [](bool value) { return value; })) return positions;
        if (attempt == 2 || deadline - monotonic_clock::now() <= retry_spacing) break;
        std::this_thread::sleep_for(retry_spacing);
    }
    std::string missing;
    for (std::size_t index = 0; index < seen.size(); ++index) {
        if (!seen[index]) missing += ' ' + std::to_string(index + 1);
    }
    throw std::runtime_error("position query incomplete, missing IDs:" + missing);
}

std::uint16_t servo_controller::voltage_mv(monotonic_time deadline) {
    const auto reply = query({header_byte, header_byte, 2, voltage_command}, voltage_command, deadline);
    if (reply.size() != 2) throw std::runtime_error("voltage response payload must contain two bytes");
    return read_le16(reply.data());
}
#else
servo_controller::servo_controller(const std::string&) { unavailable(); }
servo_controller::~servo_controller() = default;
void servo_controller::check_link() { unavailable(); }
void servo_controller::send_move(const servo_positions&, std::uint16_t, monotonic_time) { unavailable(); }
servo_positions servo_controller::positions(monotonic_time) { unavailable(); }
std::uint16_t servo_controller::voltage_mv(monotonic_time) { unavailable(); }
#endif

}  // namespace hexapod_hardware
