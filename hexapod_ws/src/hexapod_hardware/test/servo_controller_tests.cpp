#include "servo_controller.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
using namespace std::chrono_literals;
using namespace hexapod_hardware;
using monotonic_clock = std::chrono::steady_clock;

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

servo_positions sample_positions() {
    servo_positions positions{};
    for (std::size_t index = 0; index < positions.size(); ++index) {
        positions[index] = static_cast<std::uint16_t>(417 + 17 * index);
    }
    return positions;
}

void test_encoding() {
    const auto positions = sample_positions();
    const auto packet = encode_move(positions, 291);
    require(packet.size() == 61 && move_wire_time.count() == 63'541'667, "wrong packet length or wire time");
    require(packet[0] == 0x55 && packet[1] == 0x55 && packet[2] == 59 && packet[3] == 3 && packet[4] == 18,
            "incorrect synchronous movement header");
    require(packet[5] == 0x23 && packet[6] == 0x01, "duration isn't little endian");
    for (std::size_t index = 0; index < positions.size(); ++index) {
        const auto offset = 7 + 3 * index;
        require(packet[offset] == index + 1, "incorrect physical ID order");
        require((packet[offset + 1] | (packet[offset + 2] << 8)) == positions[index], "incorrect movement position");
    }
    for (const auto invalid_duration : {std::uint16_t{19}, std::uint16_t{5001}}) {
        bool rejected = false;
        try {
            encode_move(positions, invalid_duration);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        require(rejected, "invalid duration accepted");
    }
    auto invalid = positions;
    invalid.back() = 1001;
    bool rejected = false;
    try {
        encode_move(invalid, 200);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, "invalid protocol position accepted");
}

void test_virtual_port(const std::string& mode, const std::string& port) {
    servo_controller controller(port);
    controller.check_link();
    if (mode == "normal") {
        bool in_use = false;
        try {
            servo_controller other(port);
        } catch (const std::runtime_error&) {
            in_use = true;
        }
        require(in_use, "UART lock did not prevent a second owner");
        require(controller.voltage_mv(monotonic_clock::now() + 500ms) == 7400, "wrong voltage");
        require(controller.positions(monotonic_clock::now() + 500ms) == sample_positions(), "wrong positions");
        bool expired = false;
        try {
            controller.send_move(sample_positions(), 120, monotonic_clock::now() - 1ms);
        } catch (const std::runtime_error&) {
            expired = true;
        }
        require(expired, "expired movement was not rejected");
        controller.send_move(sample_positions(), 120, monotonic_clock::now() + 500ms);
        return;
    }

    const auto started = monotonic_clock::now();
    std::string failure;
    try {
        controller.positions(started + 360ms);
    } catch (const std::runtime_error& error) {
        failure = error.what();
    }
    if (mode == "timeout") {
        require(failure.find("deadline") != std::string::npos, "missing deadline failure: " + failure);
        require(monotonic_clock::now() - started < 460ms, "position retry extended the query deadline");
    } else if (mode == "duplicate") {
        require(failure.find("duplicate") != std::string::npos, "missing duplicate failure: " + failure);
    } else if (mode == "disconnect") {
        require(failure.find("disconnected") != std::string::npos, "missing transport failure: " + failure);
    } else {
        throw std::runtime_error("unknown virtual UART test mode");
    }
}
}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 1) {
            test_encoding();
        } else if (argc == 3) {
            test_virtual_port(argv[1], argv[2]);
        } else {
            throw std::runtime_error("expected no arguments or PTY test mode and device");
        }
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
