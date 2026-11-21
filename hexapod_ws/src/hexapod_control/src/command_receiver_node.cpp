#include "command_input.hpp"

#include "hexapod_interfaces/msg/motion_command.hpp"
#include "hexapod_interfaces/msg/planner_status.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace hexapod_control {
namespace {

using hexapod_interfaces::msg::MotionCommand;
using hexapod_interfaces::msg::PlannerStatus;
using sensor_msgs::msg::Joy;

constexpr std::int64_t nanoseconds_per_second = 1'000'000'000;

class command_receiver_node final : public rclcpp::Node {
 public:
    command_receiver_node()
        : Node("hexapod_command"),
          joystick_(configure_joystick()),
          timeout_(std::chrono::milliseconds(integer_parameter("input_timeout_ms", 500, 1, 30000))),
          stop_button_(integer_parameter("stop_button", -1, -1, std::numeric_limits<std::int32_t>::max())),
          teleop_(timeout_) {
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(64)).reliable().durability_volatile();
        output_ = create_publisher<MotionCommand>("motion_command", qos);
        requests_ = create_subscription<MotionCommand>("command_request", qos, [this](MotionCommand::ConstSharedPtr msg) { on_request(*msg); });
        status_ = create_subscription<PlannerStatus>("planner_status", qos, [this](PlannerStatus::ConstSharedPtr msg) { teleop_.planner_status(msg->command_sequence, msg->state); });
        joy_ = create_subscription<Joy>("joy", rclcpp::SensorDataQoS(), [this](Joy::ConstSharedPtr msg) { on_joy(*msg); });
    }

 private:
    template <typename T>
    T startup_parameter(const char* name, const T& default_value) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        return declare_parameter<T>(name, default_value, descriptor);
    }

    std::int64_t integer_parameter(const char* name, std::int64_t default_value, std::int64_t minimum, std::int64_t maximum) {
        const auto value = startup_parameter(name, default_value);
        if (value < minimum || value > maximum) {
            throw std::invalid_argument(std::string(name) + " is out of range");
        }
        return value;
    }

    joystick_config configure_joystick() {
        const auto indices = startup_parameter<std::vector<std::int64_t>>("axis_indices", {1, 0, 2});
        const auto minimums = startup_parameter<std::vector<double>>("axis_minimums", {-1.0, -1.0, -1.0});
        const auto centers = startup_parameter<std::vector<double>>("axis_centers", {0.0, 0.0, 0.0});
        const auto maximums = startup_parameter<std::vector<double>>("axis_maximums", {1.0, 1.0, 1.0});
        const auto directions = startup_parameter<std::vector<double>>("axis_directions", {1.0, 1.0, 1.0});
        joystick_config config{{}, startup_parameter("joystick_deadzone", 0.05)};
        const auto count = config.axes.size();
        if (indices.size() != count || minimums.size() != count || centers.size() != count || maximums.size() != count || directions.size() != count) {
            throw std::invalid_argument("joystick calibration arrays must contain vx, vy and yaw entries");
        }
        for (std::size_t i = 0; i < count; ++i) {
            if (indices[i] < 0 || static_cast<std::uint64_t>(indices[i]) > std::numeric_limits<std::size_t>::max()) {
                throw std::invalid_argument("axis index is out of range");
            }
            config.axes[i] = {static_cast<std::size_t>(indices[i]), minimums[i], centers[i], maximums[i], directions[i]};
        }
        config.validate();
        return config;
    }

    std::uint64_t publish(std::uint8_t kind, nanoseconds stamp, const command_components& values = {}, std::uint32_t cycles = 0) {
        if (stamp < last_output_stamp_ || stamp.count() / nanoseconds_per_second > std::numeric_limits<std::int32_t>::max()) {
            throw std::invalid_argument("robot clock cannot supply an ordered command timestamp");
        }
        MotionCommand message;
        message.kind = kind;
        message.stamp.sec = static_cast<std::int32_t>(stamp.count() / nanoseconds_per_second);
        message.stamp.nanosec = static_cast<std::uint32_t>(stamp.count() % nanoseconds_per_second);
        message.sequence = sequence_.next(std::chrono::duration_cast<nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()));
        message.vx_m_s = values[0];
        message.vy_m_s = values[1];
        message.wz_rad_s = values[2];
        message.cycle_count = cycles;
        output_->publish(message);
        last_output_stamp_ = stamp;
        return message.sequence;
    }

    void publish_teleop(teleop_source source, nanoseconds stamp, nanoseconds now, const command_components& input) {
        if (stamp >= last_output_stamp_ && teleop_.accept(source, stamp, now, input)) {
            publish(MotionCommand::TELEOP, stamp, input);
        }
    }

    void stop(nanoseconds now) {
        teleop_.stop();
        publish(MotionCommand::STOP, now);
    }

    void on_request(const MotionCommand& request) {
        try {
            const nanoseconds now(system_clock_.now().nanoseconds());
            const command_components values{request.vx_m_s, request.vy_m_s, request.wz_rad_s};
            if (request.kind == MotionCommand::TELEOP) {
                const auto stamp = require_fresh_stamp(request.stamp.sec, request.stamp.nanosec, now, timeout_);
                publish_teleop(teleop_source::request, stamp, now, normalize_teleop(values));
                return;
            }
            if (request.stamp.sec != 0 || request.stamp.nanosec != 0) {
                require_fresh_stamp(request.stamp.sec, request.stamp.nanosec, now, timeout_);
            }
            if (request.kind == MotionCommand::WALK_CYCLES) {
                require_walk(values, request.cycle_count);
                teleop_.requested_walk(publish(MotionCommand::WALK_CYCLES, now, values, request.cycle_count));
            } else if (request.kind == MotionCommand::STOP) {
                stop(now);
            } else {
                throw std::invalid_argument("unknown motion command kind");
            }
        } catch (const std::exception& error) {
            RCLCPP_WARN_THROTTLE(get_logger(), system_clock_, 2000, "Command request ignored: %s", error.what());
        }
    }

    void on_joy(const Joy& joy) {
        try {
            const nanoseconds now(system_clock_.now().nanoseconds());
            const auto stamp = require_fresh_stamp(joy.header.stamp.sec, joy.header.stamp.nanosec, now, timeout_);
            if (last_joy_stamp_ && stamp <= *last_joy_stamp_) {
                return;
            }
            if (last_joy_stamp_ && stamp - *last_joy_stamp_ >= timeout_) {
                stop_pressed_ = false;
            }
            last_joy_stamp_ = stamp;
            if (stop_button_ >= 0) {
                const auto button = static_cast<std::size_t>(stop_button_);
                if (button >= joy.buttons.size()) {
                    throw std::invalid_argument("Joy sample is missing stop_button");
                }
                const bool pressed = joy.buttons[button] != 0;
                if (pressed && !stop_pressed_) {
                    stop(now);
                }
                stop_pressed_ = pressed;
                if (pressed) {
                    return;
                }
            }
            publish_teleop(teleop_source::joystick, stamp, now, joystick_.read(joy.axes));
        } catch (const std::exception& error) {
            RCLCPP_WARN_THROTTLE(get_logger(), system_clock_, 2000, "Joy sample ignored: %s", error.what());
        }
    }

    joystick_config joystick_;
    nanoseconds timeout_;
    std::int64_t stop_button_;
    teleop_gate teleop_;
    command_sequence sequence_;
    rclcpp::Clock system_clock_{RCL_SYSTEM_TIME};
    nanoseconds last_output_stamp_{};
    std::optional<nanoseconds> last_joy_stamp_;
    bool stop_pressed_ = false;

    rclcpp::Publisher<MotionCommand>::SharedPtr output_;
    rclcpp::Subscription<MotionCommand>::SharedPtr requests_;
    rclcpp::Subscription<PlannerStatus>::SharedPtr status_;
    rclcpp::Subscription<Joy>::SharedPtr joy_;
};

}  // namespace
}  // namespace hexapod_control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<hexapod_control::command_receiver_node>();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("hexapod_command"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
