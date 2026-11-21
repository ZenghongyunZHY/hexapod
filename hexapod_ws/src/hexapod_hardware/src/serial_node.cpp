#include "serial_runtime.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "hexapod_core/joint_mapper.hpp"
#include "hexapod_interfaces/msg/execution_status.hpp"
#include "hexapod_interfaces/msg/plan_stop.hpp"
#include "hexapod_interfaces/msg/servo_plan.hpp"
#include "hexapod_interfaces/msg/standing_reference.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace hexapod_hardware {
namespace {

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using hexapod_interfaces::msg::ExecutionStatus;
using hexapod_interfaces::msg::PlanStop;
using hexapod_interfaces::msg::ServoPlan;
using hexapod_interfaces::msg::StandingReference;
using sensor_msgs::msg::JointState;
using std_srvs::srv::Trigger;
using namespace std::chrono_literals;

constexpr auto time_units_per_second = 1'000'000'000;

builtin_interfaces::msg::Time wire_stamp(std::chrono::nanoseconds since_epoch) {
  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
  if (since_epoch < 0ns || seconds.count() > std::numeric_limits<std::int32_t>::max()) {
    throw std::range_error("Robot system time cannot be represented in ROS Time");
  }
  builtin_interfaces::msg::Time result;
  result.sec = static_cast<std::int32_t>(seconds.count());
  result.nanosec = static_cast<std::uint32_t>((since_epoch - seconds).count());
  return result;
}

std::uint8_t wire_state(execution_state state) {
  switch (state) {
    case execution_state::accepted: return ExecutionStatus::ACCEPTED;
    case execution_state::completed: return ExecutionStatus::COMPLETED;
    case execution_state::rejected: return ExecutionStatus::REJECTED;
    case execution_state::failed: return ExecutionStatus::FAILED;
    case execution_state::interrupted: return ExecutionStatus::INTERRUPTED;
  }
  throw std::logic_error("Unknown serial execution state");
}

serial_mode load_mode(const std::string& mode) {
  if (mode == "preview") return serial_mode::preview;
  if (mode == "probe") return serial_mode::probe;
  if (mode == "execute") return serial_mode::execute;
  throw std::invalid_argument("mode must be preview, probe or execute");
}

class serial_node final : public rclcpp::Node {
 public:
  serial_node()
      : Node("hexapod_serial"), hardware_config_(parameter<std::string>("hardware_config", "")),
        mode_name_(parameter<std::string>("mode", "preview")), device_(parameter<std::string>("serial_port", "")),
        model_(hexapod_core::robot_model::from_file(hardware_config_)), mapper_(model_) {
    const auto mode = load_mode(mode_name_);
    if (mode != serial_mode::preview && device_.empty()) {
      throw std::invalid_argument("serial_port must be supplied for probe or execute mode");
    }
    const auto preview = load_preview_positions(parameter("preview_positions", std::vector<std::int64_t>{}));
    const auto wire_time = mode == serial_mode::preview ? 0ns : move_wire_time;
    const execution_options options{milliseconds_parameter("send_advance_ms", 90, 1, 5000), wire_time,
        milliseconds_parameter("completion_margin_ms", 5, 0, 1000)};
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(256)).reliable().durability_volatile();
    status_pub_ = create_publisher<ExecutionStatus>("execution_status", qos);
    reference_pub_ = create_publisher<StandingReference>("standing_reference", qos);
    joint_pub_ = create_publisher<JointState>("joint_states", 10);
    diagnostic_pub_ = create_publisher<DiagnosticArray>("diagnostics", 10);
    plan_sub_ = create_subscription<ServoPlan>("servo_plan", qos, [this](ServoPlan::ConstSharedPtr message) { receive_plan(*message); });
    stop_sub_ = create_subscription<PlanStop>("plan_stop", qos, [this](PlanStop::ConstSharedPtr message) { runtime_->request_stop(message->plan_id); });
    confirm_service_ = create_service<Trigger>("confirm_standing", [this](std::shared_ptr<rmw_request_id_t> request_id,
          std::shared_ptr<Trigger::Request>) { request_confirmation(std::move(request_id)); });
    runtime_ = std::make_unique<serial_runtime>(serial_settings{mode, device_, hardware_config_, preview, options},
        [this](const servo_positions& positions) { validate_positions(positions); });
    event_timer_ = create_wall_timer(5ms, [this] { publish_events(); });
    RCLCPP_INFO(get_logger(), "Serial node started in %s mode", mode_name_.c_str());
  }

 private:
  template <typename T>
  T parameter(const char* name, const T& value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }

  std::chrono::milliseconds milliseconds_parameter(const char* name, int value, int minimum, int maximum) {
    const auto ms = parameter<std::int64_t>(name, value);
    if (ms < minimum || ms > maximum) {
      throw std::invalid_argument(std::string(name) + " outside allowed range");
    }
    return std::chrono::milliseconds(ms);
  }

  std::optional<servo_positions> load_preview_positions(const std::vector<std::int64_t>& values) const {
    if (values.empty()) return std::nullopt;
    servo_positions positions{};
    if (values.size() != positions.size()) {
      throw std::invalid_argument("preview_positions requires 18 positions ordered by physical ID");
    }
    for (std::size_t i = 0; i != values.size(); ++i) {
      if (values[i] < 0 || values[i] > 1000) {
        throw std::invalid_argument("preview_positions contains a non-native position");
      }
      positions[i] = static_cast<std::uint16_t>(values[i]);
    }
    validate_positions(positions);
    return positions;
  }

  void validate_positions(const servo_positions& positions) const {
    for (const auto& joint : model_.joints()) {
      const auto id = joint.binding.id;
      const hexapod_core::servo_position position(positions[id.index()]);
      const auto angle = mapper_.servo_position_to_joint_angle_rad(id, position);
      if (mapper_.joint_angle_rad_to_servo_position(id, angle).value() != position.value()) {
        throw std::invalid_argument("Servo position cannot round-trip within installed limits");
      }
    }
  }

  void receive_plan(const ServoPlan& message) {
    timed_plan plan{message.plan_id, 0ns, message.initial_positions, {}};
    if (message.motion_start.sec >= 0 && message.motion_start.nanosec < time_units_per_second) {
      plan.motion_start = std::chrono::seconds(message.motion_start.sec) + std::chrono::nanoseconds(message.motion_start.nanosec);
      plan.segments.reserve(message.segments.size());
      for (const auto& segment : message.segments) {
        plan.segments.push_back({segment.start_offset_ms, segment.move_duration_ms, segment.positions, segment.walking_cycle_end});
      }
    }
    runtime_->submit(std::move(plan));
  }

  void request_confirmation(std::shared_ptr<rmw_request_id_t> request_id) {
    if (pending_confirmation_ || reference_pub_->get_subscription_count() == 0) {
      Trigger::Response response;
      response.success = false;
      response.message = pending_confirmation_ ? "A standing confirmation is already pending" : "No subscriber for standing_reference";
      confirm_service_->send_response(*request_id, response);
      return;
    }
    pending_confirmation_ = std::move(request_id);
    runtime_->request_confirmation();
  }

  void publish(const execution_report& report) {
    ExecutionStatus message;
    message.plan_id = report.id;
    message.state = wire_state(report.state);
    message.completed_segments = report.completed_segments;
    message.reason = report.reason;
    status_pub_->publish(message);
  }

  void publish(const standing_confirmation& result) {
    if (result.reference) {
      StandingReference message;
      message.stamp = wire_stamp(result.reference->received_at);
      message.positions = result.reference->positions;
      reference_pub_->publish(message);
    }
    if (pending_confirmation_) {
      Trigger::Response response;
      response.success = result.reference.has_value();
      response.message = result.reason;
      confirm_service_->send_response(*pending_confirmation_, response);
      pending_confirmation_.reset();
    }
  }

  static void add_value(DiagnosticStatus& status, std::string key, std::string value) {
    auto& entry = status.values.emplace_back();
    entry.key = std::move(key);
    entry.value = std::move(value);
  }

  void publish_positions(const position_readback& readback) {
    JointState message;
    // 舵机协议没有逐轴时间戳，用本次完整回包时间近似并另报查询耗时。
    message.header.stamp = wire_stamp(readback.received_at);
    const auto native = model_.servo().native_position_limits;
    for (const auto& joint : model_.joints()) {
      const auto id = joint.binding.id;
      const auto position = readback.positions[id.index()];
      if (model_.is_mapping_ready(id) && position >= native.minimum && position <= native.maximum) {
        message.name.push_back("servo_" + std::to_string(id.value()));
        message.position.push_back(mapper_.servo_position_to_joint_angle_rad(id, hexapod_core::servo_position(position)));
      }
    }
    if (!message.name.empty()) {
      joint_pub_->publish(message);
    }
  }

  void publish(const link_diagnostic& event) {
    DiagnosticArray message;
    message.header.stamp = wire_stamp(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()));
    auto& status = message.status.emplace_back();
    status.name = std::string(get_fully_qualified_name()) + "/serial";
    status.hardware_id = device_;
    status.level = static_cast<std::uint8_t>(event.level);
    status.message = event.reason;
    add_value(status, "mode", mode_name_);
    if (event.millivolts) add_value(status, "voltage_mv", std::to_string(*event.millivolts));
    if (event.readback) {
      add_value(status, "positions_completed_stamp_ns", std::to_string(event.readback->received_at.count()));
      add_value(status, "positions_query_duration_us", std::to_string(event.readback->query_duration.count()));
      publish_positions(*event.readback);
    }
    diagnostic_pub_->publish(message);
  }

  void publish_events() {
    for (const auto& event : runtime_->drain_events()) {
      std::visit([this](const auto& value) { publish(value); }, event);
    }
  }

  const std::string hardware_config_;
  const std::string mode_name_;
  const std::string device_;
  const hexapod_core::robot_model model_;
  const hexapod_core::joint_mapper mapper_;
  rclcpp::Publisher<ExecutionStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<StandingReference>::SharedPtr reference_pub_;
  rclcpp::Publisher<JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::Subscription<ServoPlan>::SharedPtr plan_sub_;
  rclcpp::Subscription<PlanStop>::SharedPtr stop_sub_;
  rclcpp::Service<Trigger>::SharedPtr confirm_service_;
  std::shared_ptr<rmw_request_id_t> pending_confirmation_;
  std::unique_ptr<serial_runtime> runtime_;
  rclcpp::TimerBase::SharedPtr event_timer_;
};

}  // namespace
}  // namespace hexapod_hardware

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<hexapod_hardware::serial_node>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
  } catch (const std::exception& error) {
    std::cerr << "hexapod_serial: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
