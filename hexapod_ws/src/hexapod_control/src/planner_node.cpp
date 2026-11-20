#include "servo_plan_builder.hpp"

#include "hexapod_core/joint_mapper.hpp"
#include "hexapod_core/walking_plan.hpp"
#include "hexapod_interfaces/msg/execution_status.hpp"
#include "hexapod_interfaces/msg/motion_command.hpp"
#include "hexapod_interfaces/msg/plan_stop.hpp"
#include "hexapod_interfaces/msg/planner_status.hpp"
#include "hexapod_interfaces/msg/standing_reference.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace hexapod_control {
namespace {

using namespace hexapod_core;
using namespace std::chrono_literals;
using namespace hexapod_interfaces::msg;
using steady_clock = std::chrono::steady_clock;
using milliseconds = std::chrono::milliseconds;
using positions = std::array<std::uint16_t, joint_count>;

const planar_body_velocity zero_velocity{0.0, 0.0, 0.0};

enum class motion_mode { idle, teleop, finite, stopping };
enum class plan_kind { finite, walking, braking, recovery };

struct boundary_state {
    tripod_sequence_initial_state gait;
    positions nominal_positions;
    planar_body_velocity velocity;
};

struct cycle_checkpoint {
    std::size_t completed_segments;
    milliseconds end_offset;
    boundary_state state;
};

struct pending_plan {
    std::uint64_t id;
    std::uint64_t sequence;
    plan_kind kind;
    std::size_t segment_count;
    boundary_state end_state;
    std::vector<cycle_checkpoint> checkpoints;
    rclcpp::Time wall_end;
    monotonic_time start;
    monotonic_time end;
    bool accepted = false;
};

struct queued_command {
    MotionCommand message;
    monotonic_time sampled;
    monotonic_time expires;
};

double linear_speed(const planar_body_velocity& velocity) {
    return std::hypot(velocity.vx_m_s, velocity.vy_m_s);
}

bool stopped(const planar_body_velocity& velocity) {
    return linear_speed(velocity) == 0.0 && velocity.wz_rad_s == 0.0;
}

planar_body_velocity scaled_velocity(const planar_body_velocity& velocity, double factor) {
    return {velocity.vx_m_s * factor, velocity.vy_m_s * factor, velocity.wz_rad_s * factor};
}

planar_body_velocity approach_velocity(const planar_body_velocity& from, const planar_body_velocity& to, double linear_step, double yaw_step) {
    const double dx = to.vx_m_s - from.vx_m_s;
    const double dy = to.vy_m_s - from.vy_m_s;
    const double distance = std::hypot(dx, dy);
    const double yaw_delta = to.wz_rad_s - from.wz_rad_s;
    const double fraction = std::min({
        1.0, distance == 0.0 ? 1.0 : linear_step / distance,
        yaw_delta == 0.0 ? 1.0 : yaw_step / std::abs(yaw_delta)
    });
    if (fraction == 1.0) {
        return to;
    }
    return {from.vx_m_s + fraction * dx, from.vy_m_s + fraction * dy, from.wz_rad_s + fraction * yaw_delta};
}

double foot_distance(const vector3& a, const vector3& b) {
    return std::hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

std::string planning_failure(const walking_plan_result& result) {
    std::string message;
    const tripod_cycle_result* cycle = nullptr;
    if (result.walking_failure) {
        const auto& failed = *result.walking_failure;
        message = "walking cycle " + std::to_string(failed.failed_cycle_index.value_or(0) + 1);
        if (failed.cycle_failure) {
            cycle = &*failed.cycle_failure;
        }
    } else if (result.recovery_failure) {
        message = "stance recovery cycle";
        cycle = &*result.recovery_failure;
    } else {
        message = "stance recovery terminal tolerance";
    }
    if (cycle && cycle->failed_frame_index) {
        message += ", frame " + std::to_string(*cycle->failed_frame_index);
    }
    auto failed_leg = result.terminal_failed_leg;
    if (cycle && cycle->initial_failed_leg) {
        failed_leg = cycle->initial_failed_leg;
    }
    if (cycle && cycle->segment_failure) {
        const auto& failed = *cycle->segment_failure;
        failed_leg = failed.failed_leg;
        if (failed.selection_failure) {
            message += ", IK selection status " + std::to_string(static_cast<int>(*failed.selection_failure));
        }
        if (failed.path_failure) {
            message += ", path status " + std::to_string(static_cast<int>(failed.path_failure->status));
        }
    }
    if (failed_leg) {
        message += ", leg " + std::to_string(leg_index(*failed_leg));
    }
    return message;
}

class planner_node final : public rclcpp::Node {
 public:
    planner_node()
        : Node("hexapod_planner"),
          model_(robot_model::from_file(startup_parameter<std::string>("hardware_config", ""))),
          mapper_(model_),
          kinematics_(model_),
          planner_(model_),
          system_clock_(RCL_SYSTEM_TIME) {
        configure();
        auto qos = rclcpp::QoS(rclcpp::KeepLast(256)).reliable().durability_volatile();
        plan_pub_ = create_publisher<ServoPlan>("servo_plan", qos);
        stop_pub_ = create_publisher<PlanStop>("plan_stop", qos);
        status_pub_ = create_publisher<PlannerStatus>("planner_status", qos);
        command_sub_ = create_subscription<MotionCommand>("motion_command", qos, [this](MotionCommand::ConstSharedPtr message) { on_command(*message); });
        reference_sub_ = create_subscription<StandingReference>("standing_reference", qos, [this](StandingReference::ConstSharedPtr message) { on_reference(*message); });
        execution_sub_ = create_subscription<ExecutionStatus>("execution_status", qos, [this](ExecutionStatus::ConstSharedPtr message) { on_execution(*message); });
        next_plan_id_ = static_cast<std::uint64_t>(system_clock_.now().nanoseconds());
        timer_ = create_wall_timer(20ms, [this] { tick(); });
        RCLCPP_INFO(get_logger(), "Waiting for a confirmed standing_reference");
    }

 private:
    template <typename T>
    T startup_parameter(const char* name, const T& default_value) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        return declare_parameter<T>(name, default_value, descriptor);
    }

    double number(const char* name, double default_value, double minimum, double maximum) {
        const auto value = startup_parameter(name, default_value);
        if (!std::isfinite(value) || value < minimum || value > maximum) {
            throw std::invalid_argument(std::string(name) + " is outside the allowed range");
        }
        return value;
    }

    int integer(const char* name, int default_value, int minimum, int maximum) {
        const auto value = startup_parameter<std::int64_t>(name, default_value);
        if (value < minimum || value > maximum) {
            throw std::invalid_argument(std::string(name) + " is outside the allowed range");
        }
        return static_cast<int>(value);
    }

    void configure() {
        timing_ = {number("cycle_period_s", 4.0, 0.01, 60.0), number("stance_ratio", 0.625, 0.500001, 0.999)};
        intervals_per_stage_ = static_cast<std::size_t>(integer("intervals_per_stage", 10, 1, 100));
        path_intervals_ = static_cast<std::size_t>(integer("path_interval_count", 4, 1, 100));
        max_cycles_ = static_cast<std::uint32_t>(integer("max_cycles", 16, 1, 100));
        lift_m_ = number("lift_height_m", 0.003, 0.000001, 0.2);
        recovery_lift_m_ = number("recovery_lift_height_m", 0.003, 0.000001, 0.2);
        standing_tolerance_m_ = number("standing_tolerance_m", 0.001, 0.000001, 0.05);
        const double speed = number("max_joint_speed_rad_s", 1.0, 0.000001, 20.0);
        for (auto& limits : speed_limits_) {
            limits.fill(speed);
        }
        const double clearance = number("foot_clearance_tolerance_m", 0.002, 0.000001, 0.03);
        clearance_ = foot_clearance_policy{clearance, clearance, clearance};

        max_linear_speed_ = number("max_linear_speed_m_s", 0.005, 0.000001, 0.5);
        max_yaw_speed_ = number("max_yaw_speed_rad_s", 0.03, 0.000001, 5.0);
        linear_step_ = number("linear_step_m_s", 0.0005, 0.000001, max_linear_speed_);
        yaw_step_ = number("yaw_step_rad_s", 0.003, 0.000001, max_yaw_speed_);
        linear_deadband_ = number("linear_deadband_m_s", 0.00005, 0.0, linear_step_);
        yaw_deadband_ = number("yaw_deadband_rad_s", 0.0005, 0.0, yaw_step_);
        time_scale_ = number("time_scale", 4.0, 1.0, 100.0);
        min_segment_ms_ = static_cast<std::uint16_t>(integer("min_segment_ms", 200, 1, 5000));
        max_segment_ms_ = static_cast<std::uint16_t>(integer("max_segment_ms", 5000, min_segment_ms_, 65535));
        command_timeout_ = milliseconds(integer("command_timeout_ms", 500, 1, 30000));
        center_hold_ = milliseconds(integer("center_hold_ms", 150, 0, static_cast<int>(command_timeout_.count())));
        filter_time_ms_ = number("teleop_filter_ms", 80.0, 1.0, 5000.0);
        lead_ = milliseconds(integer("plan_lead_ms", 350, 1, 30000));
        lookahead_ = milliseconds(integer("lookahead_ms", 1200, static_cast<int>(lead_.count()), 30000));
        feedback_grace_ = milliseconds(integer("feedback_grace_ms", 1000, 1, 30000));
        reference_age_ = milliseconds(integer("reference_max_age_ms", 3000, 1, 60000));

        const auto configured = startup_parameter("standing_positions", std::vector<std::int64_t>{});
        if (configured.size() != joint_count) {
            throw std::invalid_argument("standing_positions requires 18 explicit positions ordered by ID");
        }
        positions values{};
        for (std::size_t i = 0; i < joint_count; ++i) {
            if (configured[i] < 0 || configured[i] > 1000) {
                throw std::invalid_argument("standing_positions contains a non-native position");
            }
            values[i] = static_cast<std::uint16_t>(configured[i]);
        }
        const auto angles = decode(values);
        std::array<vector3, leg_count> feet{};
        for (const auto leg : leg_order) {
            const auto i = leg_index(leg);
            feet[i] = kinematics_.forward_in_body_frame(leg, angles[i]);
            stance_xy_[i] = {feet[i].x, feet[i].y};
            ground_height_m_ += feet[i].z / static_cast<double>(leg_count);
        }
        for (const auto& foot : feet) {
            if (std::abs(foot.z - ground_height_m_) > standing_tolerance_m_) {
                throw std::invalid_argument("standing_positions are not coplanar within standing_tolerance_m");
            }
        }
    }

    std::array<leg_joint_angles, leg_count> decode(const positions& values) const {
        std::array<leg_joint_angles, leg_count> angles{};
        for (const auto leg : leg_order) {
            std::array<double, joints_per_leg> per_leg{};
            for (const auto axis : joint_order) {
                const auto id = model_.joint(leg, axis).binding.id;
                const servo_position position{values[id.index()]};
                const double angle = mapper_.servo_position_to_joint_angle_rad(id, position);
                if (mapper_.joint_angle_rad_to_servo_position(id, angle).value() != position.value()) {
                    throw std::invalid_argument("Reference position cannot round-trip through the joint mapper");
                }
                per_leg[joint_index(axis)] = angle;
            }
            angles[leg_index(leg)] = {per_leg[0], per_leg[1], per_leg[2]};
        }
        return angles;
    }

    std::chrono::nanoseconds stamp_age(const builtin_interfaces::msg::Time& stamp) {
        if (stamp.sec < 0 || stamp.nanosec >= 1'000'000'000U) {
            throw std::invalid_argument("invalid system timestamp");
        }
        const auto age = system_clock_.now() - rclcpp::Time(stamp, RCL_SYSTEM_TIME);
        if (age.nanoseconds() < 0) {
            throw std::invalid_argument("system timestamp is in the future");
        }
        return std::chrono::nanoseconds(age.nanoseconds());
    }

    void report(std::uint64_t sequence, std::uint64_t plan, const std::string& state, const std::string& reason = {}) {
        PlannerStatus message;
        message.command_sequence = sequence;
        message.plan_id = plan;
        message.state = state;
        message.reason = reason;
        status_pub_->publish(message);
    }

    void on_reference(const StandingReference& message) {
        try {
            if (!pending_.empty() || mode_ != motion_mode::idle) {
                throw std::invalid_argument("cannot replace the standing reference while a plan is active");
            }
            if (stamp_age(message.stamp) >= reference_age_) {
                throw std::invalid_argument("standing reference expired");
            }
            const rclcpp::Time reference_stamp(message.stamp, RCL_SYSTEM_TIME);
            if (reference_watermark_ && reference_stamp <= *reference_watermark_) {
                throw std::invalid_argument("standing reference predates the most recent reference or fault");
            }
            const auto angles = decode(message.positions);
            std::array<vector3, leg_count> anchors{};
            for (const auto leg : leg_order) {
                const auto i = leg_index(leg);
                anchors[i] = {stance_xy_[i].x_m, stance_xy_[i].y_m, ground_height_m_};
                const auto foot = kinematics_.forward_in_body_frame(leg, angles[i]);
                if (foot_distance(foot, anchors[i]) > standing_tolerance_m_) {
                    throw std::invalid_argument("confirmed pose does not match the configured standing position");
                }
            }
            boundary_.emplace(boundary_state{
                {planar_body_pose{{0.0, 0.0, 0.0}, 0.0}, gait_foot_state(anchors), angles, {}},
                message.positions, zero_velocity
            });
            reference_watermark_ = reference_stamp;
            report(0, 0, "standing");
        } catch (const std::exception& error) {
            report(0, 0, "rejected", error.what());
        }
    }

    void on_command(const MotionCommand& message) {
        try {
            const auto received = steady_clock::now();
            const auto age = stamp_age(message.stamp);
            if (age >= command_timeout_) {
                throw std::invalid_argument("motion command expired");
            }
            if (last_sequence_ && message.sequence <= *last_sequence_) {
                throw std::invalid_argument("duplicate or out-of-order motion command");
            }
            const rclcpp::Time source_stamp(message.stamp, RCL_SYSTEM_TIME);
            if (last_source_stamp_ && source_stamp < *last_source_stamp_) {
                throw std::invalid_argument("motion command timestamp moved backwards");
            }
            if (message.kind != MotionCommand::TELEOP && message.kind != MotionCommand::WALK_CYCLES && message.kind != MotionCommand::STOP) {
                throw std::invalid_argument("unknown motion command kind");
            }
            if (message.kind != MotionCommand::STOP) {
                const planar_body_velocity velocity{message.vx_m_s, message.vy_m_s, message.wz_rad_s};
                if (!std::isfinite(linear_speed(velocity)) || !std::isfinite(velocity.wz_rad_s) ||
                    linear_speed(velocity) > max_linear_speed_ || std::abs(velocity.wz_rad_s) > max_yaw_speed_) {
                    throw std::invalid_argument("motion command exceeds configured velocity limits");
                }
            }
            if (message.kind == MotionCommand::WALK_CYCLES && (message.cycle_count == 0 || message.cycle_count > max_cycles_)) {
                throw std::invalid_argument("invalid requested cycle count");
            }
            if (commands_.size() == 256) {
                if (message.kind != MotionCommand::STOP) {
                    throw std::runtime_error("motion command queue is full");
                }
                commands_.clear();
            }
            const auto sampled = received - age;
            commands_.push_back({message, sampled, sampled + command_timeout_});
            last_sequence_ = message.sequence;
            last_source_stamp_ = source_stamp;
        } catch (const std::exception& error) {
            report(message.sequence, 0, "rejected", error.what());
        }
    }

    void request_plan_stop(std::uint64_t plan_id) {
        PlanStop stop;
        stop.plan_id = plan_id;
        stop_pub_->publish(stop);
    }

    void reset_mode() {
        mode_ = motion_mode::idle;
        target_ = zero_velocity;
        filtered_ = zero_velocity;
        centered_at_.reset();
        input_expires_.reset();
        filter_updated_.reset();
    }

    void invalidate(const std::string& reason) {
        const auto active = pending_.empty() ? 0 : pending_.front().id;
        for (const auto& plan : pending_) {
            request_plan_stop(plan.id);
        }
        pending_.clear();
        commands_.clear();
        boundary_.reset();
        reference_watermark_ = system_clock_.now();
        reset_mode();
        report(command_sequence_, active, "fault", reason);
    }

    void latch_stop(const std::string& reason) {
        if (mode_ == motion_mode::idle || mode_ == motion_mode::stopping) {
            return;
        }
        if (mode_ == motion_mode::finite && !pending_.empty()) {
            request_plan_stop(pending_.front().id);
        }
        mode_ = motion_mode::stopping;
        report(command_sequence_, pending_.empty() ? 0 : pending_.front().id, "stopping", reason);
    }

    void update_filter(monotonic_time now) {
        if (mode_ != motion_mode::teleop || !filter_updated_ || !input_expires_) {
            return;
        }
        const auto valid_end = std::min(now, *input_expires_);
        if (valid_end > *filter_updated_) {
            const double elapsed = std::chrono::duration<double, std::milli>(valid_end - *filter_updated_).count();
            const double alpha = -std::expm1(-elapsed / filter_time_ms_);
            filtered_ = {
                filtered_.vx_m_s + alpha * (target_.vx_m_s - filtered_.vx_m_s),
                filtered_.vy_m_s + alpha * (target_.vy_m_s - filtered_.vy_m_s),
                filtered_.wz_rad_s + alpha * (target_.wz_rad_s - filtered_.wz_rad_s)
            };
            filter_updated_ = valid_end;
        }
        if (centered_at_ && valid_end >= *centered_at_ + center_hold_) {
            latch_stop("joystick centered");
        } else if (now >= *input_expires_) {
            latch_stop("teleop command expired");
        }
    }

    tripod_sequence_cycle_parameters cycle(planar_body_velocity speed) const {
        return {timing_, intervals_per_stage_, speed, lift_m_};
    }

    std::vector<tripod_sequence_cycle_parameters> finite_cycles(const planar_body_velocity& peak, std::size_t count) const {
        const double linear = linear_speed(peak);
        const double yaw = std::abs(peak.wz_rad_s);
        const double ramp_fraction = std::min(
            linear == 0.0 ? 1.0 : linear_step_ / linear,
            yaw == 0.0 ? 1.0 : yaw_step_ / yaw);
        std::vector<tripod_sequence_cycle_parameters> result;
        result.reserve(count);
        for (std::size_t i = 0; i < count; ++i) {
            const auto steps = static_cast<double>(std::min(i + 1, count - i));
            result.push_back(cycle(scaled_velocity(peak, std::min(1.0, steps * ramp_fraction))));
        }
        return result;
    }

    void consume(const queued_command& input) {
        const auto& cmd = input.message;
        if (cmd.kind == MotionCommand::STOP) {
            command_sequence_ = cmd.sequence;
            if (mode_ == motion_mode::idle) {
                report(cmd.sequence, 0, boundary_ ? "standing" : "waiting_for_reference");
            } else {
                latch_stop("user stop");
            }
            return;
        }
        if (!boundary_) {
            report(cmd.sequence, 0, "rejected", "a confirmed standing reference is required");
            return;
        }
        if (cmd.kind == MotionCommand::WALK_CYCLES) {
            if (mode_ != motion_mode::idle || !pending_.empty()) {
                report(cmd.sequence, 0, "rejected", "a motion request is already active");
                return;
            }
            if (plan_pub_->get_subscription_count() == 0) {
                report(cmd.sequence, 0, "rejected", "no servo_plan subscriber");
                return;
            }
            const planar_body_velocity velocity{cmd.vx_m_s, cmd.vy_m_s, cmd.wz_rad_s};
            if (stopped(velocity)) {
                report(cmd.sequence, 0, "rejected", "a finite walk requires nonzero velocity");
                return;
            }
            command_sequence_ = cmd.sequence;
            try {
                if (prepare(*boundary_, finite_cycles(velocity, cmd.cycle_count), true, plan_kind::finite)) {
                    mode_ = motion_mode::finite;
                }
            } catch (const std::exception& error) {
                report(cmd.sequence, 0, "rejected", error.what());
            }
            return;
        }
        if (mode_ == motion_mode::finite || mode_ == motion_mode::stopping) {
            return;
        }
        // 同一批延迟送达的指令仍保留采样间隔。已经处理到的时间不回退。
        const auto sample_time = std::max(input.sampled, filter_updated_.value_or(input.sampled));
        update_filter(sample_time);
        if (mode_ == motion_mode::stopping || steady_clock::now() >= input.expires) {
            return;
        }
        const planar_body_velocity requested{cmd.vx_m_s, cmd.vy_m_s, cmd.wz_rad_s};
        const bool centered = linear_speed(requested) <= linear_deadband_ && std::abs(requested.wz_rad_s) <= yaw_deadband_;
        if (mode_ == motion_mode::idle && centered) {
            return;
        }
        mode_ = motion_mode::teleop;
        command_sequence_ = cmd.sequence;
        target_ = centered ? zero_velocity : requested;
        if (centered && !centered_at_) {
            centered_at_ = sample_time;
        } else if (!centered) {
            centered_at_.reset();
        }
        input_expires_ = input.expires;
        filter_updated_ = sample_time;
    }

    bool prepare(const boundary_state& begin, std::vector<tripod_sequence_cycle_parameters> cycles, bool recover, plan_kind kind) {
        const auto end_velocity = recover ? zero_velocity : cycles.back().velocity;
        const tripod_sequence_schedule_request request{
            begin.gait, std::move(cycles), stance_xy_, ground_height_m_,
            {knee_singularity_policy::reject, branch_continuity_policy::keep_reference_branch},
            speed_limits_, path_intervals_, standing_tolerance_m_, standing_tolerance_m_, clearance_
        };
        const auto recovery = recover ? std::optional<stance_recovery_parameters>{
            stance_recovery_parameters{timing_, intervals_per_stage_, recovery_lift_m_, stance_xy_, standing_tolerance_m_}
        } : std::nullopt;
        auto result = planner_.plan(request, recovery);
        if (!result.solution) {
            throw std::runtime_error(planning_failure(result));
        }
        auto& solution = *result.solution;
        if (solution.timeline.empty()) {
            if (pending_.empty()) {
                boundary_->velocity = zero_velocity;
                reset_mode();
                report(command_sequence_, 0, "standing", "already within standing tolerance");
            }
            return false;
        }
        if (kind == plan_kind::walking && (!input_expires_ || steady_clock::now() >= *input_expires_)) {
            latch_stop("teleop expired during planning");
            return false;
        }
        auto message = make_servo_plan(solution.timeline, time_scale_, min_segment_ms_, max_segment_ms_, ++next_plan_id_, begin.nominal_positions, solution.full_cycle_count);
        std::vector<cycle_checkpoint> checkpoints;
        if (kind == plan_kind::finite) {
            checkpoints.reserve(solution.full_cycle_count);
            for (std::size_t i = 0; i < message.segments.size(); ++i) {
                const auto& segment = message.segments[i];
                if (!segment.walking_cycle_end) {
                    continue;
                }
                const auto cycle_index = checkpoints.size();
                const milliseconds end_offset{segment.start_offset_ms + static_cast<std::uint64_t>(segment.move_duration_ms)};
                checkpoints.push_back({
                    i + 1, end_offset,
                    {std::move(solution.cycle_end_states.at(cycle_index)), segment.positions, request.cycles.at(cycle_index).velocity}
                });
            }
        }

        const auto now = steady_clock::now();
        const auto wall_now = system_clock_.now();
        const auto start = pending_.empty() ? now + lead_ : pending_.back().end;
        if (!pending_.empty() && start < now + lead_) {
            return false;
        }
        const auto wall_start = pending_.empty() ? wall_now + rclcpp::Duration(lead_) : pending_.back().wall_end;
        message.motion_start = static_cast<builtin_interfaces::msg::Time>(wall_start);
        const auto& last = message.segments.back();
        const milliseconds duration{last.start_offset_ms + static_cast<std::uint64_t>(last.move_duration_ms)};
        pending_.push_back({
            message.plan_id, command_sequence_, kind, message.segments.size(),
            {std::move(solution.final_state), last.positions, end_velocity},
            std::move(checkpoints), wall_start + rclcpp::Duration(duration), start, start + duration
        });
        try {
            plan_pub_->publish(message);
        } catch (...) {
            pending_.pop_back();
            throw;
        }
        report(command_sequence_, message.plan_id, "planned", std::to_string(solution.full_cycle_count) + " walking cycles");
        return true;
    }

    void on_execution(const ExecutionStatus& message) {
        if (message.plan_id == 0 && message.state == ExecutionStatus::FAILED) {
            invalidate("execution reference lost: " + message.reason);
            return;
        }
        const auto it = std::find_if(pending_.begin(), pending_.end(),
            [&message](const pending_plan& candidate) { return candidate.id == message.plan_id; });
        if (it == pending_.end()) {
            return;
        }
        switch (message.state) {
            case ExecutionStatus::ACCEPTED:
                it->accepted = true;
                report(it->sequence, it->id, "accepted");
                return;
            case ExecutionStatus::FAILED:
            case ExecutionStatus::REJECTED:
                invalidate("execution did not complete: " + message.reason);
                return;
            case ExecutionStatus::COMPLETED:
                if (it != pending_.begin() || !it->accepted ||
                    message.completed_segments != it->segment_count || steady_clock::now() < it->end) {
                    invalidate("execution completion was out of order, early, or incomplete");
                    return;
                }
                boundary_ = std::move(it->end_state);
                if (it->kind == plan_kind::finite || it->kind == plan_kind::recovery) {
                    reset_mode();
                    report(it->sequence, it->id, "standing");
                } else {
                    report(it->sequence, it->id, "cycle_complete");
                }
                pending_.pop_front();
                return;
            case ExecutionStatus::INTERRUPTED: {
                if (it != pending_.begin() || pending_.size() != 1 || it->kind != plan_kind::finite || mode_ != motion_mode::stopping) {
                    invalidate("unexpected execution interruption");
                    return;
                }
                if (message.completed_segments != 0) {
                    const auto point = std::find_if(it->checkpoints.begin(), it->checkpoints.end(),
                        [&message](const cycle_checkpoint& checkpoint) {
                            return checkpoint.completed_segments == message.completed_segments;
                        });
                    if (!it->accepted || point == it->checkpoints.end() || steady_clock::now() < it->start + point->end_offset) {
                        invalidate("execution interrupted outside a complete walking cycle boundary");
                        return;
                    }
                    boundary_ = std::move(point->state);
                }
                report(it->sequence, it->id, "interrupted", message.reason);
                pending_.pop_front();
                return;
            }
            default:
                invalidate("unknown execution status");
                return;
        }
    }

    void tick() {
        auto now = steady_clock::now();
        for (const auto& plan : pending_) {
            if ((!plan.accepted && now >= plan.start) || now >= plan.end + feedback_grace_) {
                invalidate("execution acknowledgement or completion timed out");
                return;
            }
        }
        while (!commands_.empty()) {
            auto input = std::move(commands_.front());
            commands_.pop_front();
            if (input.message.kind == MotionCommand::STOP || steady_clock::now() < input.expires) {
                consume(input);
            } else {
                report(input.message.sequence, 0, "rejected", "expired in the planner queue");
            }
        }
        now = steady_clock::now();
        update_filter(now);

        if (!boundary_ || (mode_ != motion_mode::teleop && mode_ != motion_mode::stopping) ||
            pending_.size() >= 2 || plan_pub_->get_subscription_count() == 0) {
            return;
        }
        if (!pending_.empty()) {
            const auto& tail = pending_.back();
            if (!tail.accepted || tail.kind == plan_kind::finite || tail.kind == plan_kind::recovery ||
                tail.end - now > lookahead_ || tail.end - now < lead_) {
                return;
            }
        }
        const auto& begin = pending_.empty() ? *boundary_ : pending_.back().end_state;
        try {
            if (mode_ == motion_mode::teleop) {
                const auto next = approach_velocity(begin.velocity, filtered_, linear_step_, yaw_step_);
                if (!stopped(next) && (!stopped(target_) || !stopped(begin.velocity))) {
                    prepare(begin, {cycle(next)}, false, plan_kind::walking);
                }
            } else {
                const auto next = approach_velocity(begin.velocity, zero_velocity, linear_step_, yaw_step_);
                if (stopped(next)) {
                    prepare(begin, {}, true, plan_kind::recovery);
                } else {
                    prepare(begin, {cycle(next)}, false, plan_kind::braking);
                }
            }
        } catch (const std::exception& error) {
            if (mode_ == motion_mode::teleop) {
                latch_stop(error.what());
            } else {
                invalidate(error.what());
            }
        }
    }

    robot_model model_;
    joint_mapper mapper_;
    leg_kinematics kinematics_;
    walking_planner planner_;
    rclcpp::Clock system_clock_;
    tripod_timing_request timing_{4.0, 0.625};
    std::size_t intervals_per_stage_{};
    std::size_t path_intervals_{};
    std::uint32_t max_cycles_{};
    double lift_m_{};
    double recovery_lift_m_{};
    double standing_tolerance_m_{};
    double ground_height_m_{};
    std::array<stance_midpoint_reference, leg_count> stance_xy_{};
    std::array<std::array<double, joints_per_leg>, leg_count> speed_limits_{};
    std::optional<foot_clearance_policy> clearance_;
    double max_linear_speed_{};
    double max_yaw_speed_{};
    double linear_step_{};
    double yaw_step_{};
    double linear_deadband_{};
    double yaw_deadband_{};
    double filter_time_ms_{};
    double time_scale_{};
    std::uint16_t min_segment_ms_{};
    std::uint16_t max_segment_ms_{};
    milliseconds command_timeout_{};
    milliseconds center_hold_{};
    milliseconds lead_{};
    milliseconds lookahead_{};
    milliseconds feedback_grace_{};
    milliseconds reference_age_{};

    motion_mode mode_{motion_mode::idle};
    planar_body_velocity target_ = zero_velocity;
    planar_body_velocity filtered_ = zero_velocity;
    std::optional<monotonic_time> centered_at_;
    std::optional<monotonic_time> input_expires_;
    std::optional<monotonic_time> filter_updated_;
    std::optional<rclcpp::Time> reference_watermark_;
    std::optional<rclcpp::Time> last_source_stamp_;
    std::optional<std::uint64_t> last_sequence_;
    std::uint64_t command_sequence_ = 0;
    std::uint64_t next_plan_id_ = 0;
    std::optional<boundary_state> boundary_;
    std::deque<queued_command> commands_;
    std::deque<pending_plan> pending_;
    rclcpp::Publisher<ServoPlan>::SharedPtr plan_pub_;
    rclcpp::Publisher<PlanStop>::SharedPtr stop_pub_;
    rclcpp::Publisher<PlannerStatus>::SharedPtr status_pub_;
    rclcpp::Subscription<MotionCommand>::SharedPtr command_sub_;
    rclcpp::Subscription<StandingReference>::SharedPtr reference_sub_;
    rclcpp::Subscription<ExecutionStatus>::SharedPtr execution_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace hexapod_control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int status = 0;
    try {
        auto node = std::make_shared<hexapod_control::planner_node>();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
        executor.remove_node(node);
    } catch (const std::exception& error) {
        RCLCPP_ERROR(rclcpp::get_logger("hexapod_planner"), "%s", error.what());
        status = 1;
    }
    rclcpp::shutdown();
    return status;
}
