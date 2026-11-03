#include "hexapod_core/joint_mapper.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace hexapod_core {
namespace {
// 只吸收浮点往返误差，不是允许越过机械限位的控制容差。
constexpr double numerical_tolerance = 1e-12;
}

// 内部前置检查：确认 id 对应关节已填写方向符号和关节参考角，并返回其只读模型。
// 缺少任意一项时抛 logic_error，防止解引用未知标定数据。
const joint_model& joint_mapper::require_mapping(servo_id id) const {
  if (!model_.is_mapping_ready(id)) {
    throw std::logic_error("servo " + std::to_string(id.value()) +
                           ": joint sign or reference angle is not calibrated");
  }
  return model_.joint(id);
}

// 检查 position 是否落在配置指定的原生位置范围内，超出时抛 out_of_range。
// servo_position 类型只检查协议总范围，这里进一步检查配置中的可用子范围。
void joint_mapper::require_native_position(servo_position position) const {
  const auto limits = model_.servo().native_position_limits;
  if (position.value() < limits.minimum || position.value() > limits.maximum) {
    throw std::out_of_range("position exceeds configured native servo limits");
  }
}

// 舵机位置值 → 舵机相对角（弧度）；参考位置值由配置提供，当前为 500。
// 正方向统一为位置值增大方向，不使用逐轴方向符号和关节参考角。
double joint_mapper::servo_position_to_servo_relative_angle_rad(servo_position position) const {
  require_native_position(position);
  const auto& spec = model_.servo();
  return (position.value() - spec.reference_position.value()) * spec.radians_per_position_unit;
}

// 舵机位置值 → 关节角（弧度），按指定 ID 应用方向符号和关节参考角。
// 缺少标定或原生位置超限时报错；允许解码超出安装限位的反馈以便诊断。
double joint_mapper::servo_position_to_joint_angle_rad(servo_id id, servo_position position) const {
  const auto& calibration = require_mapping(id).calibration;
  // 关节角 = 关节参考角 + 方向符号 × 舵机相对角。
  // 正方向来自该 ID 自己的标定，不来自腿的左右位置。
  return *calibration.reference_joint_angle_rad +
      static_cast<int>(*calibration.servo_to_joint_sign) *
      servo_position_to_servo_relative_angle_rad(position);
}

// 关节角（弧度）→ 舵机位置值，不含到位误差补偿。
// 要求方向符号、关节参考角、安装限位齐全；检查输入及取整后关节角的限位。
// 非有限角抛 invalid_argument，缺标定抛 logic_error，超限抛 out_of_range；不截断。
servo_position joint_mapper::joint_angle_rad_to_servo_position(servo_id id, double joint_angle_rad) const {
  if (!std::isfinite(joint_angle_rad)) throw std::invalid_argument("joint angle must be finite radians");
  const auto& calibration = require_mapping(id).calibration;
  if (!calibration.installed_limits) {
    throw std::logic_error("servo " + std::to_string(id.value()) + ": installed joint limits are unknown");
  }
  const auto limits = *calibration.installed_limits;
  if (joint_angle_rad < limits.minimum_rad - numerical_tolerance ||
      joint_angle_rad > limits.maximum_rad + numerical_tolerance) {
    throw std::out_of_range("target exceeds installed joint angle limits");
  }
  const auto& spec = model_.servo();
  // sign 只能为 ±1，其倒数仍是自身。此处求上述公式的逆，尚未加入
  // 关节到位误差补偿；之后的补偿模块应在这个名义位置上另外处理。
  const double position = spec.reference_position.value() +
      static_cast<int>(*calibration.servo_to_joint_sign) *
      (joint_angle_rad - *calibration.reference_joint_angle_rad) / spec.radians_per_position_unit;
  // 在 lround 前检查，避免极端角度导致整数溢出。
  if (!std::isfinite(position) || position < spec.native_position_limits.minimum - numerical_tolerance ||
      position > spec.native_position_limits.maximum + numerical_tolerance) {
    throw std::out_of_range("target exceeds native servo position limits");
  }
  const servo_position rounded(static_cast<int>(std::lround(position)));
  const double rounded_joint_angle_rad = servo_position_to_joint_angle_rad(id, rounded);
  if (rounded_joint_angle_rad < limits.minimum_rad - numerical_tolerance ||
      rounded_joint_angle_rad > limits.maximum_rad + numerical_tolerance) {
    throw std::out_of_range("rounded target would exceed installed joint limits");
  }
  return rounded;
}

}  // namespace hexapod_core
