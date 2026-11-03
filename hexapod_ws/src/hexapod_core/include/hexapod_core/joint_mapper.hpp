#pragma once

#include "hexapod_core/robot_model.hpp"

namespace hexapod_core {

// 统一术语（角度计算单位均为弧度，名称后缀 _rad）：
// 1. 舵机位置值 servo_position：协议整数值；当前命令限幅为 140～860。
// 2. 舵机相对角 servo_relative_angle_rad：相对参考位置 500 的转角，位置值增大为正。
// 3. 关节角 joint_angle_rad：运动学使用的角度，正方向由该关节的数学轴决定。
// 4. 关节参考角 reference_joint_angle_rad：舵机处于参考位置时的关节角，当前均为 0。
// 关系：关节角 = 关节参考角 + 方向符号 × 舵机相对角。
// 本类只做上述名义换算，不包含到位误差补偿或串口协议。
// 持有配置快照，避免构造时传入临时 robot_model 后出现悬空引用。
class joint_mapper {
 public:
  // 保存一份模型快照；支持传入已有模型或临时模型，不依赖外部对象的生命周期。
  explicit joint_mapper(robot_model model) : model_(std::move(model)) {}

  // 舵机位置值 → 关节角（弧度）；使用指定 ID 的方向符号与关节参考角。
  // 缺少其中任一参数抛 std::logic_error；允许解码超出安装限位的反馈。
  double servo_position_to_joint_angle_rad(servo_id id, servo_position position) const;

  // 关节角（弧度）→ 舵机位置值；要求方向符号、关节参考角和安装限位齐全。
  // 非有限角抛 invalid_argument，超限抛 out_of_range，不自动截断目标。
  // 取最近整数后再次检查角度，防止量化把边界内目标推到边界外。
  servo_position joint_angle_rad_to_servo_position(servo_id id, double joint_angle_rad) const;

  // 舵机位置值 → 舵机相对角（弧度）：(位置值 - 参考位置值) × 每单位弧度数。
  // 不使用关节 ID、方向符号或关节参考角；当前输入 550 返回约 +0.20944 rad（+12°）。
  double servo_position_to_servo_relative_angle_rad(servo_position position) const;

 private:
  // 确认该 ID 的方向符号和关节参考角齐全；缺参数抛 logic_error。
  const joint_model& require_mapping(servo_id id) const;
  // 检查原生位置是否位于配置范围；超出时抛 out_of_range。
  void require_native_position(servo_position position) const;
  robot_model model_;
};

}  // namespace hexapod_core
