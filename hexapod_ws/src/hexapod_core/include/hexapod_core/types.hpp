#pragma once

// 本文件只定义核心数据类型，不依赖 ROS、串口或 JSON 库。
// 自定义标识符统一使用下划线命名；内部几何使用米、弧度、秒。
// 物理参数没有确定时使用 std::optional，不能用数值 0 代替“未知”。

#include <array>
#include <chrono>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>

namespace hexapod_core {

inline constexpr std::size_t leg_count = 6;
inline constexpr std::size_t joints_per_leg = 3;
inline constexpr std::size_t joint_count = leg_count * joints_per_leg;
inline constexpr double pi = 3.14159265358979323846;

// 将角度制输入 degrees 换算为弧度；只做单位换算，不检查或限制角度范围。
constexpr double degrees_to_radians(double degrees) noexcept {
  return degrees * (pi / 180.0);
}

// 将弧度输入 radians 换算为角度制，便于显示或对照配置；不执行范围检查。
constexpr double radians_to_degrees(double radians) noexcept {
  return radians * (180.0 / pi);
}

// 左右以前进方向为准，指机器人自身左右，和图像观察者的左右无关。
enum class leg_id {
  right_front, right_middle, right_rear,
  left_front, left_middle, left_rear
};

// 每条腿从机身向脚端依次排列；不在类型层推断每个轴的数学正方向。
enum class joint_type { body_yaw, upper_leg_pitch, lower_leg_pitch };

inline constexpr std::array<leg_id, leg_count> leg_order{
    leg_id::right_front, leg_id::right_middle, leg_id::right_rear,
    leg_id::left_front, leg_id::left_middle, leg_id::left_rear};
inline constexpr std::array<joint_type, joints_per_leg> joint_order{
    joint_type::body_yaw, joint_type::upper_leg_pitch, joint_type::lower_leg_pitch};

// 将合法腿枚举转换为 0～5 的数组下标；非法枚举抛 out_of_range。
constexpr std::size_t leg_index(leg_id leg) {
  const auto index = static_cast<std::size_t>(leg);
  if (index >= leg_count) throw std::out_of_range("invalid leg_id");
  return index;
}

// 将腿内关节类型转换为 0～2 的下标；非法枚举抛 out_of_range。
constexpr std::size_t joint_index(joint_type joint) {
  const auto index = static_cast<std::size_t>(joint);
  if (index >= joints_per_leg) throw std::out_of_range("invalid joint_type");
  return index;
}

// 将腿和关节类型组合成 0～17 的逻辑下标，同时检查两种枚举是否有效。
// 这不是物理舵机 ID；两者必须通过 robot_model 关联。
constexpr std::size_t joint_index(leg_id leg, joint_type joint) {
  return leg_index(leg) * joints_per_leg + joint_index(joint);
}

// 当前机器只允许 1～18；禁止默认构造，避免产生 ID=0 的假关节。
class servo_id {
 public:
  // 从物理编号构造 ID；只接受 1～18，越界时抛 out_of_range。
  explicit constexpr servo_id(int value) : value_(value) {
    if (value < 1 || value > static_cast<int>(joint_count)) {
      throw std::out_of_range("servo_id must be in 1..18");
    }
  }
  // 取出原始物理编号（1～18），用于协议或显示，不是数组下标。
  constexpr int value() const noexcept { return value_; }
  // 将已检查的编号转换为 ID-1 下标（0～17），用于按物理 ID 排列的数组。
  constexpr std::size_t index() const noexcept {
    return static_cast<std::size_t>(value_ - 1);
  }
  // 比较两个 ID 是否代表同一个物理舵机。
  friend constexpr bool operator==(servo_id left, servo_id right) noexcept {
    return left.value_ == right.value_;
  }
  // 判断两个 ID 是否不同，复用相等比较以保持判断规则一致。
  friend constexpr bool operator!=(servo_id left, servo_id right) noexcept {
    return !(left == right);
  }

 private:
  int value_;
};

// LX-225 的原生位置值，只有协议范围检查，不代表安装后机械可达。
// 500 的姿态含义由硬件配置提供；此类不将它解释为关节零位或站姿。
class servo_position {
 public:
  // 从整数构造原生位置；超出 0～1000 抛 out_of_range，不检查安装机械限位。
  explicit constexpr servo_position(int value) : value_(value) {
    if (value < 0 || value > 1000) {
      throw std::out_of_range("servo_position must be in 0..1000");
    }
  }
  // 取出原生整数位置，用于换算或协议编码；返回值不是角度。
  constexpr int value() const noexcept { return value_; }

 private:
  int value_;
};

// 表示位置值增加时，关节角增加或减少；必须逐轴确定。
enum class rotation_sign : int { negative = -1, positive = 1 };

struct angle_limits {
  double minimum_rad;
  double maximum_rad;
};

struct position_limits {
  int minimum;
  int maximum;
};

struct vector3 {
  double x;
  double y;
  double z;
};

struct rigid_transform {
  vector3 translation_m;
  // 行优先的 3×3 旋转矩阵：将腿根局部坐标旋转到机身坐标。
  std::array<double, 9> rotation;
};

struct foot_contact_point {
  // 坐标系必须明确，不能把一组数默认当作第三轴/机身坐标。
  std::string frame;
  vector3 position_m;
};

// 六条腿共用的理想关节坐标约定；向量均在各腿的零位局部坐标系中表达。
// 当前零位沿 -Y 水平伸直，偏航正轴为 +Z，两个俯仰正轴为 -X。
// positive_joint_axes_at_zero 按 joint_order 排列；动作后需随前级关节旋转，
// 不能把这些零位轴向直接当作任意姿态下的机身固定轴向。
struct kinematic_convention {
  vector3 zero_leg_direction;
  std::array<vector3, joints_per_leg> positive_joint_axes_at_zero;
};

// 可选数值与来源一起保存。status 保留原配置描述，不自动升级为“实测”。
template <typename value_type>
struct physical_parameter {
  std::optional<value_type> value;
  std::string source;
  std::string status;
};

// 关节参考角：舵机处于 reference_position 时的关节角；当前 500 对应 0 rad。
// 一个字段已确定，并不表示其他字段或整条腿也已完成标定。
struct joint_calibration {
  // 舵机相对角转换到关节角时使用的方向符号，只能为 +1 或 -1。
  std::optional<rotation_sign> servo_to_joint_sign;
  // 单位为弧度；与方向符号一起完成 joint_angle = reference + sign × relative。
  std::optional<double> reference_joint_angle_rad;
  std::optional<angle_limits> installed_limits;
};

struct joint_binding {
  leg_id leg;
  joint_type joint;
  servo_id id;
};

struct joint_model {
  joint_binding binding;
  joint_calibration calibration;
};

struct leg_model {
  leg_id leg;
  std::array<servo_id, joints_per_leg> servo_ids;
  // 机身到第一轴的安装位置，与三段腿长分开。单位：米。
  std::optional<vector3> root_position_m;
  std::optional<vector3> root_axis_direction;
  std::optional<rigid_transform> body_to_first_axis;
};

struct leg_geometry {
  physical_parameter<double> servo_axis_to_far_end_l1_m;
  physical_parameter<double> first_leg_length_l2_m;
  physical_parameter<double> servo_far_end_gap_d_m;
  physical_parameter<double> second_leg_length_m;
  physical_parameter<double> third_leg_length_to_metal_end_l3_m;
  // 当前值为 CAD 简化的理想点足；frame 指定第三轴上随小腿转动的坐标系。
  // 名称沿用历史接口，是否实测必须看 source/status，不能因 actual 一词而推断。
  physical_parameter<foot_contact_point> actual_foot_contact_point_m;
};

struct servo_spec {
  std::string model;
  position_limits native_position_limits;
  angle_limits native_angle_limits;
  double radians_per_position_unit;
  servo_position reference_position;
  std::string reference_position_meaning;
};

// 以下是数据容器，不负责执行动作。目标与原始回读分开，防止补偿后
// 命令被误当作原始目标。数组均按物理 ID-1 索引，和腿数组下标区分。
using joint_angles = std::array<std::optional<double>, joint_count>;  // 弧度
using servo_positions = std::array<std::optional<servo_position>, joint_count>;
using monotonic_time = std::chrono::steady_clock::time_point;

struct joint_targets {
  joint_angles desired_angles_rad{};
  servo_positions nominal_positions{};
  servo_positions commanded_positions{};
};

struct joint_observation {
  servo_positions read_positions{};  // 缺失轴为 nullopt，不用 0/缓存值冒充
  joint_angles read_angles_rad{};    // 未完成角度映射时仍可保留原始位置
  // 协议没有精确采样时间；记录请求到收包的区间，不声称同时采样。
  std::optional<monotonic_time> request_started_at;
  std::optional<monotonic_time> response_completed_at;
};

}  // namespace hexapod_core
