#pragma once

#include "hexapod_core/types.hpp"

#include <string>
#include <utility>
#include <vector>

namespace hexapod_core {

// 硬件配置的只读快照：保留原始来源和未知项，不包含运动或串口操作。
// 加载成功只代表格式、编号和已填数值有效，不代表全部物理标定完成。
class robot_model {
 public:
  // 从 path 指向的 JSON 文件加载模型；支持 schema_version=2。
  // 返回配置快照；文件读取、格式或语义错误抛 std::runtime_error。
  static robot_model from_file(const std::string& path);
  // 从内存中的 JSON 文本加载模型，适用于测试或已经取得配置文本的调用方。
  // 校验规则与 from_file 相同；未知物理字段仍保留为空。
  static robot_model from_json_text(const std::string& text);

  // 返回舵机通用规格的只读引用，包括原生范围、比例及展开参考位置。
  const servo_spec& servo() const noexcept { return servo_; }
  // 返回几何参数的只读引用，同时保留各参数的来源、状态和未知项。
  const leg_geometry& geometry() const noexcept { return geometry_; }
  // 返回理想零位与数学正转轴约定；旧配置缺少该项时保留为空，不补默认值。
  const std::optional<kinematic_convention>& convention() const noexcept {
    return convention_;
  }
  // 返回按 leg_order 排列的六条腿；只读访问，不复制容器。
  const std::vector<leg_model>& legs() const noexcept { return legs_; }
  // 返回按物理 ID 升序排列的关节列表，其下标对应 ID-1。
  const std::vector<joint_model>& joints() const noexcept { return joints_; }
  // 返回机身坐标原点的可选说明；未定义时为空，不推断原点位置。
  const std::optional<std::string>& body_origin_description() const noexcept {
    return body_origin_;
  }

  // 按腿枚举查询一条腿的只读模型；非法枚举抛 out_of_range。
  const leg_model& leg(leg_id leg) const;
  // 按已检查的物理舵机 ID 查询关节，返回模型内的只读引用。
  const joint_model& joint(servo_id id) const;
  // 按腿和关节类型查询绑定的关节；非法枚举抛 out_of_range。
  const joint_model& joint(leg_id leg, joint_type joint) const;

  // 方向符号与关节参考角均已填写时返回 true，表示该轴可做位置值到关节角的映射。
  bool is_mapping_ready(servo_id id) const;
  // 映射可用且已填写安装限位时返回 true；不代表已经完成实机验证。
  bool is_command_ready(servo_id id) const;

 private:
  // 内部构造入口：先保存舵机规格，其余配置由静态加载函数继续填写和校验。
  // 不对外开放，避免调用方绕过加载检查自行构造不完整模型。
  explicit robot_model(servo_spec spec) : servo_(std::move(spec)) {}

  servo_spec servo_;
  leg_geometry geometry_;
  std::optional<kinematic_convention> convention_;
  std::vector<leg_model> legs_;
  std::vector<joint_model> joints_;
  std::optional<std::string> body_origin_;  // 未定义时不假定原点在中心
};

}  // namespace hexapod_core
