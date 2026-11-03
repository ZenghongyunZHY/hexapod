#include "hexapod_core/robot_model.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace hexapod_core {
// 以下是本文件内部的配置读取、校验和类型转换工具。
namespace {
using json = nlohmann::json;

// 统一报告配置错误：message 描述出错字段及原因。
// 添加 robot_model 前缀后抛出 runtime_error；此函数不会正常返回。
[[noreturn]] void config_error(const std::string& message) {
  throw std::runtime_error("robot_model: " + message);
}

// 从 value 严格读取 int；path 是报错时显示的配置字段路径。
// 拒绝布尔值、小数和超出 int 范围的整数，避免错误的 ID 被隐式转换。
int read_integer(const json& value, const std::string& path) {
  if (!value.is_number_integer()) config_error(path + " must be an integer");
  if (value.is_number_unsigned()) {
    const auto number = value.get<std::uint64_t>();
    if (number > static_cast<std::uint64_t>(std::numeric_limits<int>::max())) {
      config_error(path + " integer overflow");
    }
    return static_cast<int>(number);
  }
  const auto number = value.get<std::int64_t>();
  if (number < std::numeric_limits<int>::min() ||
      number > std::numeric_limits<int>::max()) config_error(path + " integer overflow");
  return static_cast<int>(number);
}

// 从 value 读取有限 double，允许 JSON 整数或小数。
// 非数值、NaN 或无穷值会报错；path 用于定位配置字段。
double read_number(const json& value, const std::string& path) {
  if (!value.is_number()) config_error(path + " must be numeric");
  const double number = value.get<double>();
  if (!std::isfinite(number)) config_error(path + " must be finite");
  return number;
}

// 确认 value 是恰好含 size 个元素的数组；成功时不返回数据。
// 类型或长度不符合要求时，使用 path 指明出错字段。
void require_size(const json& value, std::size_t size, const std::string& path) {
  if (!value.is_array() || value.size() != size) {
    config_error(path + " must be an array of length " + std::to_string(size));
  }
}

// JSON 中的腿名称，顺序与 leg_order 一致，用于字符串和枚举之间的对应。
const std::array<std::string, leg_count> leg_names{
    "right_front", "right_middle", "right_rear",
    "left_front", "left_middle", "left_rear"};

// 将 JSON 的腿名称（如 right_front）转换成 leg_id；未知名称会报错。
leg_id read_leg(const std::string& name) {
  const auto found = std::find(leg_names.begin(), leg_names.end(), name);
  if (found == leg_names.end()) config_error("unknown body_role: " + name);
  return leg_order[static_cast<std::size_t>(found - leg_names.begin())];
}

// 检查按舵机 ID 索引的可选配置表：只接受 null 或键为 "1"～"18" 的对象。
// 此处只检查表结构和键，各项具体值由后续代码校验；path 用于报错定位。
void validate_joint_table(const json& table, const std::string& path) {
  if (table.is_null()) return;
  if (!table.is_object()) config_error(path + " must be null or an ID-keyed object");
  for (auto entry = table.begin(); entry != table.end(); ++entry) {
    bool valid = false;
    for (int id = 1; id <= static_cast<int>(joint_count); ++id) {
      if (entry.key() == std::to_string(id)) valid = true;
    }
    if (!valid) config_error(path + " has invalid servo ID key: " + entry.key());
  }
}

// 从可选配置表中按 id 查找一项；table 应先经过 validate_joint_table 检查。
// 找到时返回该项的只读引用；表为空或缺项时返回静态 null，保留未知状态。
const json& optional_joint_entry(const json& table, int id) {
  static const json absent = nullptr;
  const auto key = std::to_string(id);
  return table.is_object() && table.contains(key) ? table.at(key) : absent;
}

// 检查按腿名称索引的可选配置表：接受 null，或只包含合法腿名称的对象。
// 不要求六条腿全部填写，也不在这里检查各项的坐标等具体内容。
void validate_leg_table(const json& table, const std::string& path) {
  if (table.is_null()) return;
  if (!table.is_object()) config_error(path + " must be null or a body_role-keyed object");
  for (auto entry = table.begin(); entry != table.end(); ++entry) {
    (void)read_leg(entry.key());
  }
}

// 按 leg 对应的名称查找可选配置；table 应先经过 validate_leg_table 检查。
// 找到时返回该项的只读引用；表为空或缺项时返回静态 null，不补默认坐标。
const json& optional_leg_entry(const json& table, leg_id leg) {
  static const json absent = nullptr;
  const auto& key = leg_names.at(leg_index(leg));
  return table.is_object() && table.contains(key) ? table.at(key) : absent;
}

// 将长度为 3 的数值数组转换为 vector3，每个分量都乘以 scale。
// 例如 scale=0.001 将毫米换成米，scale=1.0 保持原值；path 用于报错定位。
vector3 read_vector(const json& value, double scale, const std::string& path) {
  require_size(value, 3, path);
  return {read_number(value.at(0), path) * scale,
          read_number(value.at(1), path) * scale,
          read_number(value.at(2), path) * scale};
}

// 方向没有长度单位，必须是单位向量；拒绝零向量或错误缩放，避免轴向含义改变。
vector3 read_unit_vector(const json& value, const std::string& path) {
  const auto direction = read_vector(value, 1.0, path);
  const double norm = std::hypot(direction.x, direction.y, direction.z);
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-6) {
    config_error(path + " must be a unit vector");
  }
  return direction;
}

// 从对象读取刚体变换：translation_mm 转成米，rotation 按行存入 3×3 矩阵。
// 检查矩阵尺寸、数值、正交性及行列式，拒绝把缩放或镜像当作旋转。
rigid_transform read_transform(const json& value, const std::string& path) {
  rigid_transform result{read_vector(value.at("translation_mm"), 0.001, path), {}};
  const auto& rotation = value.at("rotation");
  require_size(rotation, 3, path + ".rotation");
  for (std::size_t row = 0; row < 3; ++row) {
    require_size(rotation.at(row), 3, path + ".rotation row");
    for (std::size_t col = 0; col < 3; ++col) {
      result.rotation[row * 3 + col] = read_number(rotation.at(row).at(col), path);
    }
  }
  // 必须是正交且右手的旋转，不能把镜像矩阵误当作刚体旋转。
  const auto& r = result.rotation;
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t other = 0; other < 3; ++other) {
      double dot = 0.0;
      for (std::size_t col = 0; col < 3; ++col) dot += r[row * 3 + col] * r[other * 3 + col];
      if (std::abs(dot - (row == other ? 1.0 : 0.0)) > 1e-6) {
        config_error(path + ".rotation must be orthonormal");
      }
    }
  }
  const double determinant = r[0] * (r[4] * r[8] - r[5] * r[7]) -
      r[1] * (r[3] * r[8] - r[5] * r[6]) + r[2] * (r[3] * r[7] - r[4] * r[6]);
  if (std::abs(determinant - 1.0) > 1e-6) config_error(path + ".rotation must be right-handed");
  return result;
}

// 从 geometry 的 key 字段读取长度，并随结果保存来源和测量状态。
// 数值由毫米换成米；null 保留为未知。默认要求正数，allow_zero 可允许零。
physical_parameter<double> read_length(const json& geometry, const std::string& key,
                                       bool allow_zero = false) {
  physical_parameter<double> result{std::nullopt,
      geometry.at("source").get<std::string>(),
      geometry.at("measurement_status").get<std::string>()};
  const auto& value = geometry.at(key);
  if (!value.is_null()) {
    const double millimeters = read_number(value, "model_geometry." + key);
    if (millimeters < 0.0 || (!allow_zero && millimeters == 0.0)) {
      config_error("model_geometry." + key + " has invalid length");
    }
    result.value = millimeters * 0.001;
  }
  return result;
}

}  // namespace

// 配置加载入口：打开 path 指向的文件，读取全文后交给 from_json_text 构建模型。
// 返回只读模型快照；文件无法打开、读取失败或配置无效时抛 runtime_error。
robot_model robot_model::from_file(const std::string& path) {
  std::ifstream input(path);
  if (!input) config_error("cannot open file: " + path);
  const std::string text{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
  if (input.bad()) config_error("failed to read file: " + path);
  return from_json_text(text);
}

// 从 JSON 文本构建模型：依次校验版本/单位、舵机规格、安装信息和几何信息。
// 将输入单位转换为内部单位，保留未知项；成功不等于物理标定全部完成。
// JSON 格式、字段类型或硬件约束不满足时，统一报告 runtime_error。
robot_model robot_model::from_json_text(const std::string& text) {
  try {
    // nlohmann 默认接受重复键并覆盖；配置禁止这种歧义，解析时直接拒绝。
    std::vector<std::set<std::string>> object_keys;
    // 解析回调：为每层 JSON 对象维护键集合，遇到同一对象内的重复键立即报错。
    const auto reject_duplicate_keys = [&object_keys](int, json::parse_event_t event, json& parsed) {
      if (event == json::parse_event_t::object_start) object_keys.emplace_back();
      if (event == json::parse_event_t::key &&
          !object_keys.back().insert(parsed.get<std::string>()).second) {
        config_error("duplicate JSON key: " + parsed.get<std::string>());
      }
      if (event == json::parse_event_t::object_end) object_keys.pop_back();
      return true;
    };
    const auto root = json::parse(text, reject_duplicate_keys);
    // 第一部分：检查配置版本、输入单位与关节顺序。输入仍用既有 mm/degree，
    // 只在加载边界换算一次，后续算法不再混合使用毫米与米。
    if (read_integer(root.at("schema_version"), "schema_version") != 2) {
      config_error("only schema_version 2 is supported");
    }
    const auto& units = root.at("units");
    if (units.at("length") != "mm" || units.at("angle") != "degree") {
      config_error("schema 2 requires units mm and degree");
    }
    const auto& joint_names = root.at("joint_order");
    if (joint_names != json::array({"body_yaw", "upper_leg_pitch", "lower_leg_pitch"})) {
      config_error("unexpected joint_order");
    }
    const auto& servo = root.at("servo");
    if (read_integer(servo.at("count"), "servo.count") != static_cast<int>(joint_count)) {
      config_error("this robot requires exactly 18 servos");
    }
    require_size(servo.at("position_range"), 2, "servo.position_range");
    require_size(servo.at("angle_range_deg"), 2, "servo.angle_range_deg");
    const position_limits positions{
        read_integer(servo.at("position_range").at(0), "position_range.minimum"),
        read_integer(servo.at("position_range").at(1), "position_range.maximum")};
    if (positions.minimum < 0 || positions.maximum > 1000 || positions.minimum >= positions.maximum) {
      config_error("native position range must be ordered and within 0..1000");
    }
    const double scale_deg = read_number(servo.at("degrees_per_position_unit"), "degrees_per_position_unit");
    const double angle_min_deg = read_number(servo.at("angle_range_deg").at(0), "angle_range_deg.minimum");
    const double angle_max_deg = read_number(servo.at("angle_range_deg").at(1), "angle_range_deg.maximum");
    const int reference = read_integer(servo.at("reference_position"), "reference_position");
    if (scale_deg <= 0.0 || degrees_to_radians(scale_deg) <= 0.0 ||
        angle_min_deg >= angle_max_deg ||
        !std::isfinite(angle_max_deg - angle_min_deg) ||
        std::abs((positions.maximum - positions.minimum) * scale_deg -
                 (angle_max_deg - angle_min_deg)) > 1e-6) {
      config_error("native angle range and position scale disagree");
    }
    if (reference < positions.minimum || reference > positions.maximum) {
      config_error("reference_position is outside native limits");
    }
    robot_model result(servo_spec{servo.at("model").get<std::string>(), positions,
        {degrees_to_radians(angle_min_deg), degrees_to_radians(angle_max_deg)},
        degrees_to_radians(scale_deg), servo_position(reference),
        servo.at("reference_position_meaning").get<std::string>()});

    const auto& mounting = root.at("mounting");
    // 第二部分：安装信息可以逐项缺省为 null。不能用“左右镜像”替代逐轴
    // 方向符号，也不能根据 pitch_position_increase_effect 自动补数学零位。
    const auto& coordinates = mounting.at("body_coordinates");
    if (coordinates.at("x") != "forward" || coordinates.at("y") != "left" ||
        coordinates.at("z") != "up") config_error("unsupported body coordinate convention");
    if (!mounting.at("body_origin").is_null()) {
      result.body_origin_ = mounting.at("body_origin").get<std::string>();
      if (result.body_origin_->empty()) config_error("body_origin description cannot be empty");
    }
    // 显式加载用户选定的数学约定。它描述零位几何，与逐轴舵机方向表分开；
    // 旧 schema 2 配置仍允许缺省此项，加载器不会凭空补出运动学定义。
    if (mounting.contains("kinematic_convention") && !mounting.at("kinematic_convention").is_null()) {
      const auto& convention = mounting.at("kinematic_convention");
      const auto& axes_at_zero = convention.at("positive_joint_axes_at_zero");
      require_size(axes_at_zero, joints_per_leg, "positive_joint_axes_at_zero");
      result.convention_ = kinematic_convention{
          read_unit_vector(convention.at("zero_leg_direction"), "zero_leg_direction"),
          {read_unit_vector(axes_at_zero.at(0), "body_yaw positive axis"),
           read_unit_vector(axes_at_zero.at(1), "upper_leg_pitch positive axis"),
           read_unit_vector(axes_at_zero.at(2), "lower_leg_pitch positive axis")}};
    }
    const auto& signs = mounting.at("joint_angle_signs");
    // 新配置明确命名为“关节参考角（度）”；兼容旧配置键，但禁止两个表相互矛盾。
    // 舵机固件零偏不是本字段的含义，加载后统一保存为 reference_joint_angle_rad。
    const bool has_reference_angles = mounting.contains("joint_reference_angles_deg");
    if (has_reference_angles && mounting.contains("joint_zero_offsets") &&
        mounting.at("joint_reference_angles_deg") != mounting.at("joint_zero_offsets")) {
      config_error("joint_reference_angles_deg conflicts with legacy joint_zero_offsets");
    }
    const std::string reference_key = has_reference_angles ? "joint_reference_angles_deg" : "joint_zero_offsets";
    const auto& references = mounting.at(reference_key);
    const auto& limits = mounting.at("installed_joint_limits");
    validate_joint_table(signs, "mounting.joint_angle_signs");
    validate_joint_table(references, "mounting.joint_reference_angles_deg");
    validate_joint_table(limits, "mounting.installed_joint_limits");
    const auto& roots = mounting.at("leg_root_positions_mm");
    const auto& axes = mounting.at("leg_root_axis_directions");
    const auto& transforms = mounting.at("fixed_body_to_first_axis_transforms");
    validate_leg_table(roots, "mounting.leg_root_positions_mm");
    validate_leg_table(axes, "mounting.leg_root_axis_directions");
    validate_leg_table(transforms, "mounting.fixed_body_to_first_axis_transforms");

    const auto& legs = root.at("legs");
    require_size(legs, leg_count, "legs");
    std::set<std::size_t> seen_legs;
    std::set<int> seen_ids;
    for (const auto& entry : legs) {
      const auto leg = read_leg(entry.at("body_role").get<std::string>());
      const auto index = leg_index(leg);
      if (!seen_legs.insert(index).second) config_error("duplicate leg body_role");
      const auto expected_side = index < 3 ? "right" : "left";
      if (entry.at("side") != expected_side) config_error("leg side and body_role disagree");
      const auto& ids = entry.at("servo_ids");
      require_size(ids, joints_per_leg, "leg.servo_ids");
      std::array<servo_id, joints_per_leg> leg_ids{
          servo_id(read_integer(ids.at(0), "servo_ids")),
          servo_id(read_integer(ids.at(1), "servo_ids")),
          servo_id(read_integer(ids.at(2), "servo_ids"))};
      for (std::size_t local = 0; local < joints_per_leg; ++local) {
        const auto id = leg_ids[local];
        // 实物已确认的 18 轴映射是约束。配置仍是取值来源，但不能静默改号。
        const int expected = static_cast<int>(index * joints_per_leg + local + 1);
        if (!seen_ids.insert(id.value()).second) config_error("duplicate physical servo ID");
        if (id.value() != expected) config_error("servo ID mapping disagrees with confirmed hardware");
        joint_calibration calibration;
        const auto& sign = optional_joint_entry(signs, id.value());
        if (!sign.is_null()) {
          const int value = read_integer(sign, "joint_angle_signs");
          if (value != -1 && value != 1) config_error("joint sign must be -1 or +1");
          calibration.servo_to_joint_sign = static_cast<rotation_sign>(value);
        }
        const auto& reference_joint_angle_deg = optional_joint_entry(references, id.value());
        // 关节参考角在 JSON 中使用度，在 C++ 中使用弧度；不改变其物理含义。
        if (!reference_joint_angle_deg.is_null()) {
          calibration.reference_joint_angle_rad = degrees_to_radians(
              read_number(reference_joint_angle_deg, reference_key));
        }
        const auto& installed = optional_joint_entry(limits, id.value());
        if (!installed.is_null()) {
          const double lower = read_number(installed.at("min_deg"), "installed_joint_limits.min_deg");
          const double upper = read_number(installed.at("max_deg"), "installed_joint_limits.max_deg");
          if (lower >= upper) config_error("installed joint limits must be increasing");
          calibration.installed_limits = angle_limits{degrees_to_radians(lower), degrees_to_radians(upper)};
        }
        result.joints_.push_back({{leg, joint_order[local], id}, calibration});
      }
      leg_model loaded_leg{leg, leg_ids, std::nullopt, std::nullopt, std::nullopt};
      const auto& root_position = optional_leg_entry(roots, leg);
      if (!root_position.is_null()) loaded_leg.root_position_m = read_vector(root_position, 0.001, "root_position_mm");
      const auto& axis = optional_leg_entry(axes, leg);
      if (!axis.is_null()) {
        loaded_leg.root_axis_direction = read_unit_vector(axis, "root_axis_direction");
      }
      const auto& transform = optional_leg_entry(transforms, leg);
      if (!transform.is_null()) {
        loaded_leg.body_to_first_axis = read_transform(transform, "body_to_first_axis");
        if (loaded_leg.root_position_m) {
          const auto& a = *loaded_leg.root_position_m;
          const auto& b = loaded_leg.body_to_first_axis->translation_m;
          if (std::hypot(a.x - b.x, a.y - b.y, a.z - b.z) > 1e-9) {
            config_error("root position disagrees with fixed transform translation");
          }
        }
      }
      result.legs_.push_back(std::move(loaded_leg));
    }
    // 接受 JSON 中腿条目的任意顺序，对外始终提供稳定的语义顺序。
    std::sort(result.legs_.begin(), result.legs_.end(), [](const auto& a, const auto& b) {
      return leg_index(a.leg) < leg_index(b.leg);
    });
    // 关节比较函数按物理 ID 升序排列，使下标始终对应 ID-1。
    std::sort(result.joints_.begin(), result.joints_.end(), [](const auto& a, const auto& b) {
      return a.binding.id.value() < b.binding.id.value();
    });

    const auto& geometry = root.at("model_geometry");
    // 第三部分：保留 L1、L2、装配间隙 d、第二段和 L3 的独立字段。
    // 不从 CAD 金属端长度自行推导胶垫触点，也不把 CAD 状态标为实测。
    if (geometry.at("units") != "mm") config_error("model_geometry.units must be mm");
    result.geometry_.servo_axis_to_far_end_l1_m = read_length(geometry, "servo_axis_to_far_end_L1");
    result.geometry_.first_leg_length_l2_m = read_length(geometry, "first_leg_length");
    result.geometry_.servo_far_end_gap_d_m = read_length(geometry, "servo_far_end_gap_d", true);
    result.geometry_.second_leg_length_m = read_length(geometry, "second_leg_length");
    result.geometry_.third_leg_length_to_metal_end_l3_m = read_length(geometry, "third_leg_length_to_metal_end");
    auto& contact = result.geometry_.actual_foot_contact_point_m;
    contact.source = geometry.at("source").get<std::string>();
    contact.status = geometry.at("measurement_status").get<std::string>();
    const auto& point = geometry.at("actual_foot_contact_point");
    if (!point.is_null()) {
      const auto frame = point.at("frame").get<std::string>();
      if (frame.empty()) config_error("foot contact point requires a named frame");
      contact.value = foot_contact_point{frame, read_vector(point.at("position_mm"), 0.001, "foot_contact_point.position_mm")};
      // 理想胶垫触点有独立的 CAD 简化来源，不能被全局腿长来源覆盖为实测值。
      // 旧配置没有逐项来源时，继续保留上方读取的全局来源与状态。
      if (point.contains("source")) contact.source = point.at("source").get<std::string>();
      if (point.contains("measurement_status")) contact.status = point.at("measurement_status").get<std::string>();
    }
    return result;
  } catch (const json::exception& error) {
    config_error(std::string("invalid JSON configuration: ") + error.what());
  } catch (const std::out_of_range& error) {
    config_error(std::string("invalid hardware value: ") + error.what());
  }
}

// 按腿枚举返回对应模型的只读引用；非法枚举或越界访问抛 out_of_range。
const leg_model& robot_model::leg(leg_id leg) const {
  return legs_.at(leg_index(leg));
}

// 按物理舵机 ID 查询关节，内部使用 ID-1 下标；返回模型内对象的只读引用。
const joint_model& robot_model::joint(servo_id id) const {
  return joints_.at(id.index());
}

// 按“哪条腿 + 哪个关节”查找绑定的舵机 ID，再返回对应关节模型。
// 不将逻辑关节下标直接当作物理 ID；非法枚举会抛 out_of_range。
const joint_model& robot_model::joint(leg_id leg, joint_type joint) const {
  return this->joint(this->leg(leg).servo_ids.at(joint_index(joint)));
}

// 判断指定轴是否已填写方向符号和关节参考角；两项齐全才允许位置值到关节角的映射。
// 此处不要求安装限位，也不表示已经验证整机运动安全。
bool robot_model::is_mapping_ready(servo_id id) const {
  const auto& calibration = joint(id).calibration;
  return calibration.servo_to_joint_sign.has_value() && calibration.reference_joint_angle_rad.has_value();
}

// 判断指定轴是否具备角度命令转换所需配置：映射可用且安装限位已填写。
// 仅判断配置完整性，不发送命令，也不代替对实际目标的限位检查。
bool robot_model::is_command_ready(servo_id id) const {
  return is_mapping_ready(id) && joint(id).calibration.installed_limits.has_value();
}

}  // namespace hexapod_core
