#include "hexapod_core/joint_mapper.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include "hexapod_core/leg_kinematics.hpp"

using namespace hexapod_core;
using json = nlohmann::json;

namespace {
// 测试断言：condition 为真则继续，为假时抛出带 message 的错误使测试失败。
void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

// 检查 actual 为有限值且与 expected 的绝对误差小于 1e-10；否则报告测试失败。
void require_near(double actual, double expected, const std::string& message) {
  require(std::isfinite(actual) && std::abs(actual - expected) < 1e-10, message);
}

// 分别检查三个分量，保留失败坐标，便于定位旋转方向或平移错误。
void require_point_near(const vector3& actual, const vector3& expected,
                        const std::string& message) {
  require_near(actual.x, expected.x, message + " X");
  require_near(actual.y, expected.y, message + " Y");
  require_near(actual.z, expected.z, message + " Z");
}

// 执行传入的可调用对象 function，确认其抛出指定的 exception_type。
// 未抛出时报告测试失败；其他类型的异常继续向外传播。
template <typename exception_type, typename function_type>
void require_throws(function_type function, const std::string& message) {
  try {
    function();
  } catch (const exception_type&) {
    return;
  }
  throw std::runtime_error("missing expected exception: " + message);
}

// 在传入 JSON 的副本中为 18 轴填写人工标定，返回修改后的测试配置。
// 这些方向、关节参考角和限位只用于测试，绝不代表用户机器的标定结果。
json synthetic_calibration(json root) {
  auto& mounting = root["mounting"];
  for (int id = 1; id <= 18; ++id) {
    const auto key = std::to_string(id);
    mounting["joint_angle_signs"][key] = id % 2 == 0 ? -1 : 1;
    mounting["joint_reference_angles_deg"][key] = 20.0;
    mounting["installed_joint_limits"][key] = {{"min_deg", -100.0}, {"max_deg", 100.0}};
  }
  return root;
}

// 验证基础类型的构造约束、枚举下标检查，以及默认目标/反馈保留未知状态。
void test_types() {
  static_assert(!std::is_default_constructible_v<servo_id>);
  static_assert(!std::is_default_constructible_v<servo_position>);
  static_assert(joint_index(leg_id::left_rear, joint_type::lower_leg_pitch) == 17);
  static_assert(servo_id(18).index() == 17);
  require_throws<std::out_of_range>([] { (void)servo_id(0); }, "zero ID");
  require_throws<std::out_of_range>([] { (void)servo_id(19); }, "ID above 18");
  require_throws<std::out_of_range>([] { (void)servo_position(-1); }, "negative position");
  require_throws<std::out_of_range>([] { (void)servo_position(1001); }, "position above native maximum");
  require_throws<std::out_of_range>([] { (void)leg_index(static_cast<leg_id>(-1)); }, "invalid leg enum");
  require_throws<std::out_of_range>([] { (void)joint_index(static_cast<joint_type>(99)); }, "invalid joint enum");
  const joint_targets targets;
  const joint_observation observation;
  for (std::size_t i = 0; i < joint_count; ++i) {
    require(!targets.desired_angles_rad[i] && !targets.commanded_positions[i], "unknown targets stay absent");
    require(!observation.read_positions[i] && !observation.read_angles_rad[i], "missing feedback stays absent");
  }
  require(!observation.request_started_at && !observation.response_completed_at, "unknown timestamps stay absent");
}

// 从 path 加载真实基准配置，对照 root 验证 18 轴绑定、单位换算和来源保留。
// 验证用户确认的 500 零位、140～860 限幅和 CAD 理想模型；不连接硬件。
void test_real_baseline(const json& root, const std::string& path) {
  const auto model = robot_model::from_file(path);
  require(model.legs().size() == 6 && model.joints().size() == 18, "full model loaded");
  const joint_mapper mapper(model);
  const std::array<vector3, leg_count> expected_roots{{
      {0.10217, -0.063, 0.021}, {-0.00004, -0.079, 0.021}, {-0.10223, -0.06294, 0.021},
      {0.10223, 0.06294, 0.021}, {0.00004, 0.079, 0.021}, {-0.10217, 0.063, 0.021}}};
  const std::array<double, leg_count> expected_yaws_deg{{45, 0, -45, 135, 180, -135}};
  for (std::size_t leg = 0; leg < leg_count; ++leg) {
    for (std::size_t local = 0; local < joints_per_leg; ++local) {
      const auto& joint = model.joint(leg_order[leg], joint_order[local]);
      const int expected = static_cast<int>(leg * joints_per_leg + local + 1);
      require(joint.binding.id.value() == expected, "confirmed 18-ID mapping");
      const auto id = joint.binding.id;
      const int sign = leg >= 3 && local != 0 ? -1 : 1;
      require(model.is_mapping_ready(id) && model.is_command_ready(id), "confirmed mapping and limits ready");
      require_near(mapper.servo_position_to_joint_angle_rad(id, servo_position(500)), 0.0, "user-confirmed zero pose");
      require_near(mapper.servo_position_to_joint_angle_rad(id, servo_position(550)),
                   degrees_to_radians(sign * 12.0), "axis sign agrees with selected convention");
      // 每个轴的所有已授权整数位置都应往返一致，包括两个限幅端点。
      for (int position = 140; position <= 860; ++position) {
        const double angle = mapper.servo_position_to_joint_angle_rad(id, servo_position(position));
        require(mapper.joint_angle_rad_to_servo_position(id, angle).value() == position, "installed range round-trip");
      }
      require_throws<std::out_of_range>([&] {
        (void)mapper.joint_angle_rad_to_servo_position(id, degrees_to_radians(sign * 86.41));
      }, "command above installed position range rejected");
      require_throws<std::out_of_range>([&] {
        (void)mapper.joint_angle_rad_to_servo_position(id, degrees_to_radians(-sign * 86.41));
      }, "command below installed position range rejected");
      // 协议范围仍为 0～1000，越过安装限位的反馈应能解码用于诊断。
      require_near(mapper.servo_position_to_joint_angle_rad(id, servo_position(0)),
                   degrees_to_radians(-sign * 120.0), "out-of-limit feedback remains observable");
    }
    const auto& loaded = model.leg(leg_order[leg]);
    require(loaded.root_position_m && loaded.root_axis_direction && loaded.body_to_first_axis, "CAD mounting loaded");
    const auto& position = *loaded.root_position_m;
    require_near(position.x, expected_roots[leg].x, "root X in metres");
    require_near(position.y, expected_roots[leg].y, "root Y in metres");
    require_near(position.z, expected_roots[leg].z, "root height from lower plate midplane");
    require_near(loaded.root_axis_direction->z, 1.0, "yaw positive axis points upward");
    const auto& transform = *loaded.body_to_first_axis;
    require_near(transform.translation_m.x, position.x, "fixed translation agrees with root");
    const auto& rotation = transform.rotation;
    const double yaw = degrees_to_radians(expected_yaws_deg[leg]);
    // 用零位腿方向 -Y 检查变换朝向，防止矩阵转置或重复计入安装角。
    require_near(-rotation[1], std::sin(yaw), "zero leg outward direction X");
    require_near(-rotation[4], -std::cos(yaw), "zero leg outward direction Y");
    require_near(rotation[8], 1.0, "local Z stays upward");
  }
  require(model.body_origin_description().has_value(), "body origin explicitly defined");
  require(model.convention().has_value(), "ideal mathematical convention loaded");
  require_near(model.convention()->zero_leg_direction.y, -1.0, "zero leg extends along local -Y");
  require_near(model.convention()->positive_joint_axes_at_zero[0].z, 1.0, "positive yaw axis");
  require_near(model.convention()->positive_joint_axes_at_zero[1].x, -1.0, "positive upper pitch axis");
  require_near(model.convention()->positive_joint_axes_at_zero[2].x, -1.0, "positive lower pitch axis");
  require_near(*model.geometry().first_leg_length_l2_m.value, 0.042, "L2 converted mm to m");
  require_near(*model.geometry().second_leg_length_m.value, 0.0742, "second length remains separate");
  require_near(*model.geometry().third_leg_length_to_metal_end_l3_m.value, 0.11253, "metal endpoint only");
  const auto& contact = model.geometry().actual_foot_contact_point_m;
  require(contact.value && contact.value->frame == "lower_leg", "contact has a lower-leg fixed frame");
  require_near(contact.value->position_m.x, 0.0, "ideal contact omits transverse offset");
  require_near(contact.value->position_m.y, -0.11253, "CAD metal-end ideal contact in metres");
  require_near(contact.value->position_m.z, 0.0, "ideal contact lies on lower-leg centreline");
  require(contact.status == root["model_geometry"]["actual_foot_contact_point"]["measurement_status"].get<std::string>(),
          "ideal contact provenance preserved separately");
  require(model.geometry().second_leg_length_m.status == root["model_geometry"]["measurement_status"].get<std::string>(),
          "CAD confirmation status preserved verbatim");
  require_near(mapper.servo_position_to_servo_relative_angle_rad(servo_position(500)), 0.0, "native delta reference");
  require_near(mapper.servo_position_to_servo_relative_angle_rad(servo_position(550)), degrees_to_radians(12.0), "native delta scale");
  auto reordered = root;
  std::reverse(reordered["legs"].begin(), reordered["legs"].end());
  const auto shuffled_model = robot_model::from_json_text(reordered.dump());
  require(shuffled_model.joint(leg_id::left_front, joint_type::upper_leg_pitch).binding.id.value() == 11,
          "JSON row order does not change physical mapping");
  require_throws<std::runtime_error>([] { (void)robot_model::from_file("missing-hexapod-test-config.json"); }, "missing file");
}

// 基于 root 的人工标定副本，验证正负方向、整数位置往返和取整误差。
// 还覆盖安装限位、原生范围、非有限角，以及取整后跨越限位边界的拒绝逻辑。
void test_mapping(const json& root) {
  const auto synthetic = synthetic_calibration(root);
  const auto model = robot_model::from_json_text(synthetic.dump());
  const joint_mapper mapper(model);
  for (int raw_id = 1; raw_id <= 18; ++raw_id) {
    const servo_id id(raw_id);
    require(model.is_mapping_ready(id) && model.is_command_ready(id), "synthetic calibration ready");
    require_near(mapper.servo_position_to_joint_angle_rad(id, servo_position(500)), degrees_to_radians(20.0),
                 "reference position has nonzero mathematical angle");
    for (int position = 0; position <= 1000; ++position) {
      const double angle = mapper.servo_position_to_joint_angle_rad(id, servo_position(position));
      const double direction = raw_id % 2 == 0 ? -1.0 : 1.0;
      require_near(angle, degrees_to_radians(20.0 + direction * (position - 500) * 0.24), "both mapping directions");
      if (angle >= degrees_to_radians(-100.0) && angle <= degrees_to_radians(100.0)) {
        require(mapper.joint_angle_rad_to_servo_position(id, angle).value() == position, "all reachable integer positions round-trip");
      }
    }
    // 非栅格角度应量化到最近位置，误差不大于半个位置单位对应的角度。
    const double desired = degrees_to_radians(37.123);
    const auto rounded = mapper.joint_angle_rad_to_servo_position(id, desired);
    require(std::abs(mapper.servo_position_to_joint_angle_rad(id, rounded) - desired) <=
                model.servo().radians_per_position_unit * 0.5 + 1e-12,
            "non-integer target has bounded quantization error");
  }
  require_near(mapper.servo_position_to_joint_angle_rad(servo_id(1), servo_position(1000)), degrees_to_radians(140.0),
               "out-of-installed-limit feedback can still be decoded");
  require_throws<std::out_of_range>([&] { (void)mapper.joint_angle_rad_to_servo_position(servo_id(1), degrees_to_radians(140.0)); },
                                  "same out-of-limit angle cannot be commanded");
  require_throws<std::invalid_argument>([&] { (void)mapper.joint_angle_rad_to_servo_position(servo_id(1), std::numeric_limits<double>::quiet_NaN()); }, "NaN angle");
  require_throws<std::invalid_argument>([&] { (void)mapper.joint_angle_rad_to_servo_position(servo_id(1), std::numeric_limits<double>::infinity()); }, "infinite angle");
  auto rounding_case = synthetic;
  rounding_case["mounting"]["joint_reference_angles_deg"]["1"] = 0.0;
  rounding_case["mounting"]["installed_joint_limits"]["1"] = {{"min_deg", 0.05}, {"max_deg", 1.0}};
  const joint_mapper boundary_mapper(robot_model::from_json_text(rounding_case.dump()));
  require_throws<std::out_of_range>([&] { (void)boundary_mapper.joint_angle_rad_to_servo_position(servo_id(1), degrees_to_radians(0.05)); },
                                  "integer quantization cannot cross installed boundary");
  auto wide_limits = synthetic;
  wide_limits["mounting"]["installed_joint_limits"]["1"] = {{"min_deg", -200.0}, {"max_deg", 200.0}};
  const joint_mapper native_mapper(robot_model::from_json_text(wide_limits.dump()));
  require_throws<std::out_of_range>([&] { (void)native_mapper.joint_angle_rad_to_servo_position(servo_id(1), degrees_to_radians(180.0)); },
                                  "native range still enforced with wider installed range");
  auto reduced_range = synthetic;
  reduced_range["servo"]["position_range"] = {100, 900};
  reduced_range["servo"]["angle_range_deg"] = {24, 216};
  const joint_mapper reduced_mapper(robot_model::from_json_text(reduced_range.dump()));
  require_throws<std::out_of_range>([&] { (void)reduced_mapper.servo_position_to_joint_angle_rad(servo_id(1), servo_position(99)); },
                                  "feedback respects configured native subrange");
}

// 只填写部分轴的部分标定，验证反馈映射与命令转换对配置完整性的要求不同。
// 确认一条轴已标定不会使其他轴自动获得方向、关节参考角或安装限位。
void test_partial_calibration(const json& root) {
  auto partial = root;
  partial["mounting"]["joint_angle_signs"] = {{"1", 1}};
  partial["mounting"]["joint_reference_angles_deg"] = {{"1", 15.0}, {"2", nullptr}};
  partial["mounting"]["installed_joint_limits"] = nullptr;
  const auto model = robot_model::from_json_text(partial.dump());
  require(model.is_mapping_ready(servo_id(1)) && !model.is_command_ready(servo_id(1)), "mapping and command readiness differ");
  require(!model.is_mapping_ready(servo_id(2)), "partial tables do not imply sibling calibration");
  const joint_mapper mapper(model);
  require_near(mapper.servo_position_to_joint_angle_rad(servo_id(1), servo_position(500)), degrees_to_radians(15.0), "partial feedback mapping");
  require_throws<std::logic_error>([&] { (void)mapper.joint_angle_rad_to_servo_position(servo_id(1), degrees_to_radians(15.0)); }, "missing installed limits");
  require_throws<std::logic_error>([&] { (void)mapper.servo_position_to_joint_angle_rad(servo_id(2), servo_position(500)); }, "missing sibling mapping");
}

// 真实配置已完整，但加载器仍须支持旧配置和未知项；未知绝不能自动变成零位。
void test_unknown_configuration(const json& root) {
  // 字段改名不应丢失旧配置中的非零关节参考角；新旧表矛盾在异常测试中拒绝。
  auto legacy = root;
  legacy["mounting"]["joint_zero_offsets"] = legacy["mounting"]["joint_reference_angles_deg"];
  legacy["mounting"]["joint_zero_offsets"]["1"] = 20.0;
  legacy["mounting"].erase("joint_reference_angles_deg");
  const joint_mapper legacy_mapper(robot_model::from_json_text(legacy.dump()));
  require_near(legacy_mapper.servo_position_to_joint_angle_rad(servo_id(1), servo_position(500)),
               degrees_to_radians(20.0), "legacy reference field retains its meaning");
  auto unknown = root;
  for (const auto* key : {"joint_angle_signs", "joint_reference_angles_deg", "installed_joint_limits",
                          "body_origin", "leg_root_positions_mm", "leg_root_axis_directions",
                          "fixed_body_to_first_axis_transforms"}) {
    unknown["mounting"][key] = nullptr;
  }
  unknown["mounting"].erase("kinematic_convention");
  unknown["model_geometry"]["actual_foot_contact_point"] = nullptr;
  const auto model = robot_model::from_json_text(unknown.dump());
  require(!model.convention() && !model.body_origin_description(), "legacy unknown definitions stay absent");
  for (const auto& joint : model.joints()) {
    require(!model.is_mapping_ready(joint.binding.id) && !model.is_command_ready(joint.binding.id),
            "unknown calibration does not become zero");
  }
  require(!model.leg(leg_id::right_middle).body_to_first_axis, "unknown transform stays absent");
  require(!model.geometry().actual_foot_contact_point_m.value, "unknown contact stays absent");
  const joint_mapper mapper(model);
  require_throws<std::logic_error>([&] {
    (void)mapper.servo_position_to_joint_angle_rad(servo_id(1), servo_position(500));
  }, "unknown mathematical reference rejected");
}

// 用人工配置验证可选安装坐标、变换和触点的加载，不修改真实基准文件。
// 检查毫米换米、缺项保留未知，并拒绝镜像矩阵及相互矛盾的安装平移。
void test_optional_geometry(const json& root) {
  auto fixture = root;
  auto& mounting = fixture["mounting"];
  mounting["body_origin"] = "synthetic fixture origin";
  mounting["leg_root_positions_mm"] = {{"right_front", {10.0, -20.0, 30.0}}};
  mounting["leg_root_axis_directions"] = {{"right_front", {0.0, 0.0, 1.0}}};
  mounting["fixed_body_to_first_axis_transforms"] = {{"right_front", {
      {"translation_mm", {10.0, -20.0, 30.0}},
      {"rotation", {{0.0, -1.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}}}}}};
  fixture["model_geometry"]["actual_foot_contact_point"] = {
      {"frame", "synthetic_third_axis"}, {"position_mm", {5.0, 0.0, -120.0}}};
  const auto model = robot_model::from_json_text(fixture.dump());
  require_near(model.leg(leg_id::right_front).root_position_m->y, -0.02, "root coordinates converted separately");
  require(model.leg(leg_id::right_front).body_to_first_axis.has_value(), "explicit transform loaded");
  require(!model.leg(leg_id::left_front).body_to_first_axis, "no mirrored default transform");
  require_near(model.geometry().actual_foot_contact_point_m.value->position_m.z, -0.12, "named contact frame loaded");
  auto invalid = fixture;
  invalid["mounting"]["fixed_body_to_first_axis_transforms"]["right_front"]["rotation"] = {
      {-1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  require_throws<std::runtime_error>([&] { (void)robot_model::from_json_text(invalid.dump()); }, "reflection is not a rotation");
  invalid = fixture;
  invalid["mounting"]["leg_root_positions_mm"]["right_front"] = {11, -20, 30};
  require_throws<std::runtime_error>([&] { (void)robot_model::from_json_text(invalid.dump()); }, "conflicting mounting translations");
}

// 逐项破坏 root 的副本，确认版本、类型、编号、方向、限位等错误均被拒绝。
// 同时验证 JSON 语法错误和重复键会使模型加载失败。
void test_invalid_configuration(const json& root) {
  // 接收一个修改操作，每次使用独立副本制造错误，再确认加载抛 runtime_error。
  const auto reject_change = [&root](const auto& change, const std::string& message) {
    auto invalid = root;
    change(invalid);
    require_throws<std::runtime_error>([&] { (void)robot_model::from_json_text(invalid.dump()); }, message);
  };
  reject_change([](json& j) { j["schema_version"] = 3; }, "unsupported schema");
  reject_change([](json& j) { j["units"]["length"] = "m"; }, "unit mismatch");
  reject_change([](json& j) { j["servo"]["count"] = 17; }, "missing servo count");
  reject_change([](json& j) { j["legs"][0]["servo_ids"][0] = 0; }, "invalid physical ID");
  reject_change([](json& j) { j["legs"][0]["servo_ids"][0] = true; }, "boolean ID");
  reject_change([](json& j) { j["legs"][0]["servo_ids"][0] = 1.5; }, "fractional ID");
  reject_change([](json& j) { j["legs"][0]["servo_ids"][0] = 18446744073709551615ULL; }, "overflow ID");
  reject_change([](json& j) { j["legs"][1]["body_role"] = "right_front"; }, "duplicate leg");
  reject_change([](json& j) { j["legs"][0]["side"] = "left"; }, "side conflict");
  reject_change([](json& j) { std::swap(j["legs"][0]["servo_ids"][0], j["legs"][0]["servo_ids"][1]); }, "remapped physical ID");
  reject_change([](json& j) { j["mounting"]["joint_angle_signs"] = {{"1", 0}}; }, "zero direction");
  reject_change([](json& j) { j["mounting"]["joint_angle_signs"] = {{"1", true}}; }, "boolean direction");
  reject_change([](json& j) { j["mounting"]["joint_angle_signs"] = {{"19", 1}}; }, "unknown table ID");
  reject_change([](json& j) { j["mounting"]["joint_reference_angles_deg"] = {{"1", "0"}}; }, "string angle");
  reject_change([](json& j) { j["mounting"]["joint_zero_offsets"] = {{"1", 20.0}}; }, "conflicting legacy reference table");
  reject_change([](json& j) { j["mounting"]["installed_joint_limits"] = {{"1", {{"min_deg", 10}, {"max_deg", -10}}}}; }, "reversed limits");
  reject_change([](json& j) { j["mounting"]["leg_root_axis_directions"] = {{"right_front", {0, 0, 0}}}; }, "zero axis");
  reject_change([](json& j) { j["mounting"]["kinematic_convention"]["zero_leg_direction"] = {0, 0, 0}; }, "zero leg direction");
  reject_change([](json& j) { j["mounting"]["kinematic_convention"]["positive_joint_axes_at_zero"][1] = {-2, 0, 0}; }, "scaled pitch axis");
  reject_change([](json& j) { j["servo"]["degrees_per_position_unit"] = 0; }, "zero angle scale");
  reject_change([](json& j) { j["servo"]["angle_range_deg"] = {0, 270}; }, "contradictory native scale");
  reject_change([](json& j) { j["model_geometry"]["second_leg_length"] = -1; }, "negative leg length");
  reject_change([](json& j) { j.erase("mounting"); }, "missing required section");
  require_throws<std::runtime_error>([] { (void)robot_model::from_json_text("{broken"); }, "malformed JSON");
  const auto original = root.dump();
  const auto duplicate = std::string("{\"schema_version\":2,") + original.substr(1);
  require_throws<std::runtime_error>([&] { (void)robot_model::from_json_text(duplicate); }, "duplicate JSON key");
}

void test_forward_kinematics(const std::string& path) {
  const leg_kinematics kinematics(robot_model::from_file(path));

  // 当前 CAD 理想模型的独立预期值，单位为米。
  constexpr double total_length_m = 0.22873;
  constexpr double second_length_m = 0.0742;
  constexpr double foot_length_m = 0.11253;

  // 1. 三个关节均为零：整条腿沿局部 -Y 水平展开。
  const auto zero =
      kinematics.forward_in_leg_frame({0.0, 0.0, 0.0});

  require_near(zero.x, 0.0, "FK zero X");
  require_near(zero.y, -total_length_m, "FK zero Y");
  require_near(zero.z, 0.0, "FK zero Z");

  // 2. 仅偏航 +30°：高度不变，脚端转向局部 +X。
  const auto yaw = kinematics.forward_in_leg_frame({
      degrees_to_radians(30.0),
      0.0,
      0.0
  });

  require_near(yaw.x, total_length_m / 2.0, "FK positive yaw X");
  require_near(
      yaw.y,
      -total_length_m * std::sqrt(3.0) / 2.0,
      "FK positive yaw Y");
  require_near(yaw.z, 0.0, "FK yaw preserves height");

  // 3. 大腿 +30°、膝关节 -30°：小腿恢复水平。
  const auto bent = kinematics.forward_in_leg_frame({
      0.0,
      degrees_to_radians(30.0),
      degrees_to_radians(-30.0)
  });

  const double expected_radial_m =
      0.042
      + second_length_m * std::sqrt(3.0) / 2.0
      + foot_length_m;

  require_near(bent.x, 0.0, "FK bent X");
  require_near(bent.y, -expected_radial_m, "FK bent Y");
  require_near(bent.z, second_length_m / 2.0, "FK relative knee");
}

// 采用配置副本检验 FK 的失败分支和配置驱动行为，不修改真实硬件参数。
void test_kinematics_validation(const json& root) {
  const leg_kinematics kinematics(robot_model::from_json_text(root.dump()));
  const auto downward = kinematics.forward_in_leg_frame({
      0.0, degrees_to_radians(-30.0), 0.0});
  require_near(downward.z, -(0.0742 + 0.11253) / 2.0,
               "negative upper pitch lowers both segments");

  for (std::size_t axis = 0; axis < joints_per_leg; ++axis) {
    for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity()}) {
      std::array<double, joints_per_leg> angles{0.0, 0.0, 0.0};
      angles[axis] = invalid;
      require_throws<std::invalid_argument>([&] {
        (void)kinematics.forward_in_leg_frame({angles[0], angles[1], angles[2]});
      }, "FK rejects non-finite joint input");
    }
  }
  require_throws<std::invalid_argument>([&] {
    const double largest = std::numeric_limits<double>::max();
    (void)kinematics.forward_in_leg_frame({0.0, largest, largest});
  }, "FK rejects overflowing combined pitch");

  const auto reject_model = [&root](const auto& change, const std::string& message) {
    auto fixture = root;
    change(fixture);
    require_throws<std::logic_error>([&] {
      const leg_kinematics rejected(robot_model::from_json_text(fixture.dump()));
      (void)rejected;
    }, message);
  };
  for (const auto* key : {"first_leg_length", "second_leg_length", "actual_foot_contact_point"}) {
    reject_model([key](json& j) { j["model_geometry"][key] = nullptr; },
                 "FK rejects missing geometry");
  }
  reject_model([](json& j) { j["mounting"].erase("kinematic_convention"); },
               "FK rejects unknown convention");
  reject_model([](json& j) {
    j["mounting"]["kinematic_convention"]["zero_leg_direction"] = {0, 1, 0};
  }, "FK rejects unsupported zero direction");
  for (std::size_t axis = 0; axis < joints_per_leg; ++axis) {
    reject_model([axis](json& j) {
      j["mounting"]["kinematic_convention"]["positive_joint_axes_at_zero"][axis] = {0, 1, 0};
    }, "FK rejects unsupported joint axis");
  }
  reject_model([](json& j) {
    j["model_geometry"]["actual_foot_contact_point"]["frame"] = "body";
  }, "FK rejects contact expressed in another frame");
  for (const std::size_t axis : {std::size_t{0}, std::size_t{2}}) {
    reject_model([axis](json& j) {
      j["model_geometry"]["actual_foot_contact_point"]["position_mm"][axis] = 1.0;
    }, "FK rejects non-axial contact");
  }
  for (const double y : {0.0, 112.53}) {
    reject_model([y](json& j) {
      j["model_geometry"]["actual_foot_contact_point"]["position_mm"][1] = y;
    }, "FK requires contact on negative local Y");
  }

  // 证明脚端长度来自触点配置，并未在算法中硬编码或重复叠加金属 L3。
  auto shorter_contact = root;
  shorter_contact["model_geometry"]["actual_foot_contact_point"]["position_mm"] = {0, -100, 0};
  const leg_kinematics shorter(robot_model::from_json_text(shorter_contact.dump()));
  const auto shorter_zero = shorter.forward_in_leg_frame({0.0, 0.0, 0.0});
  require_near(shorter_zero.y, -0.2162, "FK uses configured contact vector length");
  require_near(shorter_zero.z, 0.0, "first segment contributes no zero-pose height");
}

// 使用独立的 CAD 安装位置和零位预期值，避免只做往返而漏掉双向同错。
void test_frame_transforms(const json& root) {
  const leg_kinematics kinematics(robot_model::from_json_text(root.dump()));
  const std::array<vector3, leg_count> expected_roots{{
      {0.10217, -0.063, 0.021}, {-0.00004, -0.079, 0.021},
      {-0.10223, -0.06294, 0.021}, {0.10223, 0.06294, 0.021},
      {0.00004, 0.079, 0.021}, {-0.10217, 0.063, 0.021}}};
  const double diagonal_m = 0.22873 / std::sqrt(2.0);
  const std::array<vector3, leg_count> expected_zero{{
      {0.10217 + diagonal_m, -0.063 - diagonal_m, 0.021},
      {-0.00004, -0.30773, 0.021},
      {-0.10223 - diagonal_m, -0.06294 - diagonal_m, 0.021},
      {0.10223 + diagonal_m, 0.06294 + diagonal_m, 0.021},
      {0.00004, 0.30773, 0.021},
      {-0.10217 - diagonal_m, 0.063 + diagonal_m, 0.021}}};
  const std::array<vector3, 3> local_points{{
      {0.013, -0.16, -0.075}, {-0.027, 0.084, 0.031}, {0.0, 0.0, 0.0}}};
  const std::array<vector3, 2> body_points{{
      {0.22, -0.18, -0.1}, {-0.11, 0.24, 0.07}}};

  for (std::size_t index = 0; index < leg_count; ++index) {
    const auto leg = leg_order[index];
    const auto label = std::string("leg ") + std::to_string(index);
    require_point_near(kinematics.leg_to_body_frame(leg, {0.0, 0.0, 0.0}),
                       expected_roots[index], label + " local origin");
    require_point_near(kinematics.body_to_leg_frame(leg, expected_roots[index]),
                       {0.0, 0.0, 0.0}, label + " body root maps to local origin");
    require_point_near(kinematics.leg_to_body_frame(leg, {0.0, -0.22873, 0.0}),
                       expected_zero[index], label + " independent zero direction");
    require_point_near(kinematics.body_to_leg_frame(leg, expected_zero[index]),
                       {0.0, -0.22873, 0.0}, label + " independent inverse zero");
    require_point_near(kinematics.forward_in_body_frame(leg, {0.0, 0.0, 0.0}),
                       expected_zero[index], label + " body FK zero");

    for (const auto& point : local_points) {
      const auto body = kinematics.leg_to_body_frame(leg, point);
      require_point_near(kinematics.body_to_leg_frame(leg, body), point,
                         label + " local-body-local round-trip");
    }
    for (const auto& point : body_points) {
      const auto local = kinematics.body_to_leg_frame(leg, point);
      require_point_near(kinematics.leg_to_body_frame(leg, local), point,
                         label + " body-local-body round-trip");
    }
  }

  // 非零 FK：右前安装 45°、偏航 30°，合计 75°；膝 -30°抵消大腿 +30°。
  const double radial_m = 0.042 + 0.0742 * std::sqrt(3.0) / 2.0 + 0.11253;
  const double sin_75 = (std::sqrt(6.0) + std::sqrt(2.0)) / 4.0;
  const double cos_75 = (std::sqrt(6.0) - std::sqrt(2.0)) / 4.0;
  require_point_near(kinematics.forward_in_body_frame(leg_id::right_front,
      {degrees_to_radians(30.0), degrees_to_radians(30.0), degrees_to_radians(-30.0)}),
      {0.10217 + radial_m * sin_75, -0.063 - radial_m * cos_75, 0.0581},
      "nonzero body FK combines installation, yaw and relative knee once");
}

// 只在配置副本中设置三维安装旋转，检查 Z 分量和所有矩阵行/列的使用。
// 合成旋转把局部 (x,y,z) 映射为 (z,x,y)，不是实物安装参数。
void test_spatial_frame_transform(const json& root) {
  auto fixture = root;
  fixture["mounting"]["leg_root_positions_mm"]["right_front"] = {10, -20, 30};
  fixture["mounting"]["leg_root_axis_directions"]["right_front"] = {1, 0, 0};
  fixture["mounting"]["fixed_body_to_first_axis_transforms"]["right_front"] = {
      {"translation_mm", {10, -20, 30}},
      {"rotation", {{0, 0, 1}, {1, 0, 0}, {0, 1, 0}}}};
  const leg_kinematics kinematics(robot_model::from_json_text(fixture.dump()));
  require_point_near(kinematics.leg_to_body_frame(leg_id::right_front,
      {0.013, -0.16, -0.075}), {-0.065, -0.007, -0.13},
      "3D rotation plus translation, independent forward result");
  require_point_near(kinematics.body_to_leg_frame(leg_id::right_front,
      {0.12, 0.041, -0.087}), {0.061, -0.117, 0.11},
      "subtract translation before transposed 3D rotation");
  require_point_near(kinematics.forward_in_body_frame(leg_id::right_front,
      {0.0, 0.0, 0.0}), {0.01, -0.02, -0.19873},
      "body FK uses configured 3D transform without hardcoded yaw");
}

// 缺参、非法枚举和非有限输入都必须明确失败，不用零值或默认变换继续。
void test_frame_transform_validation(const json& root) {
  const leg_kinematics kinematics(robot_model::from_json_text(root.dump()));
  const vector3 valid_point{0.013, -0.16, -0.075};
  const std::array<const char*, leg_count> names{{
      "right_front", "right_middle", "right_rear",
      "left_front", "left_middle", "left_rear"}};
  for (std::size_t index = 0; index < leg_count; ++index) {
    const auto leg = leg_order[index];
    auto missing = root;
    missing["mounting"]["fixed_body_to_first_axis_transforms"][names[index]] = nullptr;
    const leg_kinematics incomplete(robot_model::from_json_text(missing.dump()));
    require_throws<std::logic_error>([&] {
      (void)incomplete.leg_to_body_frame(leg, valid_point);
    }, "missing leg mounting rejects forward transform");
    require_throws<std::logic_error>([&] {
      (void)incomplete.body_to_leg_frame(leg, valid_point);
    }, "missing leg mounting rejects inverse transform");
    require_throws<std::logic_error>([&] {
      (void)incomplete.forward_in_body_frame(leg, {0.0, 0.0, 0.0});
    }, "body FK cannot bypass missing mounting");
    const auto other_leg = leg_order[(index + 1) % leg_count];
    require_point_near(incomplete.forward_in_body_frame(other_leg, {0.0, 0.0, 0.0}),
        kinematics.forward_in_body_frame(other_leg, {0.0, 0.0, 0.0}),
        "missing one mounting does not disable another leg");

    for (std::size_t axis = 0; axis < 3; ++axis) {
      for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
                                   std::numeric_limits<double>::infinity(),
                                   -std::numeric_limits<double>::infinity()}) {
        std::array<double, 3> coordinates{0.01, -0.1, 0.03};
        coordinates[axis] = invalid;
        const vector3 point{coordinates[0], coordinates[1], coordinates[2]};
        require_throws<std::invalid_argument>([&] {
          (void)kinematics.leg_to_body_frame(leg, point);
        }, "forward transform rejects each non-finite coordinate");
        require_throws<std::invalid_argument>([&] {
          (void)kinematics.body_to_leg_frame(leg, point);
        }, "inverse transform rejects each non-finite coordinate");
        require_throws<std::invalid_argument>([&] {
          (void)kinematics.forward_in_body_frame(leg,
              {coordinates[0], coordinates[1], coordinates[2]});
        }, "body FK preserves local FK non-finite-angle checks");
      }
    }
  }
  for (const auto invalid_leg : {static_cast<leg_id>(-1), static_cast<leg_id>(leg_count)}) {
    require_throws<std::out_of_range>([&] {
      (void)kinematics.leg_to_body_frame(invalid_leg, valid_point);
    }, "forward transform rejects invalid leg");
    require_throws<std::out_of_range>([&] {
      (void)kinematics.body_to_leg_frame(invalid_leg, valid_point);
    }, "inverse transform rejects invalid leg");
    require_throws<std::out_of_range>([&] {
      (void)kinematics.forward_in_body_frame(invalid_leg, {0.0, 0.0, 0.0});
    }, "body FK rejects invalid leg");
  }
}

}  // namespace

// 离线测试入口：argv[1] 是基准 JSON 路径；依次执行所有测试并打印结果。
// 全部通过返回 0；参数、读文件或任一测试失败时打印异常并返回 1，不驱动硬件。
int main(int argc, char** argv) {
  try {
    require(argc == 2, "usage: core_tests robot_hardware.json");
    std::ifstream input(argv[1]);
    require(static_cast<bool>(input), "cannot read baseline fixture");
    const auto root = json::parse(input);
    test_types();
    test_real_baseline(root, argv[1]);
    test_forward_kinematics(argv[1]);
    test_kinematics_validation(root);
    test_frame_transforms(root);
    test_spatial_frame_transform(root);
    test_frame_transform_validation(root);
    test_mapping(root);
    test_partial_calibration(root);
    test_unknown_configuration(root);
    test_optional_geometry(root);
    test_invalid_configuration(root);
    std::cout << "PASS: types, real 18-ID baseline, unknown calibration, round-trips, "
                 "partial calibration, quantization limits, geometry, forward kinematics, "
                 "body/leg frame transforms "
                 "and invalid configuration.\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
