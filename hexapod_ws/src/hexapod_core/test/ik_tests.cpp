#include "hexapod_core/leg_kinematics.hpp"

#include <nlohmann/json.hpp>

#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using namespace hexapod_core;
using json = nlohmann::json;

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

template <typename exception_type, typename function_type>
void require_throws(function_type function, const std::string& message) {
  try {
    function();
  } catch (const exception_type&) {
    return;
  }
  throw std::runtime_error("missing expected exception: " + message);
}

// 周期角比较也把 +pi 与 -pi 视为同一姿态；不依赖候选的返回顺序。
bool same_angles(const leg_joint_angles& actual, const leg_joint_angles& expected,
                 double tolerance_rad = 1e-10) {
  const double full_turn_rad = 2.0 * std::acos(-1.0);
  const auto same_angle = [full_turn_rad, tolerance_rad](double a, double b) {
    return std::isfinite(a) && std::isfinite(b)
        && std::abs(std::remainder(a - b, full_turn_rad)) <= tolerance_rad;
  };
  return same_angle(actual.yaw_rad, expected.yaw_rad)
      && same_angle(actual.upper_leg_rad, expected.upper_leg_rad)
      && same_angle(actual.lower_leg_rad, expected.lower_leg_rad);
}

bool contains_angles(const leg_ik_result& result, const leg_joint_angles& expected,
                     double tolerance_rad = 1e-10) {
  for (const auto& candidate : result.solution) {
    if (same_angles(candidate.angles, expected, tolerance_rad)) return true;
  }
  return false;
}

// 复用公开 FK 检查每套候选，并在周期角意义下检查两两不重复。
void require_valid_candidates(const leg_kinematics& kinematics, const vector3& point,
                              const leg_ik_result& result, const std::string& label) {
  require(result.status == leg_ik_status::success, label + " success status");
  require(!result.solution.empty() && result.solution.size() <= 4,
          label + " one to four candidates");
  for (std::size_t index = 0; index < result.solution.size(); ++index) {
    const auto& angles = result.solution[index].angles;
    require(std::isfinite(angles.yaw_rad) && std::isfinite(angles.upper_leg_rad)
                && std::isfinite(angles.lower_leg_rad),
            label + " finite candidate angles");
    const auto recovered = kinematics.forward_in_leg_frame(angles);
    const double residual_m = std::hypot(recovered.x - point.x,
        recovered.y - point.y, recovered.z - point.z);
    require(std::isfinite(residual_m) && residual_m < 1e-10,
            label + " FK residual below 1e-10 m");
    for (std::size_t previous = 0; previous < index; ++previous) {
      require(!same_angles(angles, result.solution[previous].angles),
              label + " no duplicate periodic angles");
    }
  }
}

void require_empty_result(const leg_kinematics& kinematics, const vector3& point,
                          leg_ik_status expected, const std::string& label) {
  const auto result = kinematics.inverse_candidates_in_leg_frame(point);
  require(result.status == expected, label + " status");
  require(result.solution.empty(), label + " no discrete candidates");
}

// 独立的 CAD 基准预期：42 + 74.2 + 112.53 mm，全零只对应一套伸直解。
void test_independent_zero_pose(const leg_kinematics& kinematics) {
  const vector3 point{0.0, -0.22873, 0.0};
  const auto result = kinematics.inverse_candidates_in_leg_frame(point);
  require_valid_candidates(kinematics, point, result, "independent zero pose");
  require(result.solution.size() == 1, "independent zero pose has one candidate");
  require(same_angles(result.solution.front().angles, {0.0, 0.0, 0.0}),
          "independent zero pose recovers all zero joint angles");
  require(result.solution.front().knee_near_singular,
          "independent zero pose keeps singular knee flag");
}

// 40 个伸直姿态与 240 个普通姿态；包括用户限位端点，不驱动硬件。
void test_pose_round_trips(const leg_kinematics& kinematics) {
  std::size_t straight_count = 0;
  std::size_t ordinary_count = 0;
  for (const double yaw_deg : {-86.4, -30.0, 0.0, 17.0, 86.4}) {
    for (const double upper_deg : {-86.4, -60.0, -30.0, 0.0, 12.0, 30.0, 60.0, 86.4}) {
      const leg_joint_angles straight{degrees_to_radians(yaw_deg),
          degrees_to_radians(upper_deg), 0.0};
      const auto point = kinematics.forward_in_leg_frame(straight);
      const auto result = kinematics.inverse_candidates_in_leg_frame(point);
      const auto label = "straight pose " + std::to_string(straight_count);
      require_valid_candidates(kinematics, point, result, label);
      require(result.solution.size() == 1, label + " merged boundary branches");
      require(contains_angles(result, straight), label + " original pose recovered");
      require(result.solution.front().knee_near_singular, label + " knee flag");
      ++straight_count;

      for (const double lower_deg : {-86.4, -72.0, -30.0, 30.0, 72.0, 86.4}) {
        const leg_joint_angles ordinary{straight.yaw_rad, straight.upper_leg_rad,
            degrees_to_radians(lower_deg)};
        const auto bent_point = kinematics.forward_in_leg_frame(ordinary);
        const auto bent_result = kinematics.inverse_candidates_in_leg_frame(bent_point);
        const auto bent_label = "ordinary pose " + std::to_string(ordinary_count);
        require_valid_candidates(kinematics, bent_point, bent_result, bent_label);
        require(contains_angles(bent_result, ordinary, 1e-8),
                bent_label + " original pose recovered");
        ++ordinary_count;
      }
    }
  }
  require(straight_count == 40 && ordinary_count == 240, "pose coverage counts");
}

// 可分辨的近伸直仍保留正负膝角；标志只报告接近奇异，不把角度吸附为零。
void test_near_straight_branches(const leg_kinematics& kinematics) {
  const leg_joint_angles original{0.3, 0.2, 5e-7};
  const auto point = kinematics.forward_in_leg_frame(original);
  const auto result = kinematics.inverse_candidates_in_leg_frame(point);
  require_valid_candidates(kinematics, point, result, "near straight");
  require(result.solution.size() == 2, "near straight retains two branches");
  require(contains_angles(result, original, 1e-8), "near straight original pose recovered");
  bool positive_knee = false;
  bool negative_knee = false;
  for (const auto& candidate : result.solution) {
    require(candidate.knee_near_singular, "near straight flags both knees");
    require(std::abs(candidate.angles.lower_leg_rad) > 1e-7,
            "near straight knee is not snapped to zero");
    positive_knee = positive_knee || candidate.angles.lower_leg_rad > 0.0;
    negative_knee = negative_knee || candidate.angles.lower_leg_rad < 0.0;
  }
  require(positive_knee && negative_knee, "near straight keeps both knee signs");
}

// 这些大角度只验证几何候选枚举，不代表舵机限位允许的实机动作。
void test_candidate_counts_and_folded_boundary(const leg_kinematics& kinematics) {
  const double pi = std::acos(-1.0);
  const vector3 four_point{0.0, -0.1, 0.0};
  const auto four = kinematics.inverse_candidates_in_leg_frame(four_point);
  require_valid_candidates(kinematics, four_point, four, "four branches");
  require(four.solution.size() == 4, "both radial signs have both knee branches");
  std::size_t forward_yaw_count = 0;
  std::size_t reverse_yaw_count = 0;
  for (const auto& candidate : four.solution) {
    const double yaw = candidate.angles.yaw_rad;
    if (std::abs(yaw) < 1e-10) ++forward_yaw_count;
    if (std::abs(std::abs(yaw) - pi) < 1e-10) ++reverse_yaw_count;
    require(!candidate.knee_near_singular, "four ordinary branches have no knee flag");
  }
  require(forward_yaw_count == 2 && reverse_yaw_count == 2,
          "four candidates cover both radial directions");

  // 42 - 74.2 - 112.53 = -144.73 mm：一套反向伸直，加另一径向的两套解。
  const vector3 three_point{0.0, 0.14473, 0.0};
  const auto three = kinematics.inverse_candidates_in_leg_frame(three_point);
  require_valid_candidates(kinematics, three_point, three, "three branches");
  require(three.solution.size() == 3, "one merged boundary plus two ordinary branches");
  require(contains_angles(three, {0.0, pi, 0.0}), "reverse straight pose recovered");
  std::size_t flagged_count = 0;
  for (const auto& candidate : three.solution) {
    if (candidate.knee_near_singular) ++flagged_count;
  }
  require(flagged_count == 1, "only reverse straight candidate has knee flag");

  const leg_joint_angles folded{0.2, 0.5, pi};
  const auto folded_point = kinematics.forward_in_leg_frame(folded);
  const auto folded_result = kinematics.inverse_candidates_in_leg_frame(folded_point);
  require_valid_candidates(kinematics, folded_point, folded_result, "folded boundary");
  require(contains_angles(folded_result, folded), "folded boundary candidate recovered");
  std::size_t folded_count = 0;
  for (const auto& candidate : folded_result.solution) {
    if (same_angles(candidate.angles, folded)) {
      ++folded_count;
      require(candidate.knee_near_singular, "folded boundary has knee flag");
    }
  }
  require(folded_count == 1, "equivalent positive/negative pi knees merged once");
}

void test_free_angles(const leg_kinematics& kinematics, const json& root) {
  for (const double height_m : {0.0, 0.1}) {
    require_empty_result(kinematics, {0.0, 0.0, height_m}, leg_ik_status::singular,
                         "reachable point on free yaw axis");
  }

  // 仅修改内存中的配置副本，使两段等长；真实硬件配置保持原样。
  auto equal_link_fixture = root;
  equal_link_fixture["model_geometry"]["actual_foot_contact_point"]["position_mm"] =
      {0.0, -74.2, 0.0};
  const leg_kinematics equal_links(robot_model::from_json_text(equal_link_fixture.dump()));
  require_empty_result(equal_links, {0.0, -0.042, 0.0}, leg_ik_status::singular,
                       "equal links fully folded with free upper angle");
}

void test_unreachable_and_invalid_input(const leg_kinematics& kinematics) {
  const std::array<vector3, 3> unreachable{{
      {0.0, -0.228730001, 0.0}, {0.0, 0.0, 0.19}, {0.0, -0.1, 0.2}}};
  for (std::size_t index = 0; index < unreachable.size(); ++index) {
    require_empty_result(kinematics, unreachable[index], leg_ik_status::unreachable,
                         "outside geometric reach " + std::to_string(index));
  }

  for (std::size_t axis = 0; axis < 3; ++axis) {
    const auto label = "coordinate " + std::to_string(axis);
    for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
                                 std::numeric_limits<double>::infinity(),
                                 -std::numeric_limits<double>::infinity()}) {
      std::array<double, 3> coordinates{0.01, -0.1, 0.03};
      coordinates[axis] = invalid;
      require_throws<std::invalid_argument>([&] {
        (void)kinematics.inverse_candidates_in_leg_frame(
            {coordinates[0], coordinates[1], coordinates[2]});
      }, label + " rejects non-finite input");
    }
    for (const double largest : {std::numeric_limits<double>::max(),
                                 -std::numeric_limits<double>::max()}) {
      std::array<double, 3> coordinates{0.01, -0.1, 0.03};
      coordinates[axis] = largest;
      require_empty_result(kinematics, {coordinates[0], coordinates[1], coordinates[2]},
                           leg_ik_status::unreachable, label + " finite huge input");
    }
  }
  const double largest = std::numeric_limits<double>::max();
  require_empty_result(kinematics, {largest, largest, largest}, leg_ik_status::unreachable,
                       "finite coordinates with overflowing total distance");
}

}  // namespace

// 只读配置并执行离线断言；不连接控制板或发出舵机指令。
int main(int argc, char** argv) {
  try {
    require(argc == 2, "usage: ik_tests robot_hardware.json");
    std::ifstream input(argv[1]);
    require(static_cast<bool>(input), "cannot read baseline fixture");
    const auto root = json::parse(input);
    const leg_kinematics kinematics(robot_model::from_json_text(root.dump()));
    test_independent_zero_pose(kinematics);
    test_pose_round_trips(kinematics);
    test_near_straight_branches(kinematics);
    test_candidate_counts_and_folded_boundary(kinematics);
    test_free_angles(kinematics, root);
    test_unreachable_and_invalid_input(kinematics);
    std::cout << "PASS: independent zero pose, 40 straight and 240 ordinary FK/IK poses, "
                 "near-straight branches, four/three candidates, folded deduplication, "
                 "free-angle singularities and invalid input.\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
