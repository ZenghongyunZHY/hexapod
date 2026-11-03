#pragma once

#include "hexapod_core/robot_model.hpp"
#include <vector>

namespace hexapod_core {

//一条腿的三个数学关节角，单位为弧度
struct leg_joint_angles {
  double yaw_rad;
  double upper_leg_rad;
  double lower_leg_rad;
};

enum class leg_ik_status{
  success,        //已获得离散候选，其中包含伸直解
  unreachable,    //目标超出几何可达范围
  singular        //存在未确定的自由角，当前接口还无法完全列举候选
};

struct leg_ik_candidate{
  leg_joint_angles angles;
  bool knee_near_singular = false;
};

struct leg_ik_result{
  leg_ik_status status = leg_ik_status::unreachable;
  std::vector<leg_ik_candidate>solution;
};

class leg_kinematics {
    public:
      leg_ik_result inverse_candidates_in_leg_frame(const vector3& point_in_leg) const;

      explicit leg_kinematics(robot_model model);

      //根据三个关节角度，计算脚端在腿根局部坐标系的位置，返回单位：m
      vector3 forward_in_leg_frame(const leg_joint_angles& angles) const;

      // 腿根局部坐标 → 机身坐标，输入和输出单位均为米。
      vector3 leg_to_body_frame(leg_id leg,const vector3& point_in_leg) const;

      // 机身坐标 → 腿根局部坐标，输入和输出单位均为米。
      vector3 body_to_leg_frame(leg_id leg,const vector3& point_in_body) const;

      // 三个关节角 → 机身坐标系中的脚端位置。
      vector3 forward_in_body_frame(leg_id leg,const leg_joint_angles& angles) const;

      // 机身坐标中的脚点 → 指定腿的局部 IK 候选。
      // 输入单位为米；只枚举几何候选。
      leg_ik_result inverse_candidates_in_body_frame(leg_id leg, const vector3& point_in_body) const;

    private:
      robot_model model_;
};

}
