# 六足机器人规划器节点

`hexapod_planner` 使用 `hexapod_core` 生成并检查起步、三脚行走、减速和收脚轨迹。命令接收节点提供用户意图，串口通信节点负责执行和回报。三个节点通过 `hexapod_interfaces` 定义的话题交换数据。

## 命令接收节点

`hexapod_command` 将 `joy`（`sensor_msgs/msg/Joy`）和 `command_request`（`MotionCommand`）统一转发到 `motion_command`。用机器人系统时钟给摇杆样本打戳，并在设备在线时周期性发布当前轴值。按手柄配置 `axis_indices`、`axis_minimums`、`axis_centers`、`axis_maximums`、`axis_directions`、`joystick_deadzone` 和 `stop_button`。三个轴参数依次对应前进、左移和偏航。接收端给输出统一编号，规划器在 `planner_status` 发布任务状态。

```bash
ros2 run hexapod_control hexapod_command
ros2 topic pub --once /command_request hexapod_interfaces/msg/MotionCommand \
  '{kind: 2, vx_m_s: 0.0005, cycle_count: 1}'
ros2 topic pub --once /command_request hexapod_interfaces/msg/MotionCommand '{kind: 3}'
ros2 topic echo /planner_status
```

上例的指定周期和停止是离散请求，缺省时间戳时由接收节点在收取时填写。通过 `command_request` 注入 `TELEOP` 时使用实际采样时间。手柄在收到停止、遥控超期或指定周期任务结束后先回中，再开启下一次遥控。

```text
命令接收节点 ── motion_command ──→ 规划器节点 ── servo_plan ──→ 串口通信节点
                                     │      ←─ execution_status ─┘
                                     ├── plan_stop ────────────→ 串口通信节点
                                     └── planner_status ───────→ 状态观察方
```

## 启动与站姿

`hardware_config` 指向工作区的 `config/robot_hardware.json`。`standing_positions` 需要提供经确认的屈膝站姿，包含按物理舵机 ID 1 到 18 排列的整数位置。节点从站姿角做 FK，得到六足站姿参考。当前姿态通过 `standing_reference` 单独确认。离线测试使用显式测试姿态和假执行端。

节点启动后等待有效站姿参考和运动请求。站姿参考应由能确认机器人当前姿态的一方发送，时间戳采用机器人系统时钟。

常用参数如下。这些参数在启动时加载。默认周期、速度和时间预算是规划侧开发参数。步态配置变动后，需要重新核对缩放后的最短与最长动作段。

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `cycle_period_s`、`stance_ratio`、`intervals_per_stage` | 4 s、0.625、10 | 基础周期、支撑占空比、每阶段目标分段数。 |
| `lift_height_m`、`recovery_lift_height_m` | 0.003 m、0.003 m | 行走与收脚的额外抬脚高度。 |
| `linear_step_m_s`、`yaw_step_rad_s` | 0.0005 m/s、0.003 rad/s | 相邻周期允许的最大规划速度变化。 |
| `command_timeout_ms`、`center_hold_ms` | 500 ms、150 ms | 遥控输入有效期、持续回中判定时间。 |
| `time_scale`、`min_segment_ms`、`max_segment_ms` | 4、200 ms、5000 ms | 整批时间倍率与转换后段时长范围。 |
| `plan_lead_ms`、`lookahead_ms` | 350 ms、1200 ms | 首帧预留量、允许规划接续批次的时间窗口。 |

在 Pi 上可以单任务构建节点和依赖。在工作区根目录执行下面的命令，构建和测试只涉及软件。

```bash
source /opt/ros/jazzy/setup.bash
export MAKEFLAGS=-j1
export CMAKE_BUILD_PARALLEL_LEVEL=1
colcon build --packages-up-to hexapod_control --executor sequential \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Debug \
  '-DCMAKE_CXX_FLAGS_DEBUG=-O0 -g0' \
  "-Dnlohmann_json_DIR=$PWD/dependencies/json-3.11.3/cmake"
source install/setup.bash
ros2 interface show hexapod_interfaces/msg/ServoPlan
colcon test --packages-select hexapod_core hexapod_control --executor sequential
colcon test-result --verbose
```

## 话题约定

以下均为相对话题名，可通过命名空间或重映射隔离一台机器人。消息类型均属于 `hexapod_interfaces/msg`。各话题使用可靠传输和易失持久性，执行端仍需明确回报整批验收结果。

| 话题 | 方向 | 类型 | 含义 |
| --- | --- | --- | --- |
| `motion_command` | 输入 | `MotionCommand` | 接收节点发送 `TELEOP`、`WALK_CYCLES` 或 `STOP`。单一发布者使用严格递增的 `sequence` 及按顺序产生的机器人系统时间戳。规划器继续运行时，发布者重启后需要延续编号。 |
| `standing_reference` | 输入 | `StandingReference` | 经确认的当前站姿，带时间戳和按物理 ID 排列的 18 个位置。 |
| `execution_status` | 输入 | `ExecutionStatus` | 串口执行端按 `plan_id` 报告验收、完成、拒绝、失败或中止。 |
| `servo_plan` | 输出 | `ServoPlan` | 执行批次，包含 `plan_id`、期望动作起点、起始参考位置及各段的相对起点、动作时长和 18 个段终点目标。 |
| `plan_stop` | 输出 | `PlanStop` | 请求执行端让指定计划在下一个可停止边界停下。 |
| `planner_status` | 输出 | `PlannerStatus` | 通过 `command_sequence` 和 `plan_id` 关联规划状态和失败原因。 |

对于 `TELEOP`，现有的 `vx_m_s`、`vy_m_s` 和 `wz_rad_s` 字段承载无量纲系数。平移两轴组成的向量模长不超过 1，偏航系数在 `[-1, 1]`。规划器分别乘 `max_linear_speed_m_s` 和 `max_yaw_speed_rad_s`，将样本换算为实际目标速度后入队。比如默认上限下，遥控样本 `(0.6, 0.8, -0.5)` 换算为 `(0.003 m/s, 0.004 m/s, -0.015 rad/s)`。对于 `WALK_CYCLES`，仍按 m/s 和 rad/s 填写并校验任务速度。`STOP` 忽略速度和周期数字段。

遥控输入按 FIFO 处理并用样本时刻更新目标速度。手柄回中持续发送有效零速，暂时缺包时沿用有效期内的意图，回中达到持续时间或输入超期时进入减速与收脚。固定周期请求中的 `cycle_count` 只计完整行走周期，起步和减速用的行走周期也计入。首版速度在完整周期边界以阶梯形式改变。规划器在发布前检查完整轨迹，原地收脚使用当前机身位置与站姿脚点。运动学检查记录的是模型中的足端、关节限位和路径状态。

规划器为实时遥控生成一个完整周期一批的短计划。在执行端验收当前计划后，规划器可提前提交与当前计划末态连续的下一批。执行端应按计划时间排序，最多保留一个已接续的后续批次。规划器统一缩放整条执行时间轴一次，18 轴位置保持为规划产生的名义位置，段时长由相邻整数毫秒边界相减得到。

### 执行状态与停止

- `ACCEPTED` 表示执行端验收了整批。执行端按批次顺序在其计划终点后回报 `COMPLETED`，并将 `completed_segments` 填为总段数。回报要与当前批次的 `plan_id` 匹配。
- `INTERRUPTED.completed_segments` 是已完成的前缀段数。只有执行端明确确认首个目标尚未发出时才可报零。正值的最后一段必须带 `walking_cycle_end`，这样规划器才能继承已完成周期的末态。原地收脚段没有该标记。
- `PlanStop` 要求执行端联动撤下依赖该批次的后续计划，并回报完成的段数。若已进入原地收脚，可完成收脚后回报 `COMPLETED`。规划器按完成的周期末态接续。执行失败或使用 `plan_id = 0` 回报参考丢失时，需要重新确认站姿参考再规划。
- `ServoPlan.motion_start` 采用机器人系统时钟，表示首段期望动作时刻。执行端验收后应换算成自己的单调时钟起点，结合链路余量安排发包。

`planner_status.state` 提供 `waiting_for_reference`、`standing`、`planned`、`accepted`、`cycle_complete`、`stopping`、`interrupted`、`rejected` 和 `fault`。发布计划只产生 `planned`，由执行端报告验收和完成后才更新相应状态。

## 离线验证

`planner_node_smoke` 在本机独立 ROS 域和独立话题命名空间启动规划器。假执行端用软件测试姿态验证站姿门槛、固定周期计划、按批次匹配完成回报，以及周期中途停止回报导致的参考失效。构建并加载工作区后，可通过 `colcon test --packages-select hexapod_control` 运行。
