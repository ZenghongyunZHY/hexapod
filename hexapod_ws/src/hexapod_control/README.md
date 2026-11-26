# 六足机器人规划器节点

`hexapod_planner` 使用 `hexapod_core` 生成并检查起步、三脚行走、减速和收脚轨迹。命令接收节点提供用户意图，串口通信节点负责执行和回报。三个节点通过 `hexapod_interfaces` 定义的话题交换数据。

## 命令接收节点

`hexapod_command` 将 `joy`（`sensor_msgs/msg/Joy`）和 `command_request`（`MotionCommand`）统一转发到 `motion_command`。用机器人系统时钟给摇杆样本打戳，并在设备在线时周期性发布当前轴值。按手柄配置 `axis_indices`、`axis_minimums`、`axis_centers`、`axis_maximums`、`axis_directions`、`joystick_deadzone` 和 `stop_button`。三个轴参数依次对应前进、左移和偏航。接收端给输出统一编号，规划器在 `planner_status` 发布任务状态。

```bash
ros2 run hexapod_control hexapod_command
ros2 topic echo /planner_status
```

所有 `command_request` 都需要填写机器人系统时钟下的原始请求时间。`TELEOP` 使用实际采样时间。接收节点只转发最近一次就绪时间之后产生的指令，也在站姿失效时清除输入门控状态。手柄在收到停止、遥控超期或指定周期任务结束后先回中，再开启下一次遥控。

```text
命令接收节点 ── motion_command ──→ 规划器节点 ── servo_plan ──→ 串口通信节点
                                     │      ←─ execution_status ─┘
                                     ├── plan_stop ────────────→ 串口通信节点
                                     └── planner_status ───────→ 状态观察方
```

## 启动与站姿

`hardware_config` 指向工作区的 `config/robot_hardware.json`。`standing_positions` 与串口节点使用同一份站姿配置，包含按物理舵机 ID 1 到 18 排列的整数位置。节点用正运动学从该名义站姿计算六个足端参考。串口节点执行 `/transition_to_standing` 并完成回读验收后，通过 `standing_reference` 发送就绪事件。

收到失效事件时，规划器清除待执行计划和指令，随后等待新的就绪事件。新计划从配置的名义站姿开始，只使用新就绪时间之后产生的运动请求。每批计划都携带其就绪时间，用于执行端核对站姿代次。

常用参数如下。这些参数在启动时加载。步态配置变动后，需要重新核对各段时长和完整轨迹。

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `cycle_period_s`、`stance_ratio` | 4.0 s、0.55 | 行走周期与支撑占空比。 |
| `double_support_intervals`、`swing_intervals` | 1、9 | 每个双支撑阶段与每个摆动阶段的目标分段数。 |
| `lift_height_m`、`outward_offset_m` | 0.040 m、0.025 m | 行走中脚端在摆动中点相对起落点基线的抬升和水平外移。 |
| `recovery_lift_height_m` | 0.008 m | 原地收脚的抬脚高度。 |
| `max_linear_speed_m_s`、`max_yaw_speed_rad_s` | 0.005 m/s、0.03 rad/s | 指定周期路线的常规平移与偏航上限。 |
| `max_forward_speed_m_s` | 0.035 m/s | 手柄正向满杆和指定周期路线纯前进的上限。指定周期纯前进要求 `vx_m_s > 0`，且 `vy_m_s` 和 `wz_rad_s` 均为零。 |
| `teleop_reverse_speed_m_s`、`teleop_lateral_speed_m_s`、`teleop_yaw_speed_rad_s` | 0.035 m/s、0.020 m/s、0.20 rad/s | 手柄后退、左右横移和正负偏航的单轴满杆速度。 |
| `linear_step_m_s`、`yaw_step_rad_s` | 0.00542 m/s、0.03 rad/s | 相邻周期允许的最大平移速度向量变化与最大偏航角速度变化。 |
| `max_joint_speed_rad_s` | 2.0 rad/s | 规划器检查每个关节的名义段平均角速度时使用的上限。 |
| `command_timeout_ms`、`center_hold_ms` | 500 ms、150 ms | 遥控输入有效期、持续回中判定时间。 |
| `min_segment_ms`、`max_segment_ms` | 200 ms、5000 ms | 允许的单段动作时长范围。 |
| `plan_lead_ms`、`lookahead_ms` | 350 ms、1200 ms | 首帧预留量、允许规划接续批次的时间窗口。 |
| `feedback_grace_ms` | 1000 ms | 在执行端给出的最晚完成时刻之外，等待反馈的余量。 |

指定周期路线、手柄遥控与减速阶段共用同一套三脚步态。默认每半周期有 1 个双支撑段和 9 个摆动段，每段均为 200 ms，整周期 20 段。原地收脚按 4.8 s 周期、2/3 支撑占空比和每阶段 4 段生成，共 16 段，使用 8 mm 抬脚和零外移。行走末态在站姿容差外时才安排收脚。

指定周期路线从起止两侧对称应用周期速度上限，第 1、第 2 和后续阶段的上限分别为该方向速度上限的约 56.94%、85.42% 和 100%。规划器对平移和偏航使用同一比例收紧请求速度，并遵守周期速度变化限制。默认参数下，以 `0.035 m/s` 请求直行 13 个周期，前七个周期速度依次为 0.00542、0.01084、0.01626、0.02168、0.02710、0.03252、0.03500 m/s，后六个周期对称减速。峰值周期名义前移 140 mm，全程名义前移 1050.56 mm。行走名义时长为 52 s，收脚若需要另计 4.8 s，首帧预留和串口调度顺延另计。

从 130.08 mm 周期接入 140 mm 峰值周期时，右后腿大腿转轴至足端的最大目标距离约为 186.545 mm，距完全伸直的 186.73 mm 上界约 0.185 mm。这是固定站姿和 CAD 几何下的连续轨迹计算值。实际装配干涉及带载余量待实测。六个膝关节的软件负向下限为 −96°，正向限位仍为 +86.4°。 [物理基准](../../docs/physical-baseline.md)

V1 使用 ARM64 交叉编译和 Pi 链接安装流程。安装使用同一版消息定义的节点和依赖后，可从工作区根目录运行离线用例。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 interface show hexapod_interfaces/msg/ServoPlan
colcon test --packages-select hexapod_core hexapod_control --executor sequential
colcon test-result --verbose
```

## 话题约定

以下均为相对话题名，可通过命名空间或重映射隔离一台机器人。消息类型均属于 `hexapod_interfaces/msg`。各话题使用可靠传输。`standing_reference` 使用深度 1 的 `transient_local` 保留最近状态，其余话题使用易失持久性。执行端仍需明确回报整批验收结果。

| 话题 | 方向 | 类型 | 含义 |
| --- | --- | --- | --- |
| `motion_command` | 输入 | `MotionCommand` | 接收节点发送 `TELEOP`、`WALK_CYCLES` 或 `STOP`。单一发布者使用严格递增的 `sequence` 及按顺序产生的机器人系统时间戳。规划器继续运行时，发布者重启后需要延续编号。 |
| `standing_reference` | 输入 | `StandingReference` | 站姿状态及其机器人系统时钟时间戳。`ready=false` 使旧站姿失效，`ready=true` 表示过渡回读已通过。 |
| `execution_status` | 输入 | `ExecutionStatus` | 串口执行端按 `plan_id` 报告验收、完成、拒绝、失败或中止。 |
| `servo_plan` | 输出 | `ServoPlan` | 执行批次，包含 `plan_id`、生成该批次的 `standing_stamp`、期望动作起点、名义起始位置及各段的相对起点、动作时长和 18 个段终点目标。 |
| `plan_stop` | 输出 | `PlanStop` | 请求执行端让指定计划在下一个可停止边界停下。 |
| `planner_status` | 输出 | `PlannerStatus` | 通过 `standing_stamp`、`command_sequence` 和 `plan_id` 关联站姿状态、命令与计划。 |

对于 `TELEOP`，现有的 `vx_m_s`、`vy_m_s` 和 `wz_rad_s` 字段承载无量纲系数 `(x,y,z)`。平移两轴的向量模长不超过 1，偏航系数在 `[-1, 1]`。取杆量 `a = max(hypot(x,y), abs(z))`，把 `a` 按 `abs(x):abs(y):abs(z)` 分给三个轴，再按各轴符号及满杆上限换算为名义速度。全零直接映射为零。这样同一方向的半杆和满杆是线性关系，组合杆量共用同一个速度预算。例如纯前进满杆目标为 `0.035 m/s`，前进和偏航同时满杆时目标分别为 `0.0175 m/s` 与 `0.10 rad/s`。`WALK_CYCLES` 按 m/s 和 rad/s 填写并校验任务速度。`STOP` 忽略速度和周期数字段。

遥控输入按 FIFO 处理。每个周期采用最新有效目标，按平移及偏航步进从已计划的周期末态向目标接近。发布前用正式步态规划器核对候选整周期，再从候选末态核对完整减速和收脚，包含关节限位、名义平均角速度、段内足端约束及可下发的毫秒时长。全步进不通过时，依次尝试当前步进的 75%、50% 和 25%。全部失败则从前一个已接纳周期末态按已检查的路径减速。持续回中或输入超期也进入减速与收脚。固定周期请求中的 `cycle_count` 计入起步和减速用的完整行走周期。原地收脚使用当前规划机身位置与站姿脚点。

规划器为实时遥控生成一个完整周期一批的短计划。在执行端验收当前计划后，规划器可提前提交与当前计划末态连续的下一批。执行端应按计划时间排序，最多保留一个已接续的后续批次。规划时刻直接换算为整数毫秒，段时长由相邻边界相减，18 轴位置使用规划产生的名义位置。

### 执行状态与停止

- `ACCEPTED` 表示执行端验收了整批，并提供该批相对名义终点的最大完成延迟 `maximum_finish_delay_ms`。规划器在单调时钟上计算完成超时，只接受第一次确认，并以 `planner_status.completion_timeout_ms` 为上层提供剩余等待预算。
- `COMPLETED` 由执行端在最后一段结束后按批次顺序回报，`completed_segments` 填为总段数。规划器根据批次编号、验收和段数确认完成，名义时间轴用于计划接续排程，实际结束以执行端的回报为准。
- `INTERRUPTED.completed_segments` 是已完成的前缀段数。只有执行端明确确认首个目标尚未发出时才可报零。正值的最后一段必须带 `walking_cycle_end`，这样规划器才能继承已完成周期的末态。原地收脚段没有该标记。
- 日常 `PlanStop` 要求执行端联动撤下依赖该批次的后续计划，并回报完成的段数。规划器从最后完成的周期末态按速度步进减速，再安排收脚。默认参数下，若从 `0.035 m/s` 的周期边界开始减速，后续六个周期速度依次为 0.02958、0.02416、0.01874、0.01332、0.00790、0.00248 m/s，名义前移合计 0.38472 m。从收到 STOP 到该周期边界的剩余行走另计。若已进入原地收脚，可完成收脚后回报 `COMPLETED`。站姿过渡直接通过 `standing_reference` 使旧计划失效。发生执行故障后，等待新的站姿就绪事件。
- `ServoPlan.motion_start` 采用机器人系统时钟，表示首段期望动作时刻。执行端验收后应换算成自己的单调时钟起点，结合链路余量安排发包。

`planner_status.state` 提供 `waiting_for_reference`、`standing`、`planned`、`accepted`、`cycle_complete`、`stopping`、`interrupted`、`rejected` 和 `fault`。发布计划只产生 `planned`，由执行端报告验收和完成后才更新相应状态。

## 离线验证

`planner_node_smoke` 在本机独立 ROS 域和独立话题命名空间启动规划器。假执行端覆盖过渡抢占、旧指令丢弃、名义站姿起步、按批次匹配完成回报、允许的完成延迟，以及日常 STOP 的周期检查。`command_receiver_smoke` 覆盖过渡期间及旧时间戳的手柄、路线和 STOP 拒绝，以及新就绪后的转发。构建并加载工作区后，可通过 `colcon test --packages-select hexapod_control` 运行。

`planner_gait_smoke` 用假执行端核对行走帧表、足端几何和各方向限速，覆盖前进、后退、横移、转向、组合路线和遥控，以及首帧前停止与峰值周期后的减速和收脚。`hardware_v1_pty` 使用三个节点和虚拟串口，核对已完成的行走帧与收脚首帧的时序。
