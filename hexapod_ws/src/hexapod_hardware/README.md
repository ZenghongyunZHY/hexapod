# 六足机器人串口通信节点

`hexapod_serial` 订阅 `servo_plan` 和 `plan_stop`，发布 `execution_status` 和 `standing_reference`。`serial_node.cpp` 负责话题和确认服务，`serial_runtime.cpp` 的工作线程独占串口，`plan_executor.cpp` 验收计划并排程，`servo_controller.cpp` 用现有控制板协议收发。

| `mode` | 行为 |
| --- | --- |
| `probe` | 使用 `serial_port` 指定的设备执行只读电压探测。18 轴位置回读属于实机站姿过渡和验收。 |
| `execute`（默认） | 使用 `serial_port` 指定的设备。首次执行和重连后通过 `/transition_to_standing` 建立站姿就绪。 |

V1 执行模式使用 `standing_positions` 传入与规划器相同的 18 轴名义站姿。调用 `/transition_to_standing` 后撤销旧站姿和本地队列，等待板上已交付的帧结束，再一次发送 18 轴目标和 2000 ms 时长。操作者负责避碰。到期后回读 18 轴，每条腿的水平、高度偏差分别不超过 10 mm 才发布新的 `standing_reference` 就绪时刻。过渡期间丢弃运动与日常 STOP。后续计划携带该就绪时刻，执行器仅接受同一轮就绪生成的计划，从名义站姿开始核对计划起点。

`diagnostics` 提供通信、电压和位置查询耗时。有效回读经 `joint_states` 对外发布以供诊断，回包结束时间用作近似时间戳。

空闲时首帧保留 `send_advance_ms` 的提前量，默认为 90 ms。9600 8N1 下，全身目标包的线缆耗时约为 64 ms。后续帧按前一帧完整写入内核的单调时钟时刻加上其动作时长排程，使下一帧的传输可以与前一帧动作重叠。排程迟醒最多允许 `frame_lateness_ms`（默认 25 ms），每次写入另有 `write_timeout_ms`（默认 25 ms）的期限。超出任一预算即报告失败并撤销站姿就绪。衔接计划的首帧等待前一计划估计结束。

`ACCEPTED` 提供 `maximum_finish_delay_ms`，供上层在名义结束时刻之外留出有限的执行时间。`COMPLETED` 表示所有帧已提交，且最后一帧的预计线缆时间、动作时长与 `completion_margin_ms`（默认 5 ms）均已过去。`plan_stop` 在可达的完整周期边界中止，已进入收脚时完成收脚。预计时刻基于串口写入记录，实机动作起止与跟踪情况需测量。

串口协议和调度可在脱离 ROS 的 CMake 构建中测试。Linux 的 `hexapod_bringup` 提供 V1 三节点私有虚拟串口验收，用例自行创建伪终端，核对过渡、抢占、失败和重试。
