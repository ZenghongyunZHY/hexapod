# 六足机器人串口通信节点

`hexapod_serial` 订阅 `servo_plan` 和 `plan_stop`，发布 `execution_status` 和 `standing_reference`。`serial_node.cpp` 负责话题和确认服务，`serial_runtime.cpp` 的工作线程独占串口，`plan_executor.cpp` 验收计划并排程，`servo_controller.cpp` 用现有控制板协议收发。

| `mode` | 行为 |
| --- | --- |
| `preview`（默认） | 使用软件控制板按时间表执行；在确认服务前显式设置 `preview_positions`。 |
| `probe` | 使用 `serial_port` 指定的设备执行只读电压与位置探测。 |
| `execute` | 使用 `serial_port` 指定的设备执行已验收计划；首次执行和重连后都须重新确认站姿。 |

从已构建的 `hexapod_ws` 启动预览。下面的 18 轴值是软件测试夹具；规划器预览使用相同的 `standing_positions`。

```bash
source install/setup.bash
ros2 run hexapod_hardware hexapod_serial --ros-args -p mode:=preview \
  -p "hardware_config:=$PWD/config/robot_hardware.json" \
  -p 'preview_positions:=[500,550,200,500,550,200,500,550,200,500,450,800,500,450,800,500,450,800]'
ros2 service call /confirm_standing std_srvs/srv/Trigger '{}'
```

确认服务在规划器订阅 `standing_reference` 后调用。执行模式下仅在操作者已确认机器人处于站姿时调用。串口节点核对配置电压范围并取完整新鲜的 18 轴回读，服务成功表示参考已发布；规划器是否接受以 `planner_status` 为准。`joint_states` 使用有效串口回读，`diagnostics` 同时提供通信、电压及位置查询耗时。回包结束时间用作近似时间戳。

`send_advance_ms` 默认 90 ms。9600 8N1 下全身目标包的纯线缆耗时约 64 ms，实际起始时间与余量需通过实机测量确定。`COMPLETED` 表示目标均已提交且名义动作时间结束。`plan_stop` 在可达的完整周期边界中止，已进入收脚时完成收脚。

串口协议和调度可在脱离 ROS 的 CMake 构建中测试。Linux 另有虚拟串口测试；安装三个包后可运行 `colcon test --packages-select hexapod_bringup --executor sequential` 执行软件三节点联调。
