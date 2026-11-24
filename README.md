# Hexapod 六足机器人

18 只幻尔 LX-225 总线舵机，Lm2 金属框架，计划使用树莓派、ROS 2 和 C++ 开发。

当前开发工作区是 `hexapod_ws/`。开始涉及机器人几何、关节或舵机控制的开发前，先查阅：

- [当前物理信息基准](hexapod_ws/docs/physical-baseline.md)：已确认的编号、安装方向、500 展开姿态、转向、腿长定义及待确认项。
- [结构化硬件信息](hexapod_ws/config/robot_hardware.json)：程序可读取的编号与参数，未知项保留 `null`。
- [原始资料清单](references/manifest.json)：已保存的照片、CAD 和规格书及校验值。

CAD 测量值与实物确认值分开保存；当前采用已确认的 V1 CAD 理想模型，尚未完成全部实物尺寸标定。根目录 `config/`、`docs/` 中的同名副本保留历史状态，当前开发以工作区版本为准。

截至 2026-11-24，Pi 已安装 V1 的三个 ROS 2 节点，覆盖站姿过渡、固定周期路线、遥控和周期边界停止。固定周期纯前进的规划峰值为 4 s 周期内名义前移 140 mm，已通过三节点虚拟串口验证，带载与实际地面位移待实测。 [开发日志](hexapod_ws/docs/core-deployment-validation.md)

GitHub 仓库提交范围与克隆后的注意事项见[仓库文件说明](docs/github-repository.md)。第三方 CAD、PDF、规格书页面截图保留在本机，不随仓库分发；资料清单中的路径和哈希用于追溯来源。构建与核心离线测试不需要这些原始资料。

v1 当前规划：[运动学步态与误差调节扩展接口架构](hexapod_ws/docs/v1-architecture.md)。基于现有控制板，优先实现本地运动学、候选选择、步态与动作下发，保留可选 IMU 接口。2026-10-27 用户将角度回读闭环误差调节改为仅预留接口，到位标定与补偿实现后移；尚未实现的模块与运行频率仍需后续实现、验证。根目录 docs/v1-architecture.md 保留为历史方案，当前范围以工作区文档为准。

Windows 在 `hexapod_ws/` 中编辑源码，Pi 保留对应的源码副本。完整 V1 由 WSL 交叉编译 ARM64，再在 Pi 单任务生成 Jazzy 接口、链接、安装和运行离线测试。 [VS Code 工作区](hexapod.code-workspace) · [Windows 工作流](hexapod_ws/docs/windows-vscode-workflow.md)

固定帧演示：[站立与六周期步态](ros2_ws/src/hexapod_demo/README.md)。历史 demo 源码保留在本机 `ros2_ws/src/hexapod_demo`；2026-10-17 用户确认测试已完成后，树莓派上的旧 demo、编译与安装产物及测试日志已清理，ROS 2 和开发环境保留。随后已在树莓派工作区部署新的 `hexapod_core`。
