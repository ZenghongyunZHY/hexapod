# Hexapod 六足机器人

18 只幻尔 LX-225 总线舵机，Lm2 金属框架，计划使用树莓派、ROS 2 和 C++ 开发。

当前开发工作区是 `hexapod_ws/`。开始涉及机器人几何、关节或舵机控制的开发前，先查阅：

- [当前物理信息基准](hexapod_ws/docs/physical-baseline.md)：已确认的编号、安装方向、500 展开姿态、转向、腿长定义及待确认项。
- [结构化硬件信息](hexapod_ws/config/robot_hardware.json)：程序可读取的编号与参数，未知项保留 `null`。
- [原始资料清单](references/manifest.json)：已保存的照片、CAD 和规格书及校验值。

CAD 测量值与实物确认值分开保存；当前采用已确认的 V1 CAD 理想模型，尚未完成全部实物尺寸标定。根目录 `config/`、`docs/` 中的同名副本保留历史状态，当前开发以工作区版本为准。

2026-11-03：本地已加入三脚相位模块和正式测试，Windows 严格构建及 CTest 12/12 通过；Pi 仍保留 2026-11-02 的 11/11 验证版本，本轮未同步 Pi、未执行实机动作。详细进度见[工作区说明](hexapod_ws/README.md)。

GitHub 仓库提交范围与克隆后的注意事项见[仓库文件说明](docs/github-repository.md)。第三方 CAD、PDF、规格书页面截图保留在本机，不随仓库分发；资料清单中的路径和哈希用于追溯来源。构建与核心离线测试不需要这些原始资料。

v1 当前规划：[运动学步态与误差调节扩展接口架构](hexapod_ws/docs/v1-architecture.md)。基于现有控制板，优先实现本地运动学、候选选择、步态与动作下发，保留可选 IMU 接口。2026-10-27 用户将角度回读闭环误差调节改为仅预留接口，到位标定与补偿实现后移；尚未实现的模块与运行频率仍需后续实现、验证。根目录 docs/v1-architecture.md 保留为历史方案，当前范围以工作区文档为准。

截至 2026-11-02，[hexapod_core](hexapod_ws/src/hexapod_core/README.md) 已实现模型/映射、FK/IK、候选选择、平均速度约束、路径采样检查、六腿段规划、脚端摆动曲线和单腿离线 CSV 预览。Windows 与树莓派 aarch64 构建及 CTest 均 11/11 通过，安装后的预览入口已验证，双端 20 行 CSV 数值一致；未执行实机动作。当前开发方式为 Windows 本地 VS Code 编辑、树莓派编译运行，保留双端源码。Windows 打开 [hexapod.code-workspace](hexapod.code-workspace)，编辑目录为 `hexapod_ws/`，对应树莓派 `/home/hexapod/hexapod_ws`。用户本地修改后明确要求同步时再上传。当前证据见工作区 [验证记录](hexapod_ws/docs/core-deployment-validation.md)，操作约定见 [Windows 工作流](hexapod_ws/docs/windows-vscode-workflow.md)。

固定帧演示：[站立与六周期步态](ros2_ws/src/hexapod_demo/README.md)。历史 demo 源码保留在本机 `ros2_ws/src/hexapod_demo`；2026-10-17 用户确认测试已完成后，树莓派上的旧 demo、编译与安装产物及测试日志已清理，ROS 2 和开发环境保留。随后已在树莓派工作区部署新的 `hexapod_core`。
