# hexapod_core 迁移与验证记录

日期：2026-10-17。

后续约定更新：用户改为 Windows 本地 VS Code 编辑、树莓派编译运行。本地 `hexapod_ws/` 已从树莓派恢复源码与配置文件，保留双端；下方“本机清理”是此前操作的历史记录，不再作为后续默认规则。使用方式见 [Windows 工作流](windows-vscode-workflow.md)。

## 开发位置

用户明确要求：后续代码默认直接在树莓派编写、构建和测试；移植后清理此次 Windows 本机源码及构建产物。该约定已写入本机 AGENTS.md，并同步到树莓派工作区。

- SSH：使用现有别名 `hexapod-pi`，连接账号 `hexapod`。
- 工作区：`/home/hexapod/hexapod_ws`。
- 源码：`/home/hexapod/hexapod_ws/src/hexapod_core`。
- 硬件配置：`/home/hexapod/hexapod_ws/config/robot_hardware.json`。
- 物理基准：`/home/hexapod/hexapod_ws/docs/physical-baseline.md`。
- JSON 依赖：`/home/hexapod/hexapod_ws/dependencies/json-3.11.3`。

本机继续保留原始硬件资料、物理信息副本、架构与验证记录，以及旧 demo 历史源码。

## 已完成

- 确认树莓派为 aarch64，ROS 2 Jazzy、colcon、g++ 和 CMake 可用。
- 同步基础库全部 9 个源文件/构建说明文件，传输后逐文件核对 SHA-256，一致。
- 同步物理配置和基准文档；配置 SHA-256 为 `70585b1207d7ca0d7467fb6ed7cbef12984587da4411d9d594b4e1d99282c052`，未补造物理参数。
- 树莓派未安装系统 JSON 开发包，sudo 需要密码，故在工作区内准备 nlohmann/json 3.11.3 头文件、MIT 许可及最小 CMake 发现配置，不修改系统目录。
- 调整树莓派 CTest 默认配置路径为工作区 `config/robot_hardware.json`，提供 `hardware_config_path` 覆盖项。
- 在树莓派写入开发约定、工作区 README 与依赖来源记录。

## 验证状态

Windows 本机此前 C++17 / CMake 构建、CTest（1/1）和安装检查通过。

首次 Release 构建启动后 SSH 多次握手超时，未取得该次构建结果。用户确认设备本地终端正常；后续 SSH 恢复，地址仍为 10.36.49.166。实测系统总内存约 899 MiB、无交换空间；日志显示并行编译。检查树莓派 colcon 的构建实现后确认：其默认按 CPU 核数给 make 添加 -j 参数，单独设置 CMAKE_BUILD_PARALLEL_LEVEL 没有限制住并行编译。资源压力可能与先前连接异常有关，尚无内核 OOM 日志证明具体原因。

已直接在树莓派创建 `/home/hexapod/hexapod_ws/scripts/build_core.sh`，显式设置 MAKEFLAGS=-j1 和 CMAKE_BUILD_PARALLEL_LEVEL=1，使用 Debug / -O0 -g0 低内存验证配置。脚本为自身及子进程设置 512 MiB 虚拟内存上限，未修改系统配置。

树莓派验证已完成：

- aarch64 / GCC 13.3.0 / ROS 2 Jazzy，colcon 构建并安装成功，包构建耗时约 34.3 秒。
- CTest：model_and_mapping 通过，1/1，0 errors / 0 failures / 0 skipped；测试耗时约 0.18 秒。
- 覆盖真实 18 轴映射、未知标定、双向转换、量化限位、部分标定、几何与非法配置处理。
- `ros2 pkg prefix hexapod_core` 返回 `/home/hexapod/hexapod_ws/install/hexapod_core`。
- 单任务构建过程中 SSH 可用；一次资源快照中编译器 RSS 约 269 MiB，系统可用内存约 361 MiB，此快照不是峰值测量。
- 硬件配置 SHA-256 保持不变。没有打开舵机串口或执行实机动作。

## 后续构建命令

```bash
cd /home/hexapod/hexapod_ws
./scripts/build_core.sh
```

## 本机清理

已清理本次创建的 `ros2_ws/src/hexapod_core`、`ros2_ws/build/hexapod_core_host`、`ros2_ws/build/hexapod_core_cmake` 和 `ros2_ws/install/hexapod_core_host`。删除前已确认源码传输 SHA-256 一致，检查绝对路径与文件清单；按文件清理后移除空目录，未删除旧 demo、硬件资料或物理配置。

树莓派源码已迁移并校验，树莓派构建与离线测试也已通过。后续用户要求恢复 Windows 编辑副本，现本地源码位于 `hexapod_ws/`，与树莓派工作区按需同步，不恢复之前的本机构建产物。
