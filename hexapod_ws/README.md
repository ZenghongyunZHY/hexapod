# Hexapod 双端源码工作区

Windows 编辑目录：`C:/Users/zengh/Documents/ChatGPT/hexapod/hexapod_ws`。
树莓派构建目录：`/home/hexapod/hexapod_ws`。
用户于 2026-10-17 更新要求：Windows 本地 VS Code 编辑，保留两端源码；助手修改时维护双端，用户自己修改后明确要求同步时再上传。只保持文件结构与内容一致，不要求 Windows 搭建相同的 ROS 2 / ARM 环境。

- [开发约定](AGENTS.md)
- [物理基准](docs/physical-baseline.md)
- [硬件配置](config/robot_hardware.json)
- [核心库说明](src/hexapod_core/README.md)
- [v1 架构](docs/v1-architecture.md)
- [Windows VS Code 编辑与同步](docs/windows-vscode-workflow.md)

源码位于 src/hexapod_core，构建产物位于 build/、install/、log/。

2026-11-03 当前本地进度：我已实现 tripod_phase.hpp/.cpp 并加入核心库 CMake，按既定两组三脚返回计划支撑/摆动阶段和阶段内部进度。我请助手新增正式相位用例及 CTest 登记，Windows 严格构建和完整 CTest 12/12 通过，覆盖 32768 个整数时间表样本、支撑时长、半周期互换、浮点边界及摆动曲线连接。证据位于 build/validation-tripod-phase-20261103/。相位模块尚未生成完整步态或真实触地判断；下一步为规划机身位姿与脚点坐标变换。本轮未连接或同步 Pi，Pi 保留 2026-11-02 的 11/11 验证，最新同步基线仍为 2026-11-02；未执行实机动作。

2026-11-02 双端部署补充：我请助手补全预览程序 CMake，检查并同步到 Pi。预览目录已从误拼的 exemples 更正为 examples，源码内容未改；新增独立构建目标、ROS 包安装入口和 Python 3 CLI 正式测试。Windows 与 Pi aarch64 均构建及 CTest 11/11 通过；Pi 通过 ros2 run 运行安装后的纯离线预览，20 行、25 列 CSV 与 Windows 数值全部一致。同步先比较旧基线与双端哈希，无冲突和删除项，备份后更新 34 个源码/文档文件，再同步本轮记录；最新同步基线为 2026-11-02。未打开舵机串口或执行动作，详细证据位于 build/deployment-20261102/，见 [验证记录](docs/core-deployment-validation.md)。下方较早记录保留各自日期和当时边界。

2026-11-02 当前本地进度：用户已实现 foot_trajectory.hpp/.cpp，以五次平滑基线和六次抬升曲线生成脚端摆动目标。助手新增正式 foot_trajectory_tests.cpp，Windows 严格构建及 CTest 10/10 通过。正式测试已连通 20 段右中腿摆动与六腿段规划器，其余 15 轴目标不变，名义 FK 与理想脚点最大差约 0.392578 mm；这是选定离线用例的量化模型结果，不是实机精度。可运行的单腿预览入口、完整步态及执行层继续推进。本轮未连接或同步 Pi，未执行动作。

2026-11-01 六腿段组合补充：用户已实现 body_segment_planner.hpp/.cpp，并将 require_path_constraints() 开放为共享校验函数。助手补上遗漏的 CMake 核心源文件登记，新增正式 body_segment_tests.cpp，Windows 严格构建及 CTest 9/9 通过。只有六腿目标选择和到名义目标的路径检查全部通过，才返回完整 solution；失败保留选择或路径诊断，不推进输入参考。下一步为脚端摆动轨迹生成。本轮未连接或同步 Pi，未执行动作；下方 8/8 是同日较早的单腿检查器记录。

2026-11-01 当前本地进度：用户已写入 leg_path_checker.hpp/.cpp，助手补上遗漏的核心库 CMake 源文件登记和正式 path_checker_tests.cpp。Windows 严格构建及 CTest 8/8 通过，覆盖限位、显式平面距离、倾斜平面、首次失败字段、非法输入和离散采样遗漏反例。当前通过仅针对固定机身参考、固定平面和线性关节插值的离散检查；六腿段整体规划、步态和实机执行仍待推进。本轮未连接或同步 Pi，未执行动作。

2026-10-31 路径采样补充：用户继续实现 leg_path_sampler.hpp/.cpp，按 N 个区间返回 N+1 条比例/插值角/机身脚点记录。助手新增正式 path_sampler_tests.cpp 并接入 CTest，Windows 严格构建及 CTest 7/7 通过。采样器尚不进行路径约束放行；下一步为带显式参考平面的单腿路径检查器。本轮未连接或同步 Pi，未执行动作；下方 6/6 为同日较早的插值回归记录。

2026-10-31 当前本地进度：用户已写入 joint_interpolation.hpp/.cpp，实现有限角度端点之间的普通线性插值。助手新增正式 interpolation_tests.cpp 并接入 CTest，Windows 严格构建及 CTest 6/6 通过。测试包含独立分数点、端点、非法输入、极大有限数值，以及关节中点脚端低于两个端点的离线 FK 反例。插值函数未加入限位或路径检查；下一步为单腿路径采样。此次未同步 Pi 或执行动作，当前成功同步基线仍为 2026-10-25。

2026-10-30 当前本地进度：已实现机身坐标单腿 IK 包装、单腿候选选择与可选平均速度约束，以及六腿整体求解 body_ik_solver。六腿全部成功才返回完整理想角、名义位置、名义参考角和分支；本层 commanded_positions 保持无值，执行层旁路及整段轨迹仍待实现。助手补充正式 connection_tests.cpp 和 body_ik_tests.cpp，Windows 严格构建及 CTest 5/5 通过。本轮只读 SSH 核对树莓派：23 个已核对文件均与 2026-10-25 同步基线一致，硬件配置一致；没有上传本地源码、在 Pi 上运行新版本测试或执行动作。下方较早日期的进度为历史记录，当前验证证据见 [验证记录](docs/core-deployment-validation.md)。

2026-10-27 目标调整：V1 暂不实现读取关节角后的闭环误差调节，仅预留扩展接口；原到位标定与补偿实现后移，不作为步态开发前置条件。位置读取和角度解码仍用于初始化、按需诊断与未来扩展。首次运动需要有效反馈或明确确认的初始姿态，后续规划参考不冒充实测角度。具体边界见 [v1 架构第 6 节](docs/v1-architecture.md#6-关节误差调节仅预留接口)。

2026-10-25 已在单腿局部/机身 FK 和双向坐标变换基础上实现局部 IK 离散候选：最多四套正负径向/膝分支，经 FK 残差校验与周期去重；伸直、反折候选可成功返回并带 knee_near_singular 标记，自由角状态返回 singular 且候选为空。尚未实现舵机限位筛选、连续候选选择、机身 IK 包装接口或步态。正式 CTest 保留 model_and_mapping，新增 leg_inverse_kinematics；本版已完成双端同步，Windows 和树莓派 aarch64 构建及 CTest 2/2 均通过，结果见[验证记录](docs/core-deployment-validation.md)。

2026-10-23 的 FK/坐标变换版本已双端同步：Windows 严格编译和离线测试通过，树莓派按 scripts/build_core.sh 单任务低内存构建安装通过，CTest 1/1 通过，0 errors / 0 failures / 0 skipped。当前金属端理想触点仍为 (0,−112.53,0) mm，用户明确实物未安装胶垫。未打开舵机串口或执行实机动作。

Windows 打开上一级目录的 `hexapod.code-workspace`；本地 `.vscode/` 已指定 C++17、项目头文件和 JSON 头文件路径，并使用已有 GCC 查询标准库。真实错误提示继续启用，CMake 不在打开工程时自动配置。Windows 可使用已有 GCC 进行离线编译测试；build/install/log 为各端独立产物目录，不同步树莓派二进制，也不要求 Windows 具备相同 ROS/ARM 环境。
JSON 依赖位于 dependencies/json-3.11.3；其他三个规划包目前为空目录。
构建与离线测试不连接串口，也不自动驱动舵机。

## 构建与验证

运行 `./scripts/build_core.sh` 完成单任务、低内存构建与离线测试。2026-10-17 树莓派构建安装及 CTest（1/1）通过；未驱动舵机。详情见 [验证记录](docs/core-deployment-validation.md)。

长期测试源码放在包内 test/，可复用脚本放在 scripts/。测试结束后定期核对并清理过期临时副本、一次性脚本与二进制；build/ 仅保留产物、生成文件及必要日志，不创建自动定时删除任务。
