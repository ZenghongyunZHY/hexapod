# hexapod_core：物理模型、关节角转换与单腿运动学

截至 2026-11-03，已实现 types.hpp、robot_model、joint_mapper、leg_kinematics 的局部/机身 FK、双向坐标变换及局部/机身 IK 候选，leg_candidate_selector 的映射过滤、分支/奇异策略选择和可选名义平均速度检查，body_ik_solver 的六腿整体求解，单腿关节角线性插值、机身脚端路径采样和限位/平面距离离散检查，body_segment_planner 的六腿段组合规划，脚端摆动目标曲线、可运行离线预览及 tripod_phase 交替三脚相位采样。最新 Windows 严格构建及 CTest 12/12 通过；Pi 保留 2026-11-02 版本及 CTest 11/11 的验证，本地相位新增尚未同步。最新同步基线仍为 2026-11-02，未执行实机动作。使用 C++17，自定义类、函数、变量与文件名统一使用下划线命名；代码中提供中文注释。

这些都是项目自定义代码，不是 ROS 2 自带工具：`.hpp` 是接口头文件，`.cpp` 是实现文件。核心代码可以脱离 ROS 编译；在 ROS 2 环境下通过 ament/colcon 构建。

## 文件职责

| 文件 | 内容 |
| --- | --- |
| include/hexapod_core/types.hpp | 六腿/三关节枚举、物理 ID、原生位置、单位换算、可选物理参数、目标与回读容器 |
| include/hexapod_core/robot_model.hpp | 只读配置模型的接口 |
| src/robot_model.cpp | 加载 schema 2 JSON，校验编号、单位、可选标定及几何信息 |
| include/hexapod_core/joint_mapper.hpp | 名义关节角与舵机位置转换接口 |
| src/joint_mapper.cpp | 转换公式、参数就绪检查、取整与限位检查 |
| include/hexapod_core/leg_kinematics.hpp | 三关节角类型、局部/机身 FK、双向坐标变换与局部 IK 候选/状态接口 |
| src/leg_kinematics.cpp | V1 理想模型前置检查、相对膝角 FK、安装坐标变换、局部 IK 几何候选枚举；不执行硬件动作 |
| test/core_tests.cpp | 使用真实基准及独立合成标定进行测试 |
| test/ik_tests.cpp | 局部 IK 的独立零位预期、FK/IK 往返、分支、奇异状态与非法输入测试 |
| include/hexapod_core/leg_candidate_selector.hpp、src/leg_candidate_selector.cpp | 单腿候选映射、限位过滤、策略选择与可选名义平均速度检查 |
| include/hexapod_core/body_ik_solver.hpp、src/body_ik_solver.cpp | 六腿请求、共同段时长与逐轴速度预算、完整目标拼装及首个失败返回 |
| test/selection_tests.cpp | 单腿映射、分支策略、奇异附近记忆与候选选择测试 |
| test/connection_tests.cpp | 平均速度等号边界、非法请求、其他候选回退与整数名义角检查 |
| test/body_ik_tests.cpp | 机身 IK、18 轴目标、失败不暴露部分解、速度传播及非法输入测试 |
| include/hexapod_core/joint_interpolation.hpp、src/joint_interpolation.cpp | 普通关节角线性插值，检查比例及角度有限性；不检查限位、路径或设备能力 |
| test/interpolation_tests.cpp | 端点/分数点、反向/静止、非法输入、浮点边界与脚端中途下探反例 |
| include/hexapod_core/leg_path_sampler.hpp、src/leg_path_sampler.cpp | 均匀比例采样线性关节段，返回比例、连续插值角和机身坐标脚点 |
| test/path_sampler_tests.cpp | 点数/顺序、独立六腿圆弧预期、组合关节段、静止/反向、中途下探与非法输入 |
| include/hexapod_core/leg_path_checker.hpp、src/leg_path_checker.cpp | 显式参考平面、最小离面距离、逐点安装限位和首次失败诊断 |
| test/path_checker_tests.cpp | 距离/限位边界、倾斜平面、首中末失败、字段一致性、配置缺失及数值溢出 |
| include/hexapod_core/body_segment_planner.hpp、src/body_segment_planner.cpp | 组合六腿目标选择和到名义角的逐腿路径检查，全部通过才返回完整目标 |
| test/body_segment_tests.cpp | 成功结果、两阶段失败分离、逐腿约束、末腿拒绝、名义终点与整份输入校验 |
| include/hexapod_core/foot_trajectory.hpp、src/foot_trajectory.cpp | 同一显式规划参考中的五次平滑基线与六次抬升曲线，精确端点和输入校验 |
| test/foot_trajectory_tests.cpp | 独立分数点、端点平缓性、刚体坐标变换、数值边界及 20 段单腿离线集成 |
| examples/single_leg_swing_preview.cpp | 独立纯离线预览入口，全部段规划成功后输出理想/名义脚点和 18 轴位置 CSV |
| test/preview_cli_tests.py | 调用真实可执行文件检查 20×25 CSV、15 固定轴、几何/时间和失败时无 CSV |
| include/hexapod_core/tripod_phase.hpp、src/tripod_phase.cpp | 交替三脚分组、支撑/摆动阶段及阶段进度的无状态采样 |
| test/tripod_phase_tests.cpp | 整数时间表、分组/支撑时长、半周期互换、浮点边界、非法输入和摆动曲线连接 |

## 三脚相位本地进度（2026-11-03）

用户已实现 tripod_phase.hpp/.cpp 并登记核心源文件。sample_tripod_phase() 接收有限的周期进度 [0,1) 与支撑占空比 [0.5,1)，按 leg_order 返回六腿计划阶段和阶段内部进度。A 组为右前、左中、右后，B 组偏移半周期；B 组直接在原始周期中判断边界，避免相加舍入造成提前摆动或提前回绕。阶段切换归入新阶段且进度为 0，浮点舍入可能使临近末尾的阶段进度为 1。

助手新增正式 tripod_phase_tests.cpp 和 CTest 登记。Windows GCC/C++17 严格构建及完整 CTest 12/12 通过、0 failures，用时约 1.95 秒。测试包含 32768 个整数时间表样本、每腿支撑时长、双支撑窗口、半周期互换、nextafter 边界、非法请求、无状态采样及摆动中点调用。生产输入 25 个文件测试前后哈希一致；证据位于 build/validation-tripod-phase-20261103/。

本模块不保存阶段事件或完整周期数，不生成六腿脚点，不证明真实触地、静态稳定或可执行步态。下一步教学为规划机身位姿及脚点坐标变换。本轮未连接或同步 Pi，最新成功同步基线仍为 2026-11-02；未执行实机动作。以下较早章节保留对应日期的历史状态。

## 单腿离线预览与部署（2026-11-02）

single_leg_swing_preview 独立于 BUILD_TESTING 构建，以与核心库相同的严格警告选项编译；安装到 lib/hexapod_core，供 ROS 包入口调用。原目录误拼 exemples 已更正为 examples，文件内容 SHA-256 保持 d7c7a07a8fa3c8e6c3df111d0932830b1ea90aeb085b72ec875bc5482a5c57e7。输入是显式离线候选姿态与开发参数，程序不读取舵机或发送动作。

树莓派构建与离线测试仍使用 scripts/build_core.sh，显式 MAKEFLAGS=-j1。安装后可运行：

```bash
source /opt/ros/jazzy/setup.bash
source /home/hexapod/hexapod_ws/install/setup.bash
ros2 run hexapod_core single_leg_swing_preview \
  /home/hexapod/hexapod_ws/config/robot_hardware.json \
  > /home/hexapod/hexapod_ws/build/single_leg_swing_preview.csv
```

Windows 对应可执行文件为 build/hexapod_core_standalone/single_leg_swing_preview.exe，传入 config/robot_hardware.json 的实际路径。程序完整规划后才输出 20 行段终点记录：规划时间、理想脚点、名义 FK 脚点及 p1～p18，共 25 列；诊断写入 stderr。数据行从 0.1 s 到 2.0 s，不另输出 0 s 初始参考。Python 3 仅用于新增 CLI 测试，已在 BUILD_TESTING 中明确查找并在 package.xml 声明 test_depend。

本轮 Windows/Pi CTest 均 11/11，Pi 安装后 CLI 验证通过，两端 CSV 忽略换行形式后文本一致，数值最大差 0。原始模型回归最大名义 FK 误差约 0.392578 mm；CSV 六位小数再计算为约 0.391922 mm，二者精度层次不同，均不是实测精度。审计日志、CSV、哈希与三方比较记录位于工作区 build/deployment-20261102/；同步前有旧源码备份，未同步构建产物或驱动舵机。以下较早章节保留当时阶段状态。

## 脚端摆动目标进度（2026-11-02）

sample_swing_trajectory() 使用 u(s)=10s^3-15s^4+6s^5 和 b(s)=64s^3(1-s)^3，返回 (1-u)start+u*end+H*b*n；三个向量须位于同一调用方明确选定的参考系。H 是相对平滑基线沿单位方向 n 的最大额外偏移，不自动解释为真实离地高度。函数检查完整请求后精确返回端点，拒绝非法比例、非有限值、非单位方向、负抬升量及不可表示的结果。曲线尚不包含坐标变换、IK 或执行逻辑。

新增正式测试后 Windows CTest 10/10 通过。数学用例覆盖独立四分之一点、不同方向/端点高度、零抬升、原地抬升、反向对称、刚体坐标变换、端点附近三阶位移缩小和非法输入。集成用例以显式候选姿态 (0,12,-72) 度为初始名义姿态，右中腿沿机身 X 前移 10 mm、抬升 8 mm，2 s 分为 20 段，每段内部检查 4 区间，各轴平均速度预算 1 rad/s；合成参考平面比初始脚点低 2 mm。20 段全部通过，其余 15 轴位置保持不变，名义 FK 与理想脚点最大差约 0.392578 mm。上述数据仅为离线回归参数和模型结果，不是默认站姿、实际初始反馈、地面标定、机械速度极限或实机精度；预览程序仍待实现。证据为 build/hexapod_core_standalone/ctest-foot-trajectory-20261102.log。

## 六腿运动段组合进度（2026-11-01，单腿检查之后）

body_segment_planner::plan() 先校验区间数、共享平面及六腿最小距离，再调用 body_ik_solver；选解成功后，从各腿请求参考角连接到整数位置解码后的名义目标角，依次调用 leg_path_checker。只有全部通过才转交完整 solution，选择失败和路径失败保留不同诊断字段；不修改输入参考/历史分支，也不创建执行命令。

require_path_constraints() 已由用户移出匿名命名空间并在头文件声明，供组合层复用。助手补上遗漏的 CMake 源文件登记并新增 body_segment_tests.cpp，Windows CTest 9/9 通过。正式用例验证每腿选择/距离失败、速度请求传播、末腿中点下探、完整失败字段、输入不变、后续名义参考复用，以及最终名义角而非理想角参与路径检查。当前方案只检查已选候选，路径失败时不回溯搜索全部 IK 组合；固定参考平面和有限采样的边界保持不变。证据为 build/hexapod_core_standalone/ctest-body-segment-20261101.log。本轮未修改用户功能实现，未连接或同步 Pi，未执行动作。

## 路径约束检查进度（2026-11-01）

leg_path_checker 已实现：先校验约束与当前腿命令配置，生成完整采样，再按顺序检查各点的三个连续关节角和显式参考平面距离。采样点通过返回 samples_passed；失败只返回首个失败点，checked_sample_count 包含失败点，关节失败与距离失败保留不同诊断字段。计算非法或配置缺失继续抛异常，不混成普通约束失败。当前法向量长度采用 1e-9 数值容差，关节比较沿用 1e-12 rad 数值容差；它们不是机械余量，距离比较没有新增宽限。

助手补上遗漏的 CMake 核心源文件登记并新增正式 path_checker_tests.cpp，Windows CTest 8/8 通过。测试保留同一运动段只取端点通过、加中点后失败的反例，不能将离散通过表述为连续路径证明；显式平面不代表已经测准实际地面。当前只覆盖固定机身参考、固定平面和线性关节插值模型，不包含碰撞、真实触地、机身运动或实机执行验证。日志为 build/hexapod_core_standalone/ctest-path-checker-20261101.log。本轮未修改用户功能实现或物理参数，未连接或同步 Pi。

## 路径采样进度（2026-10-31，插值回归之后）

leg_path_sampler::sample_linear_segment() 接收指定腿、起止名义角及区间数 N，返回 N+1 条记录，包含精确端点比例。函数先验证输入角度和该腿 FK 所需安装信息，再预留空间；零区间和不可能的容器容量明确报错。内部连续插值角不重新取整为命令，输出脚点在机身坐标系表达。

助手新增正式 path_sampler_tests.cpp 后 Windows CTest 7/7 通过。独立 CAD 根部/安装角与零位总腿长构成六腿圆弧预期，不依赖生产坐标变换生成预期；另覆盖组合关节角分数点、1/2/4/17/32 区间、静止和反向路径、输入不变、中途脚端下探、非法编号/非有限角/缺失安装及容量边界。采样器当前只计算，不进行安装限位或地面约束放行；路径检查器、参考平面、离地余量和连续段保证仍待实现。证据为 build/hexapod_core_standalone/ctest-path-sampler-20261031.log。本轮未修改用户功能源码，未连接或同步 Pi，未执行动作。

## 关节插值进度（2026-10-31）

interpolate_leg_joint_angles() 接收起止关节角与 [0,1] 比例，先校验全部输入，再返回端点或加权和；不进行整圈归一化、夹紧或整数位置重新量化。当前线性插值只是离线采用的明确模型，不表示控制板实际插值行为已验证。新增正式测试后 Windows CTest 6/6 通过，测试与功能源码直接链接，具体日志为 build/hexapod_core_standalone/ctest-20261031.log。

离线反例使用局部角度从 (0,-30,-30) 度到 (0,-30,-86.4) 度，按当前 CAD 模型计算脚端局部 Z 起点/中点/终点约为 -134.554/-149.574/-137.894 mm。它说明中间脚点可能低于两个端点，不是当前站姿、地面高度或实机测试结果。单腿路径采样、地面坐标处理、离地余量与连续段检查仍待实现；不能把有限个采样点通过视为完整连续路径的严格证明。本轮没有修改用户功能源码或硬件参数，也未同步 Pi 或执行动作。

## 当前六腿求解边界（2026-10-30）

body_ik_solver 按 leg_order 处理六腿，通过 robot_model 查询物理 ID 后拼装 joint_targets。六腿全部成功才设置 solution；首个单腿失败返回 failed_leg 和 failure_reason，没有部分 solution。返回的 nominal_angles/branches 只在上层接受整帧后作为后续规划参考，不自动改写输入或伪装成实测姿态。

本层只填写 desired_angles_rad 与 nominal_positions，commanded_positions 保持无值，执行层显式旁路尚待实现。共同段时长与全部 18 个速度上限先校验，逐腿选解时再检查量化后的名义角差。通过表示当前端点/候选规则和可选平均速度预算成立；整段插值路径、峰值速度、加速度、碰撞、实际执行时序与到位仍未验证。

本轮只在 Windows 增加正式测试与 CMake 测试接入，没有修改用户功能源码或硬件配置。现有较早章节中的“待实现”保留其日期语境，当前状态以本节及工作区验证记录为准。Pi 的 23 个已核对文件与 2026-10-25 基线一致，本轮没有上传新代码或运行 Pi 新版本测试。

## 统一名称（2026-10-19）

| 名称 | 代码名称 | 含义 |
| --- | --- | --- |
| 舵机位置值 | servo_position | 协议整数；原生范围 0～1000，当前命令限幅 140～860 |
| 舵机相对角 | servo_relative_angle_rad | 相对参考位置值的转角，位置值增大为正；当前参考位置为 500 |
| 关节角 | joint_angle_rad | 运动学使用的角度，按该关节约定的数学正轴确定正负 |
| 关节参考角 | reference_joint_angle_rad | 舵机处于参考位置值时的关节角；当前全部为 0 |

角度名的 _rad 表示弧度，配置名的 _deg 表示度。方向符号 servo_to_joint_sign 只取 ±1，连接舵机相对角与关节角。

```text
舵机相对角 = (舵机位置值 − 参考位置值) × 每位置单位弧度数
关节角 = 关节参考角 + 方向符号 × 舵机相对角
```

同样输入位置 550，舵机相对角均为 +12°；右侧大腿关节角为 +12°，左侧大腿关节角为 −12°。两个角度计算结果均以弧度返回。它们是位置值的名义换算，是否来自目标或回读由调用方决定。

三个接口统一为 servo_position_to_servo_relative_angle_rad()、servo_position_to_joint_angle_rad()、joint_angle_rad_to_servo_position()。旧 C++ 函数名已移除，当前工作区调用处同步更新。

## 读取现有配置

程序显式传入仓库 config/robot_hardware.json 的路径，不依赖当前运行目录或复制另一份硬件参数。以下代码为库调用示例，不会操作硬件：

```cpp
#include "hexapod_core/joint_mapper.hpp"

// config_path 是调用程序接收的配置路径。
const auto model = hexapod_core::robot_model::from_file(config_path);
const auto& joint = model.joint(hexapod_core::leg_id::right_front,
                                hexapod_core::joint_type::upper_leg_pitch);
const hexapod_core::joint_mapper mapper(model);

// 当前 ID2 使用用户确认的 500 零位与 CAD 推导方向；缺省配置仍不自动补值。
const auto id = joint.binding.id;
if (model.is_mapping_ready(id)) {
  const double joint_angle_rad = mapper.servo_position_to_joint_angle_rad(
      id, hexapod_core::servo_position(500));
  (void)joint_angle_rad;
}

// 舵机相对角：统一以位置值增大为正；还没有应用逐轴方向符号。
const double servo_relative_angle_rad = mapper.servo_position_to_servo_relative_angle_rad(
    hexapod_core::servo_position(550));  // 对当前配置为 +12° 的弧度值
(void)servo_relative_angle_rad;
```

`is_mapping_ready()` 只检查该轴的数学方向与关节参考角是否齐全；`is_command_ready()` 还检查安装限位是否已填写。两者都不表示已完成实机标定或碰撞检查，也不表示 FK/IK 所需的安装信息全部齐全。

当前配置采用用户选择的水平展开零位（500→0°）、18 轴方向、140～860 对应的数学限位 ±86.4°及六腿 CAD 安装变换。2026-10-20 已选择金属端触点，2026-10-21 用户明确确认实物未安装胶垫；当前 lower_leg 触点为 (0,−112.53,0) mm，模长等于 CAD 金属端 L3。方向为偏航全部 +1，右俯仰 +1、左俯仰 −1。原生协议范围仍为 0～1000，实际角度命令受安装限位约束。几何采用 CAD 理想值，状态不升级为实测。完整坐标和来源见工作区 docs/physical-baseline.md。

model.convention() 可读取零位伸展方向 (0,−1,0) 和三个零位数学正轴 [(0,0,1),(−1,0,0),(−1,0,0)]。它们在腿根局部坐标系表达，后级轴随前级关节转动。lower_leg 触点坐标系在第三轴中心，随小腿转动；零位与腿根坐标系同向。金属 L3 和理想触点向量分别保留，当前触点模长等于金属 L3，计算脚端时不能重复叠加。

## 单腿局部正运动学（2026-10-21）

leg_joint_angles 的 yaw_rad、upper_leg_rad、lower_leg_rad 均为数学关节角（弧度）；lower_leg_rad 为小腿相对大腿的角度。forward_in_leg_frame() 输出第一转轴原点下的腿根局部脚端坐标（米），不含机身安装变换。

令 a 为第一段 L2、b 为第二段长度、c 为触点向量的 -Y 分量，则 r=a+b*cos(q2)+c*cos(q2+q3)，输出 (r*sin(q1),-r*cos(q1),b*sin(q2)+c*sin(q2+q3))。第一段不贡献高度，金属 L3 不再次叠加。当前零位脚点为 (0,−0.22873,0) m。

构造时拒绝缺失几何、未知或不支持的零位/轴向约定、错误触点坐标系及非轴向触点。计算时拒绝非有限角度和俯仰和溢出。FK 可用于反馈分析，不代表安装限位、碰撞或轨迹执行检查通过；命令仍由映射及后续约束层审核。

## 腿根与机身坐标变换（2026-10-23）

leg_to_body_frame() 按指定 leg_id 的配置计算 p_body=R*p_leg+t；body_to_leg_frame() 使用 R 的转置计算 p_leg=R^T*(p_body-t)。R 来自 body_to_first_axis.rotation，实际方向为局部到机身；t 已由加载器从毫米转换为米。forward_in_body_frame() 组合局部 FK 与上述安装变换，不重复计入安装角。

对应腿的安装变换缺失时拒绝该腿计算，其他参数完整的腿仍可使用；非法腿枚举与非有限输入明确报错。2026-10-23 新增测试覆盖六腿独立零位/根部预期、双向往返、右前非零 FK、合成三维旋转（不只绕 Z）、逐腿缺参及各分量 NaN/Inf。测试已从 main() 调用，Windows C++17/GCC 严格编译及完整离线测试通过；同日联网后已同步到树莓派，aarch64 构建安装及 CTest 1/1 通过，0 errors / 0 failures / 0 skipped。

可选字段缺省仍保留 std::nullopt，旧 schema 2 配置兼容；加载器不自动推断其他机器人参数。

## 单腿局部逆运动学候选（2026-10-25）

inverse_candidates_in_leg_frame() 接收腿根局部脚点（米），返回 leg_ik_result。solution 最多包含四套离散几何候选：分别检查正、负径向，再枚举正、负膝角；每套候选通过公开 FK 的残差校验，并按角度周期去重，不依赖候选顺序选择姿态。

| status | 含义 |
| --- | --- |
| success | 已枚举离散候选，solution 非空；允许包含伸直或反折的膝边界解 |
| unreachable | 超出几何可达范围，solution 为空 |
| singular | 存在当前接口不能完整枚举的自由角，solution 为空 |

伸直、反折及其附近的离散候选使用 knee_near_singular 标记，仍可返回 success；该标记不自动拒绝候选，也不表示可以直接执行。脚点在可达偏航轴上时 yaw 自由；合成等长两段完全折叠到第二轴时 q2 自由，这些情况返回 singular，不随意填入一个角度。NaN/Inf 坐标抛出 std::invalid_argument；数值容差只处理计算误差，不代表机械精度。

本接口尚未筛选舵机安装限位、碰撞或轨迹连续性，也未实现连续候选选择、机身坐标 IK 包装接口和步态。几何候选可超出实物角度范围，后续约束层必须独立审核。

## 可选标定表的格式约定

以下只说明加载器接受的格式，**不是这台机器的标定参数，不要直接复制进实物配置**。只填了 ID1 不会自动标定其余 ID。

```json
{
  "joint_angle_signs": {"1": 1},
  "joint_reference_angles_deg": {"1": 20.0},
  "installed_joint_limits": {"1": {"min_deg": -100.0, "max_deg": 100.0}}
}
```

这些字段位于 mounting 中，均可整体为 null，也可按 ID 部分填写；单个 ID 的值也允许 null。旧配置键 joint_zero_offsets 仍可读取，统一解释为关节参考角；如果新旧两张表同时出现且内容矛盾，加载时报错。

- joint_angle_signs：位置值增加时关节角的方向，只允许整数 +1 或 -1。
- joint_reference_angles_deg：关节参考角表，数值为 servo.reference_position 处的关节角，单位度；加载到 reference_joint_angle_rad 后使用弧度。
- installed_joint_limits：安装后的关节角范围，单位度；不等于舵机原生 0～240°。

其余可选安装字段也可部分填写：

- leg_root_positions_mm：按 body_role 键保存 [x,y,z]，单位毫米。
- leg_root_axis_directions：按 body_role 键保存机身坐标系中的单位轴向量；不从该字段自动推断其他关节符号。
- fixed_body_to_first_axis_transforms：按 body_role 键保存对象，包含 translation_mm=[x,y,z] 和 rotation=3×3 行优先旋转矩阵。旋转将局部坐标转换到机身坐标，必须正交且右手；若同时填了根部位置，两者平移必须一致。
- body_origin：原点定义的非空文字说明；未知时为 null，不默认在板中心。
- model_geometry.actual_foot_contact_point：明确的 {"frame":"坐标系名称","position_mm":[x,y,z]}；未知时为 null，不能只给一个未定义坐标系的长度。

几何长度保留全局 source / measurement_status；触点对象可额外提供自己的 source / measurement_status，加载后优先保留逐项来源。actual_foot_contact_point 的历史名称不表示已经实测。mounting.kinematic_convention 为可选对象，包含 zero_leg_direction 和按 joint_order 排列的三个 positive_joint_axes_at_zero，均检查为单位向量。后续新增实测数据时须同步更新基准文档和 JSON。

## 转换与异常

内部角度一律是弧度；度只在 JSON 加载和显示边界转换：

```text
servo_relative_angle_rad = (p - p_ref) × radians_per_position_unit
joint_angle_rad = reference_joint_angle_rad + servo_to_joint_sign × servo_relative_angle_rad
p = p_ref + servo_to_joint_sign × (joint_angle_rad - reference_joint_angle_rad) / radians_per_position_unit
```

servo_position_to_joint_angle_rad() 允许解码超出安装角度限位的回读，用于诊断；仍检查原生位置范围。joint_angle_rad_to_servo_position() 要求完整映射和安装限位，同时检查目标角、原生位置，以及取最近整数后的实际角是否越界；不会静默夹紧。

| 情况 | 异常 |
| --- | --- |
| 文件、JSON、schema、单位或参数错误 | std::runtime_error |
| 未确定方向/关节参考角，或发目标时未确定安装限位 | std::logic_error |
| 无效 ID/枚举、原生位置或目标超限 | std::out_of_range |
| 输入 NaN/无穷关节角 | std::invalid_argument |

joint_targets 与 joint_observation 的数组都按物理 ID-1 索引。每个元素默认无值；原始目标、名义位置、准备下发的命令和回读分开存储。它们是数据容器，完整帧验证和发送调度留给后续模块。

2026-10-27 用户调整 V1 目标：基于角度回读的闭环误差调节仅预留接口，具体算法及到位标定/补偿配置后移。当前没有新增调节接口源码；现有 commanded_positions 保留为未来扩展边界，V1 对通过检查的目标使用与 nominal_positions 相同的位置，不以旁路表示误差已标定为零。joint_observation 与位置到角度转换继续保留，供初始化与按需诊断；不要求每帧回读再纠偏。第一帧参考必须有效，后续已接受目标对应的名义角可以作为规划参考，但不是实际观测。

## 构建与测试

JSON 依赖使用 nlohmann_json 3.10 或更高版本。当前树莓派未安装系统开发包，3.11.3 头文件及 CMake 配置集中放在工作区 dependencies/json-3.11.3；不需要管理员权限，不混入项目源码。参考：[官方 CMake 集成说明](https://json.nlohmann.me/integration/cmake/)。

树莓派默认使用已验证的低内存构建入口（该脚本会完成编译、安装、测试和 ROS 包路径检查）：

```bash
cd /home/hexapod/hexapod_ws
./scripts/build_core.sh
```

当前系统总内存约 899 MiB、无交换空间。colcon 会按 CPU 核数自动设置 make 并行参数，因此脚本显式设置 MAKEFLAGS=-j1；只设置 CMAKE_BUILD_PARALLEL_LEVEL 不足以限制该路径。验证使用 Debug / -O0 -g0，并将脚本子进程虚拟内存限制在 512 MiB，避免一次构建耗尽整机内存。该限制不修改系统配置。

使用模型的测试程序接收仓库真实 config/robot_hardware.json 的路径；相位测试不需要模型配置。当前 Windows CTest 共 12 项，覆盖模型/映射、IK、选择器、连接速度、六腿 IK、关节插值、路径采样、路径约束、六腿段、脚端曲线、三脚相位和预览 CLI。CLI 测试通过 Python 3 调用实际构建的程序，不创建影子实现。默认配置路径为源码相对路径 ../../config/robot_hardware.json，可通过 -Dhardware_config_path=/绝对路径 指定；单独复制包时须提供实际配置。

普通 CMake 环境也可构建，不需要安装 ROS：

```bash
cmake -S src/hexapod_core -B build/hexapod_core_standalone -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Debug "-DCMAKE_CXX_FLAGS_DEBUG=-O0 -g0" -Dnlohmann_json_DIR="$HOME/hexapod_ws/dependencies/json-3.11.3/cmake"
cmake --build build/hexapod_core_standalone --parallel 1
ctest --test-dir build/hexapod_core_standalone --output-on-failure
```

测试覆盖真实 18 轴编号、500 零位、140～860 全整数范围往返及两侧边界拒绝、CAD 坐标/旋转方向/触点来源、旧配置未知项、两种符号和非零关节参考角、部分标定、量化越限、原生限位、非法数值、重复 JSON 键与安装变换冲突；FK 覆盖零位、正偏航、相对膝角、负俯仰、配置触点变化、非有限输入及无效模型拒绝。IK 正式测试覆盖独立零位、40 个伸直与 240 个普通姿态往返、近伸直分支、正负径向、多候选、反折周期去重、自由角、不可达与非法数值。合成标定和等长腿模型仅存在于测试内，不写回真实配置。

Windows 编辑目录为 C:/Users/zengh/Documents/ChatGPT/hexapod/hexapod_ws/src/hexapod_core；树莓派目录为 /home/hexapod/hexapod_ws/src/hexapod_core。用户本地修改后明确要求同步时再上传，助手修改时维护双端版本；Windows 可用已有 GCC 进行本机离线编译测试，目标平台构建与验证仍在树莓派进行，不要求两端具备相同 ROS/ARM 环境。Windows 编辑器配置说明见工作区 docs/windows-vscode-workflow.md，部署验证结果见 docs/core-deployment-validation.md。该库及其测试均不打开串口或驱动舵机。

长期保留的测试源码放在 test/，可复用脚本放在工作区 scripts/。测试结束后定期核对并清理过期临时副本、一次性脚本和二进制；build/ 仅保留构建产物、生成文件及必要日志，不以自动定时删除代替核对。

2026-10-25 上述 IK 实现与正式 CTest 入口已完成双端同步，Windows 完整 CMake 构建及 CTest 2/2 通过；树莓派 aarch64 按低内存脚本构建安装成功，包构建约 33.1 秒，CTest 2/2 通过（约 0.58 秒），0 errors / 0 failures / 0 skipped。安装头文件和静态库与本次源码/构建一致，金属端硬件配置哈希未变化。2026-10-23 的 FK/坐标变换版本记录继续保留；以上均为离线验证。

2026-10-17 树莓派验证：aarch64 / GCC 13.3.0 / ROS 2 Jazzy 构建安装通过，CTest 1/1 通过，0 错误/0 失败；ROS 包位于 /home/hexapod/hexapod_ws/install/hexapod_core。未连接舵机串口。

## 2026-10-28：单腿候选选择器 Windows 验证

用户已完成 leg_candidate_selector 的映射、过滤、评分、分支识别/记忆与策略选择。助手负责补充正式 test/selection_tests.cpp，CTest 新增 leg_candidate_selection；Windows GNU 16.2 / C++17 严格构建通过，完整 CTest 3/3 通过。测试覆盖六腿映射、策略差异、失败分类、伸直与量化后的分支记忆、负径向和非法输入，直接链接生产库。

本轮未修改功能实现或硬件配置；记录见工作区 docs/core-deployment-validation.md。当前结果为 Windows 离线验证，选择器和本地修改尚未完成本轮 Pi 同步及 ARM 测试。端点选解成功不代表整段轨迹、碰撞或时间连接检查通过。后续正式测试由助手编写、运行和核对。

## 2026-10-29：平均速度约束与当前请求契约

用户已写入 check_connection() 并在 select_candidate() 的策略筛选后、角距评分前逐套应用约束。外层 optional<leg_connection_request> 表示是否要求检查；一旦提供请求，时长和三个规划速度上限必须完整、有限且为正，内部使用 array<double, joints_per_leg>。不再使用逐轴可选速度和“速度未知”状态；要求检查的模式不能因缺配置而自动省略请求。

连接结果没有默认构造，检查后明确返回 within_average_speed_limits 或 average_speed_limit_exceeded；选解层区分 no_candidate_under_policy 与 no_candidate_within_speed_limits。选解成功时，connection 有值表示所选候选通过本次平均速度预算检查；为空表示本次未要求检查。速度预算由上层提供，不等于已验证的设备能力，检查也不保证整段轨迹或实际到位时间。

当前实现和已有选择器测试的严格语法/警告检查通过，尚未完成新增速度行为的正式回归测试。2026-10-28 的 CTest 3/3 结果对应此前未加入速度检查的版本。请求完整性、结果构造和状态命名的调整理由及主/侧对话决策见 [开发历史与验证记录](../../docs/core-deployment-validation.md)。本次仅整理文档，没有同步 Pi 或执行动作。
