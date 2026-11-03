# 固定帧站立与六周期步态 Demo

两个 C++ 可执行程序：`stand_demo` 和 `walk_demo`，通过 ROS 2 的 ament/colcon 构建、`ros2 run` 启动。演示不启动常驻节点，不使用逆运动学；所有 18 舵机目标值直接读取 `config/poses.csv`。控制板在固定目标之间按指定时间插值。

## 引脚与供电

适用当前树莓派 Pi 4B、幻尔串行总线舵机控制器、18 只 LX-225。

| 树莓派端 | 排针物理脚号 | 控制板端 |
| --- | ---: | --- |
| GPIO14 / TX | 8 | RX |
| GPIO15 / RX | 10 | TX（信号需兼容 3.3V） |
| GND | 6，或其他 GND | GND |

UART 为 `/dev/ttyS0`，9600、8 数据位、无校验、1 停止位。不要误用舵机直连的 115200 协议。

**UART 排针的 5V 不接树莓派。** Pi 使用独立 USB-C 电源，控制板通过其电源口供电，供电电压与 LX-225 的 6～8.4V 要求匹配。不要将 5V 接到 GPIO 信号脚；也不要在两套独立电源之间连接未经确认的 5V 电源引脚。是否需要 RX 电平转换，要根据控制板实际 TX 电平判断；5V 引脚的存在本身不能证明 TX 是 5V 或 3.3V。

参考：[Pi UART 官方说明](https://www.raspberrypi.com/documentation/computers/configuration.html#configuring-uarts)、[控制板官方协议](https://docs.hiwonder.com/projects/Bus-Servo-Controller/en/latest/docs/2.Bus_Servo_Controller_Secondary_Development.html)。

## 当前固定站姿

以全部舵机 500 的展开姿态为参考，用户确认：右侧第二、三段位置增大都向上，左侧都向下。用户反馈初稿站得太矮，要求提高站姿、加大步幅和加快；当前 v2 选择大腿上抬12°、小腿相对大腿下折72°，小腿向地面倾斜60°。按 CAD 腿长估算，相比初稿第二轴到金属脚端的竖直距离增加约21mm；实际机身高度仍需观察。

| 侧别 | 偏航 | 大腿 | 小腿 |
| --- | ---: | ---: | ---: |
| 右侧三腿 | 500 | 550 | 200 |
| 左侧三腿 | 500 | 450 | 800 |

这些角度是与草图形态相符的**可调整初稿**，不是照片测得的精确角度，尚未经过实机站立验证，也不代表测定的机械限位。

抬腿帧：右侧大腿再增加35、小腿再增加15；左侧对应减少，使用8.4°大腿增量与3.6°小腿增量。偏航摆幅±35位置单位（±8.4°），初稿为±20。抬腿/落地300ms、前摆/回中400ms、稳定等待100ms；初稿对应450/650ms。站姿过渡仍用3000ms。完整44帧的含通信预估时间从40.4秒缩短为32秒，实际还受回读和补读影响。偏航前进方向暂依 CAD 中动力轴朝上的安装推定：右侧增大、左侧减小为向前；尚未以实机行走验证。

## 构建与预览

树莓派上的文件只放在 `~/hexapod_ws`：源码在 `src/hexapod_demo/`，其余为标准 `build/`、`install/`、`log/`。不向家目录根部放脚本或压缩包，不自动修改 `.bashrc`。

```bash
cd ~/hexapod_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select hexapod_demo --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash

# 默认预览：不打开串口，不驱动舵机
ros2 run hexapod_demo stand_demo
ros2 run hexapod_demo walk_demo

# 只读联通检查：查询控制板电压和18只舵机位置，不发动作命令
ros2 run hexapod_demo stand_demo --probe
```

## 实机运行

首次站立时支撑好机身，观察新站姿是否与草图相符、各关节是否顺畅，然后再进行地面步态测试。先完成 TX 电平和接线核对。

```bash
source /opt/ros/jazzy/setup.bash
source ~/hexapod_ws/install/setup.bash

# 一次缓慢过渡到屈膝站姿，核对后保持力矩并退出
ros2 run hexapod_demo stand_demo --execute

# 从相同站姿开始，6个完整步态周期，分组回中，最终回到相同站姿
ros2 run hexapod_demo walk_demo --execute
```

没有无限循环参数，不会自动重复试走，也不会下载或启动控制板动作组。一个完整周期包含 A、B 两组三脚各完成一次摆动与落地：

- A：右前（1～3）、左中（13～15）、右后（7～9）。
- B：左前（10～12）、右中（4～6）、左后（16～18）。
- 每周期：A 抬腿 → A 前摆/B 支撑后摆 → A 落地 → B 抬腿 → B 前摆/A 支撑后摆 → B 落地。
- 6 周期共 12 次行走摆动落地。结束后另有两组三脚分别抬起并原位回中，回中时不推动支撑组三脚；这是回到站姿的收尾动作，不是额外行走周期。

## 停止与通信故障

按 **Ctrl+C** 后不再发送后续步态帧；程序尝试读回 18 只舵机的当前位置，再以该位置替换当前动作目标，保留力矩。异常、SSH 挂断和 SIGTERM 同样终止后续帧。

这是有串口往返延迟的软件停止，不是硬急停。读回失败时无法保证取消控制板已收到的当前帧；需要立即停止运动时使用实体电源开关。程序不会在中断时自动返回站姿，也不会自动卸力，以免突然执行额外动作或失去支撑。

本次 demo 按用户于2026-10-14的明确要求，将程序电压放行上限临时放宽至 **8.7V（包含8.7V）**；下限仍为6.0V。**这是软件演示阈值，不是修改 LX-225 手册规定的6～8.4V额定范围。** 该临时设置同时作用于站立和行走两个 demo；测试结束后应恢复8.4V。

启动执行前必须读到 6.0～8.7V 电压和全部18只舵机位置；每帧完成后检查读回位置与目标误差（最多30位置单位）。真实诊断发现控制板偶尔只返回17只，曾缺ID7：现在在原查询期限内，最多针对缺失ID补读两次，读齐后才继续，日志显示 `POSITION_RETRY`。持续缺失、重复ID、错误数据、电压异常和超时仍终止后续动作；不以缓存位置或默认值替代缺失反馈。程序总运行期限90秒。查询控制板的停止动作组指令不能停止即时位置指令，因此程序不把该指令当作急停。

只运行预览、编译或虚拟串口测试，不能证明实物已经成功站立或行走。

## 调整固定姿态

`config/poses.csv` 是执行时唯一的目标位置表，列严格按 ID1～ID18 排列；`config/profile.json` 记录表格的选值依据，不是第二套生效配置。修改站姿后必须同步更新各抬腿、落地和回中帧。重新构建后安装目录中的表格才会更新；也可以通过 `--frames /完整路径/poses.csv` 显式指定调试表。

```bash
cd ~/hexapod_ws
colcon test --packages-select hexapod_demo --event-handlers console_direct+
colcon test-result --verbose
python3 src/hexapod_demo/test/serial_integration.py install/hexapod_demo/lib/hexapod_demo src/hexapod_demo/config/poses.csv
```
