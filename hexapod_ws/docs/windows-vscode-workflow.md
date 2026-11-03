# Windows VS Code 编辑与树莓派构建

日期：2026-10-17；更新：2026-10-25。用户明确更改开发方式，覆盖此前只保留树莓派源码的约定。

## 使用入口

在 Windows 打开项目根目录的 `hexapod.code-workspace`。它打开的本地目录是 `hexapod_ws/`，对应树莓派 `/home/hexapod/hexapod_ws`。

源码、配置、脚本、依赖和说明文件保留双端副本。两端保留相同的相对结构；build/、install/、log/ 各端独立，Windows 可保存本机离线构建产物，不复制 Linux 构建产物。

## 编辑器配置

- 使用电脑已经安装的 Microsoft C/C++ 扩展。
- 使用已有 `C:/msys64/ucrt64/bin/g++.exe` 查询 C++ 标准库位置，不另行安装编译器或 Windows ROS 2。
- `.vscode/c_cpp_properties.json` 指定 C++17、项目 include/ 和 JSON 库 include/。
- `.vscode/settings.json` 保留语法和类型错误提示；关闭本工作区的 CMake 自动配置，由用户或助手显式启动本机离线构建；设置 UTF-8、LF。
- 当前 hexapod_core 只有标准 C++ 和 nlohmann/json 依赖，因此具备完整的本地头文件路径。后续新增 ROS 消息或 Linux 专用代码时，需要按实际新增依赖扩展解析配置，不能把本地无报错当作 ARM/ROS 构建通过。

这些设置服务于编辑、补全和跳转。核心库可使用已有 GCC 在 Windows 独立编译和离线测试，不要求安装相同的 ROS 2 / ARM 环境；目标平台验证仍在树莓派完成。

## 同步与构建

1. 在 Windows 保存文件。
2. 告诉助手“把本地修改同步到 pi”。助手比较同步基线和双端文件，处理冲突、上传并核验哈希。
3. 同步后在树莓派运行 `./scripts/build_core.sh`，或在 VS Code 的“终端 → 运行任务”中选择“树莓派：编译与离线测试（已同步文件）”。

任务只构建树莓派已有文件，不自动上传。没有后台自动同步，也不会因编译执行舵机动作。

助手自己修改代码时，维护两端源码一致。同步范围不包括 Windows 专用 `.vscode/`、各端 build/install/log，也不删除另一端未确认的文件。同步基线保存于项目根目录 `.pi_sync_baseline.json`，记录上次一致的文件哈希，便于发现两端独立修改。

## 测试文件整理

长期测试源码放在包内 test/，可复用脚本放在工作区 scripts/；当前 core_tests 和 ik_tests 分别对应 CTest 的 model_and_mapping 与 leg_inverse_kinematics。build/ 用于构建产物、生成文件及必要日志，不作为长期测试源码存放处。

测试结束后定期核对并清理过期临时副本、一次性脚本和二进制，保留可复现测试入口及必要验证记录。删除前确认用途与双端差异，不进行整个目录镜像删除，也不创建自动定时删除任务。
