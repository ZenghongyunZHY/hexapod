# Windows VS Code 编辑与树莓派构建

日期：2026-10-17。本次用户明确更改开发方式，覆盖此前只保留树莓派源码的约定。

## 使用入口

在 Windows 打开项目根目录的 `hexapod.code-workspace`。它打开的本地目录是 `hexapod_ws/`，对应树莓派 `/home/hexapod/hexapod_ws`。

源码、配置、脚本、依赖和说明文件从树莓派当前版本拉取。两端保留相同的相对结构；Windows 的 build/、install/、log/ 仅保留空目录，不复制 Linux 构建产物。

## 编辑器配置

- 使用电脑已经安装的 Microsoft C/C++ 扩展。
- 使用已有 `C:/msys64/ucrt64/bin/g++.exe` 查询 C++ 标准库位置，不另行安装编译器或 Windows ROS 2。
- `.vscode/c_cpp_properties.json` 指定 C++17、项目 include/ 和 JSON 库 include/。
- `.vscode/settings.json` 保留语法和类型错误提示；关闭本工作区的 CMake 自动配置，防止编辑时尝试构建 Windows 工程；设置 UTF-8、LF。
- 当前 hexapod_core 只有标准 C++ 和 nlohmann/json 依赖，因此具备完整的本地头文件路径。后续新增 ROS 消息或 Linux 专用代码时，需要按实际新增依赖扩展解析配置，不能把本地无报错当作 ARM/ROS 构建通过。

这些设置只服务于编辑、补全和跳转，不把 Windows 模拟成树莓派。真正的目标平台验证仍在树莓派完成。

## 同步与构建

1. 在 Windows 保存文件。
2. 告诉助手“把本地修改同步到 pi”。助手比较同步基线和双端文件，处理冲突、上传并核验哈希。
3. 同步后在树莓派运行 `./scripts/build_core.sh`，或在 VS Code 的“终端 → 运行任务”中选择“树莓派：编译与离线测试（已同步文件）”。

任务只构建树莓派已有文件，不自动上传。没有后台自动同步，也不会因编译执行舵机动作。

助手自己修改代码时，维护两端源码一致。同步范围不包括 Windows 专用 `.vscode/`、各端 build/install/log，也不删除另一端未确认的文件。同步基线保存于项目根目录 `.pi_sync_baseline.json`，记录上次一致的文件哈希，便于发现两端独立修改。
