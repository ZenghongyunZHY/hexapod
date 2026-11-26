# GitHub 仓库文件说明

日期：2026-11-03。

我将项目同步到 GitHub，请助手整理首次提交范围和忽略规则。当前开发工作区为 `hexapod_ws/`，根目录原有配置、物理文档和 `ros2_ws/src/hexapod_demo/` 保留为历史资料。

## 提交范围

- 当前源码、头文件、正式测试、离线示例、CMake 和 ROS 包描述。
- 工作区硬件配置、物理基准、架构与验证记录、可复用构建脚本。
- nlohmann/json 3.11.3 头文件、CMake 适配、版本来源和 MIT 许可证。
- 开发约定、工作区入口、分析脚本、测量 JSON 和结论。公开配图采用经确认的 Pad 手绘示意图或为项目绘制的原创示意图。

## 本机保留

- 两个工作区各自的 `build/`、`install/`、`log/` 和 Python 缓存。
- `.pi_sync_baseline.json`：属于当前 Windows/Pi 配对的同步状态；克隆不能继承这台机器已完成同步的结论。
- `.vscode/`：包含本机编译器路径、个人偏好和 Pi 任务，保留本地文件；克隆后按 `hexapod_ws/docs/windows-vscode-workflow.md` 配置。
- `hexapod_remote/` 中的 Windows 操作台、Pi 端 Python 请求管理工具及其状态文件用于本地联调。
- `references/hardware/` 保留实拍照片、原舵机编号截图及原始标注资料。`outputs/` 下的 PNG 核对图和渲染预览保留在本机。
- `references/cad/` 与 `references/datasheets/` 保留第三方原始资料。
- 环境凭据文件、私钥和本机缓存。

本机文件按 Git 忽略规则保留。`references/manifest.json` 记录原始 CAD、规格书的来源及公开示意图路径。原始硬件配图的来源和哈希随本机资料保存。现有分析脚本保留历史输入路径，重跑 CAD 分析前需要准备原始资料并调整路径。

## 开发与验证边界

GitHub 推送不等于同步树莓派。本机新代码上传 GitHub 后，Pi 仍保留上次显式同步的版本；继续按既有双端哈希基线规则处理同步，不进行自动上传或目录镜像删除。

Windows 可独立构建核心库。Pi 单独验证核心库时使用 `hexapod_ws/scripts/build_core.sh`，并显式设置 `MAKEFLAGS=-j1`。完整 V1 采用 WSL ARM64 交叉编译、Pi 单任务生成 Jazzy 接口及链接安装的流程。构建、离线测试和 Git 操作均不启动舵机动作。 [构建流程](../hexapod_ws/docs/windows-vscode-workflow.md)

正式测试源码随仓库保存。`build/` 下完整原始验证日志仍留在本机，长期结论见工作区 `docs/core-deployment-validation.md`，不能将克隆仓库视为包含全部本机审计记录。

项目自身尚未添加开源许可证；第三方依赖保留原许可证，本次发布不替项目选择授权条款。
