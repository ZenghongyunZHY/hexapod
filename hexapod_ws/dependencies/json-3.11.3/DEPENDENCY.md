# 工作区 JSON 依赖

版本：nlohmann/json 3.11.3，MIT。
上游：https://github.com/nlohmann/json/tree/v3.11.3
单头文件源路径：single_include/nlohmann/json.hpp。
下载镜像：https://cdn.jsdelivr.net/gh/nlohmann/json@v3.11.3/single_include/nlohmann/json.hpp
SHA-256：9bea4c8066ef4a1c206b2be5a36302f8926f7fdc6087af5d20b417d0cf103ea6。

include/ 保存上游头文件；cmake/ 是项目准备的最小包发现适配，提供
nlohmann_json::nlohmann_json 接口目标，不是上游完整源码工程。
该依赖只位于本工作区，没有修改系统目录，也不需要 sudo。
若以后安装系统 nlohmann-json3-dev，可改用系统 CMake 包。
