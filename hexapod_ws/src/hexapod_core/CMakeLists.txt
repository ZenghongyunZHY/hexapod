cmake_minimum_required(VERSION 3.16)
project(hexapod_core VERSION 0.1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 核心库不依赖 ROS。ROS 环境提供 ament 导出；普通电脑也能独立编译测试。
find_package(ament_cmake QUIET)
find_package(nlohmann_json 3.10 REQUIRED)

add_library(hexapod_core src/robot_model.cpp src/joint_mapper.cpp src/leg_kinematics.cpp src/leg_candidate_selector.cpp src/body_ik_solver.cpp
            src/joint_interpolation.cpp src/leg_path_sampler.cpp src/leg_path_checker.cpp src/body_segment_planner.cpp src/foot_trajectory.cpp
            src/tripod_phase.cpp)
target_compile_features(hexapod_core PUBLIC cxx_std_17)
target_include_directories(hexapod_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(hexapod_core PRIVATE nlohmann_json::nlohmann_json)
if(MSVC)
  target_compile_options(hexapod_core PRIVATE /W4 /utf-8)
else()
  target_compile_options(hexapod_core PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

# 纯离线预览入口：只加载配置、规划和输出 CSV。
add_executable(single_leg_swing_preview examples/single_leg_swing_preview.cpp)
target_link_libraries(single_leg_swing_preview PRIVATE hexapod_core)
if(MSVC)
  target_compile_options(single_leg_swing_preview PRIVATE /W4 /utf-8)
else()
  target_compile_options(single_leg_swing_preview PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

include(CTest)
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(core_tests test/core_tests.cpp)
  target_link_libraries(core_tests PRIVATE hexapod_core nlohmann_json::nlohmann_json)
  add_executable(ik_tests test/ik_tests.cpp)
  target_link_libraries(ik_tests PRIVATE hexapod_core nlohmann_json::nlohmann_json)
  add_executable(selection_tests test/selection_tests.cpp)
  target_link_libraries(selection_tests PRIVATE hexapod_core)
  add_executable(connection_tests test/connection_tests.cpp)
  target_link_libraries(connection_tests PRIVATE hexapod_core)
  add_executable(body_ik_tests test/body_ik_tests.cpp)
  target_link_libraries(body_ik_tests PRIVATE hexapod_core nlohmann_json::nlohmann_json)
  add_executable(interpolation_tests test/interpolation_tests.cpp)
  target_link_libraries(interpolation_tests PRIVATE hexapod_core)
  add_executable(path_sampler_tests test/path_sampler_tests.cpp)
  target_link_libraries(path_sampler_tests PRIVATE hexapod_core nlohmann_json::nlohmann_json)
  add_executable(path_checker_tests test/path_checker_tests.cpp)
  target_link_libraries(path_checker_tests PRIVATE hexapod_core nlohmann_json::nlohmann_json)
  add_executable(body_segment_tests test/body_segment_tests.cpp)
  target_link_libraries(body_segment_tests PRIVATE hexapod_core)
  add_executable(foot_trajectory_tests test/foot_trajectory_tests.cpp)
  target_link_libraries(foot_trajectory_tests PRIVATE hexapod_core)
  add_executable(tripod_phase_tests test/tripod_phase_tests.cpp)
  target_link_libraries(tripod_phase_tests PRIVATE hexapod_core)
  if(MSVC)
    target_compile_options(core_tests PRIVATE /W4 /utf-8)
    target_compile_options(ik_tests PRIVATE /W4 /utf-8)
    target_compile_options(selection_tests PRIVATE /W4 /utf-8)
    target_compile_options(connection_tests PRIVATE /W4 /utf-8)
    target_compile_options(body_ik_tests PRIVATE /W4 /utf-8)
    target_compile_options(interpolation_tests PRIVATE /W4 /utf-8)
    target_compile_options(path_sampler_tests PRIVATE /W4 /utf-8)
    target_compile_options(path_checker_tests PRIVATE /W4 /utf-8)
    target_compile_options(body_segment_tests PRIVATE /W4 /utf-8)
    target_compile_options(foot_trajectory_tests PRIVATE /W4 /utf-8)
    target_compile_options(tripod_phase_tests PRIVATE /W4 /utf-8)
  else()
    target_compile_options(core_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_options(ik_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_options(selection_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_options(connection_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_options(body_ik_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_options(interpolation_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_options(path_sampler_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_options(path_checker_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_options(body_segment_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_options(foot_trajectory_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_compile_options(tripod_phase_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
  # 工作区集中保存物理配置；也允许调用方显式指定其他只读测试配置。
  set(hardware_config_path "${CMAKE_CURRENT_SOURCE_DIR}/../../config/robot_hardware.json"
    CACHE FILEPATH "Physical hardware configuration used by core_tests")
  add_test(NAME model_and_mapping
    COMMAND core_tests "${hardware_config_path}")
  add_test(NAME leg_inverse_kinematics
    COMMAND ik_tests "${hardware_config_path}")
  add_test(NAME leg_candidate_selection
    COMMAND selection_tests "${hardware_config_path}")
  add_test(NAME leg_connection_constraints
    COMMAND connection_tests "${hardware_config_path}")
  add_test(NAME body_inverse_kinematics
    COMMAND body_ik_tests "${hardware_config_path}")
  add_test(NAME joint_interpolation
    COMMAND interpolation_tests "${hardware_config_path}")
  add_test(NAME leg_path_sampling
    COMMAND path_sampler_tests "${hardware_config_path}")
  add_test(NAME leg_path_constraints
    COMMAND path_checker_tests "${hardware_config_path}")
  add_test(NAME body_segment_planning
    COMMAND body_segment_tests "${hardware_config_path}")
  add_test(NAME foot_swing_trajectory
    COMMAND foot_trajectory_tests "${hardware_config_path}")
  add_test(NAME tripod_gait_phase
    COMMAND tripod_phase_tests)
  add_test(NAME single_leg_swing_preview_cli
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/preview_cli_tests.py"
      "$<TARGET_FILE:single_leg_swing_preview>" "${hardware_config_path}")
  set_tests_properties(single_leg_swing_preview_cli PROPERTIES TIMEOUT 30)
endif()

install(TARGETS hexapod_core EXPORT hexapod_core_targets
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS single_leg_swing_preview RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(EXPORT hexapod_core_targets
  NAMESPACE hexapod_core:: DESTINATION share/${PROJECT_NAME}/cmake)
if(ament_cmake_FOUND)
  ament_export_targets(hexapod_core_targets HAS_LIBRARY_TARGET)
  ament_export_dependencies(nlohmann_json)
  ament_package()
endif()
