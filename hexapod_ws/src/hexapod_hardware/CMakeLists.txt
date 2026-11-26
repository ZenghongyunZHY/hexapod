cmake_minimum_required(VERSION 3.16)
project(hexapod_hardware VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake QUIET)
find_package(nlohmann_json 3.10 REQUIRED)
find_package(Threads REQUIRED)

add_library(hexapod_serial_runtime STATIC src/plan_executor.cpp src/serial_runtime.cpp src/servo_controller.cpp)
target_compile_features(hexapod_serial_runtime PUBLIC cxx_std_17)
target_include_directories(hexapod_serial_runtime PUBLIC src)
target_link_libraries(hexapod_serial_runtime PRIVATE nlohmann_json::nlohmann_json Threads::Threads)

function(hexapod_hardware_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /utf-8)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
endfunction()

hexapod_hardware_warnings(hexapod_serial_runtime)

if(ament_cmake_FOUND)
  find_package(diagnostic_msgs REQUIRED)
  find_package(hexapod_core REQUIRED)
  find_package(hexapod_interfaces REQUIRED)
  find_package(rcl_interfaces REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_srvs REQUIRED)

  add_executable(hexapod_serial src/serial_node.cpp)
  target_link_libraries(hexapod_serial PRIVATE hexapod_serial_runtime hexapod_core::hexapod_core)
  ament_target_dependencies(hexapod_serial PUBLIC diagnostic_msgs hexapod_interfaces rcl_interfaces rclcpp sensor_msgs std_srvs)
  hexapod_hardware_warnings(hexapod_serial)
  install(TARGETS hexapod_serial DESTINATION lib/${PROJECT_NAME})
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(plan_executor_tests test/plan_executor_tests.cpp)
  target_link_libraries(plan_executor_tests PRIVATE hexapod_serial_runtime)
  hexapod_hardware_warnings(plan_executor_tests)
  add_test(NAME plan_executor COMMAND plan_executor_tests)

  add_executable(servo_controller_tests test/servo_controller_tests.cpp)
  target_link_libraries(servo_controller_tests PRIVATE hexapod_serial_runtime)
  hexapod_hardware_warnings(servo_controller_tests)
  add_test(NAME servo_protocol COMMAND servo_controller_tests)

  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_executable(serial_runtime_pty_tests test/serial_runtime_pty_tests.cpp)
    target_link_libraries(serial_runtime_pty_tests PRIVATE hexapod_serial_runtime)
    hexapod_hardware_warnings(serial_runtime_pty_tests)
    add_test(NAME serial_pty
      COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/serial_pty_test.py"
        $<TARGET_FILE:servo_controller_tests> $<TARGET_FILE:serial_runtime_pty_tests>
        "${CMAKE_CURRENT_SOURCE_DIR}/../../config/robot_hardware.json")
    set_tests_properties(serial_pty PROPERTIES TIMEOUT 30)
  endif()
endif()

if(ament_cmake_FOUND)
  ament_package()
endif()
