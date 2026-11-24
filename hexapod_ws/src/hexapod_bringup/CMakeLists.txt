cmake_minimum_required(VERSION 3.16)
project(hexapod_bringup LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

include(CTest)
if(BUILD_TESTING AND UNIX)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME hardware_v1_pty
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/hardware_v1_pty.py"
      "${CMAKE_CURRENT_SOURCE_DIR}/../../config/robot_hardware.json")
  set_tests_properties(hardware_v1_pty PROPERTIES TIMEOUT 85)
endif()

ament_package()
