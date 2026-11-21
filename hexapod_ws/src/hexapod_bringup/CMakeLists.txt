cmake_minimum_required(VERSION 3.16)
project(hexapod_bringup LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

include(CTest)
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME three_node_preview
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/three_node_preview.py"
      "${CMAKE_CURRENT_SOURCE_DIR}/../../config/robot_hardware.json")
  set_tests_properties(three_node_preview PROPERTIES TIMEOUT 80)
endif()

ament_package()
