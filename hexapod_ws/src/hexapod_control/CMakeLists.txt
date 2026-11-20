cmake_minimum_required(VERSION 3.16)
project(hexapod_control LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(hexapod_core REQUIRED)
find_package(hexapod_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_library(servo_plan_builder STATIC src/servo_plan_builder.cpp)
target_compile_features(servo_plan_builder PUBLIC cxx_std_17)
target_include_directories(servo_plan_builder PUBLIC src)
target_link_libraries(servo_plan_builder PUBLIC hexapod_core::hexapod_core)
ament_target_dependencies(servo_plan_builder PUBLIC hexapod_interfaces)

add_executable(hexapod_planner src/planner_node.cpp)
target_link_libraries(hexapod_planner PRIVATE servo_plan_builder)
ament_target_dependencies(hexapod_planner PUBLIC rcl_interfaces rclcpp)

if(MSVC)
  target_compile_options(servo_plan_builder PRIVATE /W4 /utf-8)
  target_compile_options(hexapod_planner PRIVATE /W4 /utf-8)
else()
  target_compile_options(servo_plan_builder PRIVATE -Wall -Wextra -Wpedantic -Werror)
  target_compile_options(hexapod_planner PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

include(CTest)
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(servo_plan_builder_tests test/servo_plan_builder_tests.cpp)
  target_link_libraries(servo_plan_builder_tests PRIVATE servo_plan_builder)
  add_test(NAME servo_plan_builder_tests COMMAND servo_plan_builder_tests)
  add_test(NAME planner_node_smoke
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/planner_node_smoke.py"
      "$<TARGET_FILE:hexapod_planner>" "${CMAKE_CURRENT_SOURCE_DIR}/../../config/robot_hardware.json")
  set_tests_properties(planner_node_smoke PROPERTIES TIMEOUT 45)
endif()

install(TARGETS hexapod_planner DESTINATION lib/${PROJECT_NAME})
ament_package()
