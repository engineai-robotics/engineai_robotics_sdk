cmake_minimum_required(VERSION 3.16)
project(robot_session_examples CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(EngineAIRobotSession REQUIRED)
find_package(yaml-cpp REQUIRED)

if(TARGET yaml-cpp::yaml-cpp)
  set(ENGINEAI_EXAMPLES_YAML yaml-cpp::yaml-cpp)
else()
  set(ENGINEAI_EXAMPLES_YAML yaml-cpp)
endif()

function(add_sdk_example name src)
  add_executable(${name} ${src})
  target_link_libraries(${name} PRIVATE EngineAIRobotSession::robot_session)
  target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
endfunction()

# Basic examples: one public entry (or one reader group) per binary.
add_sdk_example(basic_session basic/session.cpp)
add_sdk_example(basic_read_state basic/read_state.cpp)
add_sdk_example(basic_read_diagnostics basic/read_diagnostics.cpp)
add_sdk_example(basic_motion basic/motion.cpp)
add_sdk_example(basic_joint_command basic/joint_command.cpp)
add_sdk_example(basic_joint_motion_planner basic/joint_motion_planner.cpp)
add_sdk_example(basic_body_velocity basic/body_velocity.cpp)
add_sdk_example(basic_joint_override basic/joint_override.cpp)
add_sdk_example(basic_led basic/led.cpp)

# Scenario examples: one program per user task.
add_sdk_example(connectivity_doctor scenarios/connectivity_doctor.cpp)
add_sdk_example(robot_status_snapshot scenarios/robot_status_snapshot.cpp)
add_sdk_example(motion_mode_manager scenarios/motion_mode_manager.cpp)
add_sdk_example(safe_velocity_walk scenarios/safe_velocity_walk.cpp)
add_sdk_example(keyboard_velocity_control scenarios/keyboard_velocity_control.cpp)
add_sdk_example(joint_plan_sequence scenarios/joint_plan_sequence.cpp)
add_sdk_example(interactive_action_trigger scenarios/interactive_action_trigger.cpp)
add_sdk_example(joint_bridge_trajectory scenarios/joint_bridge_trajectory.cpp)
add_sdk_example(upper_body_override scenarios/upper_body_override.cpp)
add_sdk_example(t800pro_hand_gesture_control scenarios/t800pro_hand_gesture_control.cpp)
add_sdk_example(led_control scenarios/led_control.cpp)
add_sdk_example(telemetry_logger scenarios/telemetry_logger.cpp)
add_sdk_example(gamepad_input_monitor scenarios/gamepad_input_monitor.cpp)
add_sdk_example(yaml_motion_config_check scenarios/yaml_motion_config_check.cpp)
add_sdk_example(walk_and_motion_plan scenarios/walk_and_motion_plan.cpp)
target_link_libraries(joint_plan_sequence PRIVATE ${ENGINEAI_EXAMPLES_YAML})
target_link_libraries(interactive_action_trigger PRIVATE ${ENGINEAI_EXAMPLES_YAML})
target_link_libraries(joint_bridge_trajectory PRIVATE ${ENGINEAI_EXAMPLES_YAML})
target_link_libraries(upper_body_override PRIVATE ${ENGINEAI_EXAMPLES_YAML})
target_link_libraries(yaml_motion_config_check PRIVATE ${ENGINEAI_EXAMPLES_YAML})
target_link_libraries(walk_and_motion_plan PRIVATE ${ENGINEAI_EXAMPLES_YAML})

find_path(ENGINEAI_EXAMPLE_EIGEN_INCLUDE Eigen/Core
  PATHS /usr/include/eigen3 /usr/local/include/eigen3
  REQUIRED)
set(ENGINEAI_EXAMPLE_MNN_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mnn)
set(ENGINEAI_EXAMPLE_MNN_LIB ${ENGINEAI_EXAMPLE_MNN_ROOT}/lib/${_ENGINEAI_RS_PLATFORM}/libMNN.so)
if(NOT EXISTS "${ENGINEAI_EXAMPLE_MNN_LIB}")
  message(FATAL_ERROR "libMNN.so for ${_ENGINEAI_RS_PLATFORM} not found: ${ENGINEAI_EXAMPLE_MNN_LIB}")
endif()
add_sdk_example(joint_bridge_stand_and_walk scenarios/joint_bridge_stand_and_walk.cpp)
target_include_directories(joint_bridge_stand_and_walk PRIVATE
  ${ENGINEAI_EXAMPLE_MNN_ROOT}/include
  ${ENGINEAI_EXAMPLE_EIGEN_INCLUDE})
target_link_libraries(joint_bridge_stand_and_walk PRIVATE
  ${ENGINEAI_EXAMPLES_YAML}
  ${ENGINEAI_EXAMPLE_MNN_LIB})
set_target_properties(joint_bridge_stand_and_walk PROPERTIES
  BUILD_RPATH "${ENGINEAI_EXAMPLE_MNN_ROOT}/lib/${_ENGINEAI_RS_PLATFORM}"
  INSTALL_RPATH "${ENGINEAI_EXAMPLE_MNN_ROOT}/lib/${_ENGINEAI_RS_PLATFORM}")
