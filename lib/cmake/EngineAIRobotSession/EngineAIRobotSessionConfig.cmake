# CMake package config for RobotSession SDK.
# Layout: <sdk>/lib/cmake/EngineAIRobotSession/

get_filename_component(_ENGINEAI_RS_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../.." ABSOLUTE)
if(NOT EXISTS "${_ENGINEAI_RS_ROOT}/include/engineai_sdk/robot.h")
  message(FATAL_ERROR "EngineAIRobotSession: could not locate SDK root (include/engineai_sdk)")
endif()

include("${CMAKE_CURRENT_LIST_DIR}/EngineAIRobotSessionPlatform.cmake")

engineai_resolve_robot_session_platform(_ENGINEAI_RS_PLATFORM)
set(_ENGINEAI_RS_LIBDIR "${_ENGINEAI_RS_ROOT}/lib/${_ENGINEAI_RS_PLATFORM}")
set(_ENGINEAI_RS_SO "${_ENGINEAI_RS_LIBDIR}/libEngineAIRobotSession.so")
if(NOT EXISTS "${_ENGINEAI_RS_SO}")
  message(FATAL_ERROR
    "EngineAIRobotSession: missing ${_ENGINEAI_RS_SO}\n"
    "Valid platforms: ${ENGINEAI_ROBOT_SESSION_VALID_PLATFORMS}")
endif()

if(NOT TARGET EngineAIRobotSession::robot_session)
  add_library(EngineAIRobotSession::robot_session SHARED IMPORTED)
endif()
set_target_properties(EngineAIRobotSession::robot_session PROPERTIES
  IMPORTED_LOCATION "${_ENGINEAI_RS_SO}"
  IMPORTED_LOCATION_RELEASE "${_ENGINEAI_RS_SO}"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_ENGINEAI_RS_SO}"
  IMPORTED_LOCATION_DEBUG "${_ENGINEAI_RS_SO}"
  INTERFACE_INCLUDE_DIRECTORIES "${_ENGINEAI_RS_ROOT}/include"
)
