# Shared platform identifiers for RobotSession SDK binaries.

set(ENGINEAI_ROBOT_SESSION_VALID_PLATFORMS
  ubuntu22.04-amd64
  ubuntu22.04-arm64
  ubuntu24.04-arm64
)

function(_engineai_rs_cpu_tag out)
  string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _cpu)
  if(_cpu MATCHES "^(x86_64|amd64)$")
    set(${out} "amd64" PARENT_SCOPE)
  elseif(_cpu MATCHES "^(aarch64|arm64)$")
    set(${out} "arm64" PARENT_SCOPE)
  else()
    set(${out} "" PARENT_SCOPE)
  endif()
endfunction()

function(_engineai_rs_ubuntu_version out)
  set(_ver "")
  if(EXISTS "/etc/os-release")
    file(STRINGS "/etc/os-release" _lines)
    foreach(_line IN LISTS _lines)
      if(_line MATCHES "^VERSION_ID=\"?([0-9]+\\.[0-9]+)\"?")
        set(_ver "${CMAKE_MATCH_1}")
        break()
      endif()
    endforeach()
  endif()
  set(${out} "${_ver}" PARENT_SCOPE)
endfunction()

function(engineai_resolve_robot_session_platform out)
  set(_platform "")
  if(ENGINEAI_ROBOT_SESSION_PLATFORM)
    set(_platform "${ENGINEAI_ROBOT_SESSION_PLATFORM}")
  elseif(DEFINED ENV{ENGINEAI_ROBOT_SESSION_PLATFORM}
      AND NOT "$ENV{ENGINEAI_ROBOT_SESSION_PLATFORM}" STREQUAL "")
    set(_platform "$ENV{ENGINEAI_ROBOT_SESSION_PLATFORM}")
  elseif(NOT CMAKE_CROSSCOMPILING
      AND CMAKE_SYSTEM_PROCESSOR STREQUAL CMAKE_HOST_SYSTEM_PROCESSOR)
    _engineai_rs_cpu_tag(_cpu)
    _engineai_rs_ubuntu_version(_os)
    if(_cpu AND _os)
      set(_platform "ubuntu${_os}-${_cpu}")
    endif()
  endif()

  if(_platform STREQUAL "")
    message(FATAL_ERROR
      "Cannot determine RobotSession platform. Set ENGINEAI_ROBOT_SESSION_PLATFORM to one of: "
      "${ENGINEAI_ROBOT_SESSION_VALID_PLATFORMS}")
  endif()

  list(FIND ENGINEAI_ROBOT_SESSION_VALID_PLATFORMS "${_platform}" _idx)
  if(_idx EQUAL -1)
    message(FATAL_ERROR
      "Unsupported RobotSession platform '${_platform}'. Valid platforms: "
      "${ENGINEAI_ROBOT_SESSION_VALID_PLATFORMS}")
  endif()
  message(STATUS "EngineAIRobotSession platform: ${_platform}")
  set(${out} "${_platform}" PARENT_SCOPE)
endfunction()
