#pragma once

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "engineai_sdk/robot.h"

#include "parse_util.hpp"

namespace robot_session_examples {

inline const char* ErrName(engineai_sdk::SessionError e) {
  using E = engineai_sdk::SessionError;
  switch (e) {
    case E::kOk:
      return "Ok";
    case E::kSessionClosed:
      return "SessionClosed";
    case E::kTransportUnavailable:
      return "TransportUnavailable";
    case E::kPublishFailed:
      return "PublishFailed";
    case E::kInvalidArgument:
      return "InvalidArgument";
  }
  return "Unknown";
}

inline bool WantsHelp(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      return true;
    }
  }
  return false;
}

inline bool ReadStrictInt(const std::string& text, int* out, const char* name) {
  if (!ParseStrictInt(text, out)) {
    std::cerr << name << " must be an integer\n";
    return false;
  }
  return true;
}

inline bool ReadFiniteDouble(const std::string& text, double* out, const char* name) {
  if (!ParseFiniteDouble(text, out)) {
    std::cerr << name << " must be a finite number\n";
    return false;
  }
  return true;
}

inline std::string JsonQuote(const std::string& text) {
  std::string out;
  out.reserve(text.size() + 2);
  out.push_back('"');
  for (unsigned char ch : text) {
    switch (ch) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (ch < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", ch);
          out += buf;
        } else {
          out.push_back(static_cast<char>(ch));
        }
        break;
    }
  }
  out.push_back('"');
  return out;
}

inline std::string JsonFinite(double value) {
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream stream;
  stream.setf(std::ios::fmtflags(0), std::ios::floatfield);
  stream << value;
  return stream.str();
}

// Extract --domain-id / --interface / --network-address from argv; remaining flags stay in *rest.
// Non-empty --interface takes priority over --network-address.
inline bool ParseSessionArgs(int argc, char** argv, engineai_sdk::SessionOptions* options,
                             std::vector<std::string>* rest) {
  bool have_domain = false;
  if (rest) {
    rest->clear();
  }
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--domain-id" && i + 1 < argc) {
      int domain_id = 0;
      if (!ReadStrictInt(argv[++i], &domain_id, "domain-id")) {
        return false;
      }
      options->dds_domain_id = domain_id;
      have_domain = true;
    } else if (arg == "--interface" && i + 1 < argc) {
      options->network_interface_name = argv[++i];
    } else if (arg == "--network-address" && i + 1 < argc) {
      options->network_address = argv[++i];
    } else if (rest) {
      rest->push_back(arg);
    }
  }
  if (!have_domain) {
    std::cerr << "Missing required --domain-id <int> (use the robot ROS_DOMAIN_ID; 0 if unset)\n";
    return false;
  }
  if (options->dds_domain_id < 0) {
    std::cerr << "InvalidArgument: dds_domain_id must be >= 0\n";
    return false;
  }
  return true;
}

inline std::unique_ptr<engineai_sdk::Robot> CreateSessionOrExit(const engineai_sdk::SessionOptions& options) {
  auto result = engineai_sdk::Robot::create(options);
  if (!result) {
    std::cerr << "local session creation failed: " << ErrName(result.error) << "\n";
    std::exit(1);
  }
  return std::move(result.session);
}

inline bool CheckOk(engineai_sdk::SessionError err, const char* what) {
  if (err != engineai_sdk::SessionError::kOk) {
    std::cerr << what << " -> " << ErrName(err) << "\n";
    return false;
  }
  return true;
}

// latest() keeps returning the last sample after the robot stops publishing.
// Freshness comes from last_receive_time(). Thresholds below follow the robot
// publish periods: heartbeat 1 s, motion_state 200 ms, joint/imu 2 ms.
constexpr auto kHeartbeatMaxAge = std::chrono::seconds(3);
constexpr auto kMotionMaxAge = std::chrono::seconds(1);
constexpr auto kStateMaxAge = std::chrono::milliseconds(100);

// Read the time before latest(): the value read afterwards is at least as new,
// so the elapsed time since receive can only be overestimated.
inline bool IsFresh(const std::optional<engineai_sdk::ReceiveTime>& received,
                    std::chrono::steady_clock::duration max_elapsed) {
  return received && std::chrono::steady_clock::now() - *received <= max_elapsed;
}

// Time from last_receive_time() to now. "(no sample)" when nothing has arrived.
inline std::string SinceReceiveText(const std::optional<engineai_sdk::ReceiveTime>& received) {
  if (!received) {
    return "(no sample)";
  }
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - *received);
  return std::to_string(elapsed.count()) + " ms";
}

// The heartbeat is the robot liveness signal.
inline bool RobotOnline(const engineai_sdk::Robot& session) {
  return IsFresh(session.node_heartbeat().last_receive_time(), kHeartbeatMaxAge);
}

inline std::string MotionOrNone(const engineai_sdk::Robot& session) {
  const auto current = session.motion().current_motion();
  return current ? *current : "(no sample)";
}

inline void PrintNoRobotDataHint() {
  std::cerr << "No sample from the robot yet. A successfully created local session "
               "does not mean that a robot was found.\n"
            << "Check: robot is running, --domain-id equals its ROS_DOMAIN_ID "
               "(0 when unset). On the same machine with ROS_LOCALHOST_ONLY=1, "
               "export that variable and do not pass --interface. "
               "Otherwise select this PC's NIC with --interface or --network-address "
               "(ip -br addr).\n";
}

// A stale motion sample does not count: the robot may have left that motion.
inline bool MotionIs(const engineai_sdk::Robot& session, const std::string& name) {
  if (!IsFresh(session.motion().last_receive_time(), kMotionMaxAge)) {
    return false;
  }
  const auto current = session.motion().current_motion();
  return current && *current == name;
}

// Wait for a fresh heartbeat or motion sample before a one-shot command. Do not send if none arrives.
inline bool WaitForRobotSample(engineai_sdk::Robot* session, int timeout_ms = 5000) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (!session->is_open()) {
      return false;
    }
    if (RobotOnline(*session) || IsFresh(session->motion().last_receive_time(), kMotionMaxAge)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

// Pause before close() so this call can still reach the robot.
inline void PauseBeforeClose(int pause_ms = 500) {
  std::this_thread::sleep_for(std::chrono::milliseconds(pause_ms));
}

inline bool WaitForMotionSample(engineai_sdk::Robot* session, int timeout_ms = 5000) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (IsFresh(session->motion().last_receive_time(), kMotionMaxAge) && session->motion().current_motion()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

inline bool RequireCurrentMotion(engineai_sdk::Robot* session, const std::string& name) {
  if (!WaitForMotionSample(session)) {
    PrintNoRobotDataHint();
    return false;
  }
  if (!MotionIs(*session, name)) {
    std::cerr << "current motion=" << MotionOrNone(*session) << ", required=" << name << "\n";
    return false;
  }
  return true;
}

inline bool WaitJointState(engineai_sdk::Robot* session, engineai_sdk::JointState* out,
                           int timeout_ms = 5000) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (IsFresh(session->joint_state().last_receive_time(), kStateMaxAge) && session->joint_state().latest(*out) &&
        !out->position.empty()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

inline volatile std::sig_atomic_t g_stop = 0;

inline void HandleSigInt(int) { g_stop = 1; }

inline void CatchSigInt() {
  g_stop = 0;
  std::signal(SIGINT, HandleSigInt);
}

}  // namespace robot_session_examples
