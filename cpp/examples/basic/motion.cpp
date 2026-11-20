// basic_motion: motion().current_motion / available_transitions / request_transition
//
//   ./basic_motion --domain-id 0
//   ./basic_motion --domain-id 0 --motion joint_bridge [--timeout SEC]
//
// Safety when the target is joint_bridge: this mode commands the joints
// directly. The standing pose is not necessarily stable, so hoist the robot
// before use. Both feet must be on the ground before entering walk.

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "engineai_sdk/robot.h"

#include "support/common_cli.hpp"

namespace {

void PrintUsage(const char* argv0) {
  std::cerr << "Usage: " << argv0 << " --domain-id N [--interface IF] [--network-address ADDR] [--motion NAME] [--timeout SEC]\n";
}

void PrintMotion(engineai_sdk::Robot* session, const char* label) {
  std::cout << "==== " << label << " ====\n";
  std::cout << "current=" << robot_session_examples::MotionOrNone(*session) << "\n";
  std::cout << "available_transitions:";
  if (const auto transitions = session->motion().available_transitions()) {
    for (const auto& name : *transitions) {
      std::cout << " " << name;
    }
  } else {
    std::cout << " (no sample)";
  }
  std::cout << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  if (robot_session_examples::WantsHelp(argc, argv)) {
    PrintUsage(argv[0]);
    return 0;
  }

  engineai_sdk::SessionOptions options;
  std::vector<std::string> rest;
  if (!robot_session_examples::ParseSessionArgs(argc, argv, &options, &rest)) {
    PrintUsage(argv[0]);
    return 2;
  }

  std::string motion;
  int timeout_sec = 8;
  for (size_t i = 0; i < rest.size(); ++i) {
    const std::string& arg = rest[i];
    if (arg == "--motion" && i + 1 < rest.size()) {
      motion = rest[++i];
    } else if (arg == "--timeout" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadStrictInt(rest[++i], &timeout_sec, "timeout")) {
        return 2;
      }
    } else {
      PrintUsage(argv[0]);
      return 2;
    }
  }

  auto session = robot_session_examples::CreateSessionOrExit(options);
  const bool have_motion = robot_session_examples::WaitForMotionSample(session.get());
  PrintMotion(session.get(), "before");
  if (!have_motion) {
    robot_session_examples::PrintNoRobotDataHint();
    session->close();
    return 1;
  }

  if (motion.empty()) {
    session->close();
    return 0;
  }

  if (robot_session_examples::MotionIs(*session, motion)) {
    std::cout << "already in " << motion << "\n";
    session->close();
    return 0;
  }

  const auto transitions = session->motion().available_transitions();
  if (!transitions || std::find(transitions->begin(), transitions->end(), motion) == transitions->end()) {
    std::cerr << "motion '" << motion << "' not in available_transitions; request was not sent\n";
    PrintMotion(session.get(), "not sent");
    session->close();
    return 1;
  }

  const auto err = session->motion().request_transition(motion);
  std::cout << "motion.request_transition(" << motion
            << ") -> " << robot_session_examples::ErrName(err) << "\n";
  if (err != engineai_sdk::SessionError::kOk) {
    session->close();
    return 1;
  }

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_sec);
  while (std::chrono::steady_clock::now() < deadline) {
    if (robot_session_examples::MotionIs(*session, motion)) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  PrintMotion(session.get(), "after");
  const bool ok = robot_session_examples::MotionIs(*session, motion);
  if (!ok) {
    std::cerr << "timeout waiting for motion=" << motion << "\n";
  }
  session->close();
  return ok ? 0 : 1;
}
