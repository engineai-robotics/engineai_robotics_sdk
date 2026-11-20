// motion_mode_manager: list the current motion and request a listed transition.
// A target that is not in available_transitions is not sent.
//
//   ./motion_mode_manager --domain-id 0 --list
//   ./motion_mode_manager --domain-id 0 --target-motion joint_bridge --timeout 8
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
  std::cerr << "Usage: " << argv0
            << " --domain-id N [--interface IF] [--network-address ADDR]\n"
            << "       [--list] [--target-motion NAME] [--timeout SEC] [--json]\n"
            << "  --list is the default when --target-motion is omitted.\n"
            << "  A target outside available_transitions is not sent.\n";
}

std::string JsonEscape(const std::string& text) {
  std::string out;
  for (char ch : text) {
    if (ch == '\\' || ch == '"') {
      out.push_back('\\');
    }
    out.push_back(ch);
  }
  return out;
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

  bool as_json = false;
  std::string target;
  int timeout_sec = 8;
  for (size_t i = 0; i < rest.size(); ++i) {
    const std::string& arg = rest[i];
    if (arg == "--list") {
      continue;
    } else if (arg == "--json") {
      as_json = true;
    } else if (arg == "--target-motion" && i + 1 < rest.size()) {
      target = rest[++i];
    } else if (arg == "--timeout" && i + 1 < rest.size()) {
      if (!robot_session_examples::ReadStrictInt(rest[++i], &timeout_sec, "timeout")) {
        return 2;
      }
    } else {
      PrintUsage(argv[0]);
      return 2;
    }
  }
  if (timeout_sec <= 0) {
    std::cerr << "timeout must be > 0\n";
    return 2;
  }

  auto session = robot_session_examples::CreateSessionOrExit(options);
  if (!robot_session_examples::WaitForMotionSample(session.get(), timeout_sec * 1000)) {
    robot_session_examples::PrintNoRobotDataHint();
    session->close();
    return 1;
  }

  const std::string current = robot_session_examples::MotionOrNone(*session);
  const auto transitions = session->motion().available_transitions();
  std::vector<std::string> names;
  if (transitions) {
    names = *transitions;
  }

  auto print_state = [&](const std::string& result, const std::string& detail) {
    if (!as_json) {
      std::cout << "current=" << current << "\n";
      std::cout << "available_transitions:";
      if (names.empty()) {
        std::cout << " (none)";
      }
      for (const auto& name : names) {
        std::cout << " " << name;
      }
      std::cout << "\n";
      std::cout << "result=" << result;
      if (!detail.empty()) {
        std::cout << " " << detail;
      }
      std::cout << "\n";
      return;
    }
    std::cout << "{\"current\": \"" << JsonEscape(current) << "\", \"available\": [";
    for (size_t i = 0; i < names.size(); ++i) {
      if (i != 0) {
        std::cout << ", ";
      }
      std::cout << "\"" << JsonEscape(names[i]) << "\"";
    }
    std::cout << "], \"target\": " << (target.empty() ? "null" : ("\"" + JsonEscape(target) + "\""))
              << ", \"result\": \"" << JsonEscape(result) << "\", \"detail\": \"" << JsonEscape(detail) << "\"}\n";
  };

  if (target.empty()) {
    print_state("listed", "");
    session->close();
    return names.empty() ? 1 : 0;
  }
  if (!transitions) {
    print_state("no_transitions", "available_transitions has no sample");
    session->close();
    return 1;
  }
  if (robot_session_examples::MotionIs(*session, target)) {
    print_state("already_active", "current motion already matches");
    session->close();
    return 0;
  }
  if (std::find(names.begin(), names.end(), target) == names.end()) {
    print_state("unreachable", "target is not in available_transitions; request was not sent");
    session->close();
    return 1;
  }

  const auto err = session->motion().request_transition(target);
  if (!as_json) {
    std::cout << "request_transition -> " << robot_session_examples::ErrName(err) << "\n";
  }
  if (err != engineai_sdk::SessionError::kOk) {
    print_state("send_failed", robot_session_examples::ErrName(err));
    session->close();
    return 1;
  }

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_sec);
  while (std::chrono::steady_clock::now() < deadline) {
    if (robot_session_examples::MotionIs(*session, target)) {
      print_state("switched", "current motion matches target");
      session->close();
      return 0;
    }
    if (!as_json) {
      std::cout << "waiting current=" << robot_session_examples::MotionOrNone(*session) << "\n";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  print_state("timeout", "current=" + robot_session_examples::MotionOrNone(*session));
  session->close();
  return 1;
}
